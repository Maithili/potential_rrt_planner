#ifndef HIGHDOF_WORLD_H
#define HIGHDOF_WORLD_H

#include <openrave/environment.h>
#include <openrave/openrave.h>
#include "World.h"
#include "HighDofUtils.h"

class PotentialField
{
public:

   PotentialField(OpenRAVE::EnvironmentBasePtr& env): env_{env}
   {
      calculateSDFs();
   }
   
   void getPotentialGradientAt(const Sphere& sphere, std::vector<float>& gradient);

protected:
   void calculateSDFs();
   OpenRAVE::EnvironmentBasePtr& env_;
   Sdf sdf_;
};

class HighDofWorld : public World
{
public:

   HighDofWorld(OpenRAVE::EnvironmentBasePtr& env): World{env}, field_{env_}
   {
      if (env->GetRobot("BarrettWAM")) robot_spheres_.setBarretWAM(env);
      RAVELOG_INFO("Constructed high DOF world");
   }

   Config stepTowards(Config from, Config towards, std::vector<Config>& intermediate_steps) override;

private:
   Config getObstacleGradient(Config cfg);
   void updateSphereLocations(Config cfg);
   PotentialField field_;
   Spheres robot_spheres_;
};

Config HighDofWorld::stepTowards(Config from, Config towards, std::vector<Config>& intermediate_steps)
{
   Config step;
   // std::cout<<"Stepping from : "<<from.transpose()<<std::endl;
   // std::cout<<"Stepping to : "<<towards.transpose()<<std::endl;
   intermediate_steps.clear();
   for (int i=0; i<num_baby_steps; ++i)
   {
      updateSphereLocations(from);
      showRobot(env_);
      step = getObstacleGradient(from);
      step += (towards-from).normalized() * potential_params::goal_potential_gradient;
      step.normalize();
      if(!configInLimits(step+from) || isInCollision(step+from))
         break;
      intermediate_steps.push_back(step);
      from = step;
   }
   showRobotAt(from, env_);
   return from;
}

void HighDofWorld::updateSphereLocations(Config cfg)
{
   OpenRAVE::RobotBasePtr robot = env_->GetRobot(robot_spheres_.robotname);
   std::vector<double> config_vector;
   copyToVector(cfg, config_vector);
   robot->SetActiveDOFValues(config_vector);
   for (Sphere& s: robot_spheres_.list)
   {
      OpenRAVE::Transform t = robot->GetLink(s.linkname)->GetTransform();
      OpenRAVE::Vector v = t * OpenRAVE::Vector(s.pos_linkframe); /* copies 3 values */
      s.pos_worldframe[0] = v[0]; s.pos_worldframe[1] = v[1]; s.pos_worldframe[2] = v[2];
   }
}

Config HighDofWorld::getObstacleGradient(Config cfg)
{
   OpenRAVE::RobotBasePtr robot = env_->GetRobot(robot_spheres_.robotname);
   std::vector<double> cfg_stl(cfg.data(), cfg.data()+cfg.rows());
   robot->SetActiveDOFValues(cfg_stl);

   Config net_config_gradient = Config::Zero();

   for (const Sphere& s : robot_spheres_.list)
   {
      std::vector<float> gradient{0.0, 0.0, 0.0};
      field_.getPotentialGradientAt(s, gradient);
      
      boost::multi_array<double,2> jacobian;
      s.getJacobian(jacobian, robot);
      std::vector<double> grad_vec = jacobianTransposeApply(jacobian, gradient);
      Config config_gradient;
      copyToEigen(grad_vec, config_gradient);
      net_config_gradient += config_gradient;
   }

   return net_config_gradient;
}

void PotentialField::getPotentialGradientAt(const Sphere& sphere, std::vector<float>& gradient)
{
   double distance = potential_params::max_dist;
   double sphere_center_on_grid[3];
   cd_mat_memcpy(sphere_center_on_grid, sphere.pos_worldframe, 3, 1);
   sdf_.pointToGridFrame(sphere_center_on_grid, env_);

   /* get sdf value (from interp) */   
   int err = cd_grid_double_interp(sdf_.grid, sphere_center_on_grid, &distance);
   if (err)
      return;

   distance -= sphere.radius;
   /* get sdf gradient */
   /* this will be a unit vector away from closest obs */
   double grad[3];
   grad[0] = 0.0;grad[1]=0.0;grad[2]=0.0;
   cd_grid_double_grad(sdf_.grid, sphere_center_on_grid, grad);
   sdf_.pointToWorldFrame(grad, env_);

   double norm = sqrt(grad[0]*grad[0] + grad[1]*grad[1] + grad[2]*grad[2]);
   float multiplier = calculatePotentialGradient(distance)/norm;
   gradient[0] = grad[0] * multiplier;
   gradient[1] = grad[1] * multiplier;
   gradient[2] = grad[2] * multiplier;
}

void PotentialField::calculateSDFs()
{
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   std::vector<OpenRAVE::RobotBasePtr> robots;
   env_->GetRobots(robots);
   OpenRAVE::RobotBasePtr robot = robots.front();
   /* parameters */
   double cube_extent = 0.02;
   char * cache_filename = "sdf_tablemug.dat";
   /* other */
   struct cd_grid * g_obs;
   double pose_world_gsdf[7];
   // struct timespec ticks_tic;
   // struct timespec ticks_toc;
   
   /* lock environment */
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->env_->GetMutex());
  
   addGridMetaData(this->sdf_, robot, cube_extent);

   bool sdf_data_loaded = readSdfFromCache(cache_filename, this->sdf_, robot);
   
   if (!sdf_data_loaded)
   {
      /* create the obstacle grid (same size as sdf_.grid that we already calculated) */
      int err = cd_grid_create_copy(&g_obs, sdf_.grid);
      if (err)
      {
         cd_grid_destroy(sdf_.grid);
         throw OpenRAVE::openrave_exception("Not enough memory for distance field!");
      }
      
      /* start timing voxel grid computation */
      // clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);
      
      RAVELOG_INFO("Computing occupancy grid ...\n");
      this->sdf_.originInWorld(pose_world_gsdf, this->env_);   
      getOccupancyGrid(g_obs, pose_world_gsdf, cube_extent, this->env_);
      
      // /* stop timing voxel grid computation */
      // clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      // CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      // RAVELOG_INFO("Total voxel grid computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      // /* start timing flood fill computation */
      // clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

      /* we assume the point at in the very corner x=y=z is free*/
      RAVELOG_INFO("performing flood fill ...\n");
      size_t idx = 0;
      cd_grid_flood_fill(g_obs, idx, 0, (int (*)(void *, void *))replace_1_to_0, 0);
      
      /* change any remaining 1.0 cells to HUGE_VAL (assumed inside of obstacles) */
      for (idx=0; idx<g_obs->ncells; idx++)
         if (*(double *)cd_grid_get_index(g_obs, idx) == 1.0)
            *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
      
      // /* stop timing flood fill computation */
      // clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      // CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      // RAVELOG_INFO("total flood fill computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      // /* start timing sdf computation */
      // clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

      /* compute the signed distance field (in the module instance) */
      RAVELOG_DEBUG("computing signed distance field ...\n");
      cd_grid_double_bin_sdf(&sdf_.grid, g_obs);

      // /* stop timing sdf computation */
      // clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      // CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      // RAVELOG_DEBUG("total sdf computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      /* we no longer need the obstacle grid */
      cd_grid_destroy(g_obs);
      
      /* if we were passed a cache_filename, save what we just computed! */
      if (cache_filename)
      {
         FILE * fp;
         RAVELOG_INFO("Saving sdf data to file %s ...\n", cache_filename);
         fp = fopen(cache_filename, "wb");
         int i = fwrite(sdf_.grid->data, sdf_.grid->cell_size, sdf_.grid->ncells, fp);
         fclose(fp);
         if (i != sdf_.grid->ncells)
            RAVELOG_ERROR("Error, couldn't write the sdf data to the file!\n");
      }
   }
}


#endif  // HIGHDOF_WORLD_H