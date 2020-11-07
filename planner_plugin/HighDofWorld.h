#ifndef HIGHDOF_WORLD_H
#define HIGHDOF_WORLD_H

#include <openrave/environment.h>
#include "World.h"
#include "HighDofUtils.h"

class PotentialField
{
public:

   PotentialField(OpenRAVE::EnvironmentBasePtr& env): env_{env}
   {
      calculateSDFs();
   }
   
   std::vector<float> getPotentialGradientAt(Sphere sphere, OpenRAVE::EnvironmentBasePtr& env);
   static float calculatePotentialGradient(float dist);

protected:
   void calculateSDFs();
   OpenRAVE::EnvironmentBasePtr& env_;
   Sdf sdf_;
};

class HighDofWorld : public World
{
public:

   HighDofWorld(OpenRAVE::EnvironmentBasePtr& env): World{env}, field_{env}
   {
      if (env->GetRobot("BarrettWAM")) robot_spheres_.setBarretWAM(env);
   }

   Config stepTowards(Config from, Config towards, std::vector<Config>& intermediate_steps) override;

private:
   Config getObstacleGradient(Config cfg);
   void updateSphereLocations();
   PotentialField field_;
   Spheres robot_spheres_;
};

Config HighDofWorld::stepTowards(Config from, Config towards, std::vector<Config>& intermediate_steps)
{
   Config step;
   intermediate_steps.clear();
   for (int i=0; i<num_baby_steps; ++i)
   {
      updateSphereLocations();
      step = from + getObstacleGradient(from);
      step += (towards-from).normalized() * potential_params::goal_potential_gradient;
      intermediate_steps.push_back(step);
      from = step;
   }
   return step;
}

void HighDofWorld::updateSphereLocations()
{
   OpenRAVE::RobotBasePtr robot = env_->GetRobot(robot_spheres_.robotname);
   for (Sphere s: robot_spheres_.list)
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

   for (Sphere& s : robot_spheres_.list)
   {
      std::vector<float> gradient = field_.getPotentialGradientAt(s, this->env_);
      boost::multi_array<double,2> jacobian;
      s.getJacobian(jacobian, robot);
      Config config_gradient;
      copyToEigen(jacobianTransposeApply(jacobian, gradient), config_gradient);
      net_config_gradient += config_gradient;
   }

   return net_config_gradient;
}

std::vector<float> PotentialField::getPotentialGradientAt(Sphere sphere, OpenRAVE::EnvironmentBasePtr& env)
{
   double g[3];
   std::vector<float> gradient;
   float distance = sphereDistanceInField(sdf_, sphere, gradient, env);
   double norm = sqrt(gradient[0]*gradient[0] + gradient[1]*gradient[1] + gradient[2]*gradient[2]);
   gradient[0] *= calculatePotentialGradient(distance)/norm;
   gradient[1] *= calculatePotentialGradient(distance)/norm;
   gradient[2] *= calculatePotentialGradient(distance)/norm;
   return gradient;
}

//This is copied from FlatPotential
float PotentialField::calculatePotentialGradient(float dist)
{
    if(dist > potential_params::max_dist)
        return 0.0F;
    else
    {   
        float factor = potential_params::max_potential_gradient
                     * pow(potential_params::min_dist,potential_params::potential_power);
        if (dist < potential_params::min_dist)
            return potential_params::max_potential_gradient;
        else
        {
            return (factor/pow(dist,potential_params::potential_power));
        }
    }
}

void PotentialField::calculateSDFs()
{
   int i;
   int err;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   OpenRAVE::KinBodyPtr robot = env_->GetRobot("BarretWAM");
   /* parameters */
   double cube_extent = 0.02;
   double aabb_padding = 0.2;
   char * cache_filename = "sdf_tablemug.dat";
   /* other */
   double temp;
   OpenRAVE::KinBodyPtr cube;
   size_t idx;
   struct cd_grid * g_obs;
   int gsdf_sizearray[3];
   OpenRAVE::geometry::aabb< OpenRAVE::dReal > aabb;
   double pose_world_gsdf[7];
   double pose_cube[7];
   Sdf sdf_;
   struct timespec ticks_tic;
   struct timespec ticks_toc;
   int sdf_data_loaded;
   
   /* lock environment */
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->env_->GetMutex());
  
   /* copy in name */
   if (strlen(robot->GetName().c_str())+1 > sizeof(sdf_.kinbody_name))
      throw OpenRAVE::openrave_exception("ugh, we currently don't support long kinbody names!");
   strcpy(sdf_.kinbody_name, robot->GetName().c_str());

   /* compute aabb when object is at world origin */
   {
      OpenRAVE::KinBody::KinBodyStateSaver statesaver(robot);
      robot->SetTransform(OpenRAVE::Transform());
      aabb = KinBodyComputeEnabledAABB(robot);
   }
   RAVELOG_INFO("    pos: %f %f %f\n", aabb.pos[0], aabb.pos[1], aabb.pos[2]);
   RAVELOG_INFO("extents: %f %f %f\n", aabb.extents[0], aabb.extents[1], aabb.extents[2]);

   /* calculate dimension sizes (number of cells) */
   for (i=0; i<3; i++)
   {
      /* 0.15m padding (was 0.3m) on each side
       * (this is the radius of the biggest herb2 spehere)
       * (note: extents are half the side lengths!) */
      gsdf_sizearray[i] = (int) ceil((aabb.extents[i]+aabb_padding) / cube_extent);
      RAVELOG_INFO("gsdf_sizearray[%d]: %d\n", i, gsdf_sizearray[i]);
   }
   
   /* Create a new grid located around the current kinbody;
    * per-dimension sizes set above */
   temp = 1.0; /* free space */
   err = cd_grid_create_sizearray(&sdf_.grid, &temp, sizeof(double), 3, gsdf_sizearray);
   if (err) throw OpenRAVE::openrave_exception("Not enough memory for distance field!");
   
   /* set side lengths */
   for (i=0; i<3; i++)
      sdf_.grid->lengths[i] = gsdf_sizearray[i] * 2.0 * cube_extent;
   // RAVELOG_INFO("sdf_.grid->lengths: %s\n", sf_vector(sdf_.grid->lengths,3).c_str());
   
   /* set pose of grid w.r.t. kinbody frame */
   cd_kin_pose_identity(sdf_.pose_kinbody);
   for (i=0; i<3; i++)
      sdf_.pose_kinbody[i] = aabb.pos[i] - 0.5 * sdf_.grid->lengths[i];
   // RAVELOG_INFO("pose_gsdf: %s\n", sf_vector(sdf_.pose_kinbody,7).c_str());
   
   /* we don't have sdf grid data yet */
   sdf_data_loaded = 0;
   
//    /* attempt to load the sdf from file */
//    if (cache_filename) do
//    {
//       FILE * fp;
//       RAVELOG_INFO("Reading SDF data for KinBody '%s' from file %s ...\n",
//          robot->GetName().c_str(), cache_filename);
//       fp = fopen(cache_filename, "rb");
//       if (!fp) { RAVELOG_ERROR("could not read from file!\n"); break; }
      
//       /* check file size */
//       fseek(fp, 0L, SEEK_END);
//       if (ftell(fp) != sdf_.grid->cell_size * sdf_.grid->ncells)
//       {
//          RAVELOG_ERROR("cached file size %lu bytes doesn't match expected size %lu! recomputing ...\n",
//             ftell(fp), sdf_.grid->cell_size * sdf_.grid->ncells);
//          fclose(fp);
//          break;
//       }
//       fseek(fp, 0L, SEEK_SET);
//       /* read grid data */
//       i = fread(sdf_.grid->data, sdf_.grid->cell_size, sdf_.grid->ncells, fp);
//       if (i != sdf_.grid->ncells)
//       {
//          RAVELOG_ERROR("error, couldn't all read the sdf data from the file!\n");
//          fclose(fp);
//          break;
//       }
//       fclose(fp);
//       sdf_data_loaded = 1;
//    } while (0);
   
   if (!sdf_data_loaded)
   {
      /* create the obstacle grid (same size as sdf_.grid that we already calculated) */
      err = cd_grid_create_copy(&g_obs, sdf_.grid);
      if (err)
      {
         cd_grid_destroy(sdf_.grid);
         throw OpenRAVE::openrave_exception("Not enough memory for distance field!");
      }
      
      /* start timing voxel grid computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);
      
      /* create the cube */
      cube = OpenRAVE::RaveCreateKinBody(this->env_);
      cube->SetName("cube");
      
      /* set its dimensions */
      {
         std::vector<OpenRAVE::AABB> vaabbs(1);
         vaabbs[0].extents = OpenRAVE::Vector(cube_extent, cube_extent, cube_extent); /* extents = half side lengths */
         cube->InitFromBoxes(vaabbs, 1);
      }
      
      /* add the cube */
    #if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,8,0)
        this->env_->Add(cube);
    #else
        this->env_->AddKinBody(cube);
    #endif

      sdf_.originInWorld(pose_world_gsdf, this->env_);

      int collisions = 0;
      
      /* go through the grid, testing for collision as we go;
       * collisions are HUGE_VAL, free are 1.0 */
      RAVELOG_INFO("Computing occupancy grid ...\n");
      for (idx=0; idx<g_obs->ncells; idx++)
      {
         OpenRAVE::Transform t;
         
         if (idx % 100000 == 0)
            RAVELOG_INFO("  idx=%d (%5.1f%%)...\n", (int)idx, (100.0*((double)idx)/((double)g_obs->ncells)));
         
         /* set cube location */
         t.identity();
         cd_kin_pose_identity(pose_cube);
         cd_grid_center_index(g_obs, idx, pose_cube);
         cd_kin_pose_compose(pose_world_gsdf, pose_cube, pose_cube);
         t.trans.x = pose_cube[0];
         t.trans.y = pose_cube[1];
         t.trans.z = pose_cube[2];
         t.rot.y = pose_cube[3];
         t.rot.z = pose_cube[4];
         t.rot.w = pose_cube[5];
         t.rot.x = pose_cube[6];
         cube->SetTransform(t);
         
         /* do collision check */
         if (this->env_->CheckCollision(cube))
         {
            *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
            collisions++;
         }
      }

      RAVELOG_INFO("Found %d/%d collisions!\n", collisions, (int)(g_obs->ncells));
      
      /* remove cube */
      this->env_->Remove(cube);
      
      /* stop timing voxel grid computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      RAVELOG_INFO("Total voxel grid computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      /* start timing flood fill computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

      /* we assume the point at in the very corner x=y=z is free*/
      RAVELOG_INFO("performing flood fill ...\n");
      idx = 0;
      cd_grid_flood_fill(g_obs, idx, 0, (int (*)(void *, void *))replace_1_to_0, 0);
      
      /* change any remaining 1.0 cells to HUGE_VAL (assumed inside of obstacles) */
      for (idx=0; idx<g_obs->ncells; idx++)
         if (*(double *)cd_grid_get_index(g_obs, idx) == 1.0)
            *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
      
      /* stop timing flood fill computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      RAVELOG_INFO("total flood fill computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      /* start timing sdf computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

      /* compute the signed distance field (in the module instance) */
      RAVELOG_DEBUG("computing signed distance field ...\n");
      cd_grid_double_bin_sdf(&sdf_.grid, g_obs);

      /* stop timing sdf computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      RAVELOG_DEBUG("total sdf computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      /* we no longer need the obstacle grid */
      cd_grid_destroy(g_obs);
      
      /* if we were passed a cache_filename, save what we just computed! */
      if (cache_filename)
      {
         FILE * fp;
         RAVELOG_INFO("Saving sdf data to file %s ...\n", cache_filename);
         fp = fopen(cache_filename, "wb");
         i = fwrite(sdf_.grid->data, sdf_.grid->cell_size, sdf_.grid->ncells, fp);
         fclose(fp);
         if (i != sdf_.grid->ncells)
            RAVELOG_ERROR("Error, couldn't write the sdf data to the file!\n");
      }
   }
}


#endif  // HIGHDOF_WORLD_H