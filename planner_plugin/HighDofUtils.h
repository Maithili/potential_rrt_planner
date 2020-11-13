#ifndef HIGHDOF_UTILS_H
#define HIGHDOF_UTILS_H

#include <openrave/environment.h>

std::vector<double> jacobianTransposeApply( boost::multi_array<double,2> matrix , std::vector<float> vector)
{
    std::vector<double> result;
    result.reserve(3);
    float accumulator = 0;
    for (int i=0;i<matrix.shape()[1];i++)
    {
        for(int j=0;j<3;j++)
        {
            accumulator += matrix[j][i]*vector[j];
        }
    result.push_back(accumulator);
    accumulator = 0;
    }
    return result;
}

OpenRAVE::AABB KinBodyComputeEnabledAABB(OpenRAVE::KinBodyConstPtr kb)
{
   OpenRAVE::Vector vmin, vmax;
   bool binitialized = false;
   OpenRAVE::AABB ab;
   const std::vector<OpenRAVE::KinBody::LinkPtr> & links = kb->GetLinks();
   for (std::vector<OpenRAVE::KinBody::LinkPtr>::const_iterator
        it=links.begin(); it!=links.end(); it++)
   {
      // TODO (maithili) : Find out why i had to disable this
      // if (!(*it)->IsEnabled())
      //    continue;
      ab = (*it)->ComputeAABB();
      if((ab.extents.x == 0)&&(ab.extents.y == 0)&&(ab.extents.z == 0)) {
         continue;
      }
      OpenRAVE::Vector vnmin = ab.pos - ab.extents;
      OpenRAVE::Vector vnmax = ab.pos + ab.extents;
      if( !binitialized ) {
         vmin = vnmin;
         vmax = vnmax;
         binitialized = true;
      }
      else {
         if( vmin.x > vnmin.x ) {
            vmin.x = vnmin.x;
         }
         if( vmin.y > vnmin.y ) {
            vmin.y = vnmin.y;
         }
         if( vmin.z > vnmin.z ) {
            vmin.z = vnmin.z;
         }
         if( vmax.x < vnmax.x ) {
            vmax.x = vnmax.x;
         }
         if( vmax.y < vnmax.y ) {
            vmax.y = vnmax.y;
         }
         if( vmax.z < vnmax.z ) {
            vmax.z = vnmax.z;
         }
      }
   }
   if( !binitialized ) {
      ab.pos = kb->GetTransform().trans;
      ab.extents = OpenRAVE::Vector(0,0,0);
   }
   else {
      ab.pos = (OpenRAVE::dReal)0.5 * (vmin + vmax);
      ab.extents = vmax - ab.pos;
   }
   return ab;
}

void getOccupancyGrid(cd_grid * g_obs, double* pose_world_gsdf, double cube_extent, OpenRAVE::EnvironmentBasePtr& env)
{
   OpenRAVE::KinBodyPtr cube = OpenRAVE::RaveCreateKinBody(env);
   cube->SetName("cube");
   {
      std::vector<OpenRAVE::AABB> vaabbs(1);
      vaabbs[0].extents = OpenRAVE::Vector(cube_extent, cube_extent, cube_extent); /* extents = half side lengths */
      cube->InitFromBoxes(vaabbs, 1);
   }
   
   /* add the cube */
   #if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,8,0)
      env->Add(cube);
   #else
      env->AddKinBody(cube);
   #endif

   int collisions = 0;
   double pose_cube[7];
   
   OpenRAVE::Transform t;
   /* go through the grid, testing for collision as we go;
      * collisions are HUGE_VAL, free are 1.0 */
   for (size_t idx=0; idx<g_obs->ncells; idx++)
   {
      
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
      if (env->CheckCollision(cube))
      {
         *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
         collisions++;
      }
   }

   RAVELOG_INFO("Found %d/%d collisions!\n", collisions, (int)(g_obs->ncells));
   
   /* remove cube */
   env->Remove(cube);
}

void addGridMetaData(Sdf& sdf, OpenRAVE::RobotBasePtr& robot, double cube_extent)
{
   double aabb_padding = 0.2;
   /* copy in name */
   if (strlen(robot->GetName().c_str())+1 > sizeof(sdf.kinbody_name))
      throw OpenRAVE::openrave_exception("ugh, we currently don't support long kinbody names!");
   strcpy(sdf.kinbody_name, robot->GetName().c_str());

   OpenRAVE::geometry::aabb< OpenRAVE::dReal > aabb;
   /* compute aabb when object is at world origin */
   {
      OpenRAVE::KinBody::KinBodyStateSaver statesaver(robot);
      robot->SetTransform(OpenRAVE::Transform());
      aabb = KinBodyComputeEnabledAABB(robot);
   }
   RAVELOG_INFO("    pos: %f %f %f\n", aabb.pos[0], aabb.pos[1], aabb.pos[2]);
   RAVELOG_INFO("extents: %f %f %f\n", aabb.extents[0], aabb.extents[1], aabb.extents[2]);

   int gsdf_sizearray[3];
   /* calculate dimension sizes (number of cells) */
   for (int i=0; i<3; i++)
   {
      /* 0.15m padding (was 0.3m) on each side
       * (this is the radius of the biggest herb2 spehere)
       * (note: extents are half the side lengths!) */
      gsdf_sizearray[i] = (int) ceil((aabb.extents[i]+aabb_padding) / cube_extent);
      RAVELOG_INFO("gsdf_sizearray[%d]: %d\n", i, gsdf_sizearray[i]);
   }
   
   /* Create a new grid located around the current kinbody;
    * per-dimension sizes set above */
   double temp = 1.0; /* free space */
   int err = cd_grid_create_sizearray(&sdf.grid, &temp, sizeof(double), 3, gsdf_sizearray);
   if (err) throw OpenRAVE::openrave_exception("Not enough memory for distance field!");
   
   /* set side lengths */
   for (int i=0; i<3; i++)
      sdf.grid->lengths[i] = gsdf_sizearray[i] * 2.0 * cube_extent;
   // RAVELOG_INFO("sdf_.grid->lengths: %s\n", sf_vector(sdf_.grid->lengths,3).c_str());
   
   /* set pose of grid w.r.t. kinbody frame */
   cd_kin_pose_identity(sdf.pose_kinbody);
   for (int i=0; i<3; i++)
      sdf.pose_kinbody[i] = aabb.pos[i] - 0.5 * sdf.grid->lengths[i];
   // RAVELOG_INFO("pose_gsdf: %s\n", sf_vector(sdf_.pose_kinbody,7).c_str());
}

bool readSdfFromCache(char * cache_filename, Sdf& sdf, OpenRAVE::RobotBasePtr robot)
{
   FILE * fp;
   RAVELOG_INFO("Reading SDF data for KinBody '%s' from file %s ...\n",
      robot->GetName().c_str(), cache_filename);
   fp = fopen(cache_filename, "rb");
   if (!fp) { RAVELOG_ERROR("could not read from file!\n"); return false;; }
   
   /* check file size */
   fseek(fp, 0L, SEEK_END);
   if (ftell(fp) != sdf.grid->cell_size * sdf.grid->ncells)
   {
      RAVELOG_ERROR("cached file size %lu bytes doesn't match expected size %lu! recomputing ...\n",
         ftell(fp), sdf.grid->cell_size * sdf.grid->ncells);
      fclose(fp);
      return false;
   }
   fseek(fp, 0L, SEEK_SET);
   /* read grid data */
   int i = fread(sdf.grid->data, sdf.grid->cell_size, sdf.grid->ncells, fp);
   if (i != sdf.grid->ncells)
   {
      RAVELOG_ERROR("error, couldn't all read the sdf data from the file!\n");
      fclose(fp);
      return false;
   }
   fclose(fp);
   return true;
}

double robotSelfCollisionGradient()
{
      // /* consider effects from all other spheres (i.e. self collision) */
      // for (active_sphere_index2=0,sact2=r->spheres; active_sphere_index2<r->n_spheres; active_sphere_index2++,sact2=sact2->next)
      // {
      //    /*if (active_sphere_index2 >= r->n_spheres_active) continue;*/ /* SKIP ACTIVE-INACTIVE SPHERE GRADIENTS FOR NOW */
         
      //    /* skip spheres on the same link (their Js would be identical anyways) */
      //    if (sact->robot_linkindex == sact2->robot_linkindex) continue;
         
      //    /* compute vector from our (active_sphere_index) location to their (active_sphere_index2) location;
      //     * whats the deal with this crazy sphere indexing? */
      //    cd_mat_memcpy(v_from_other, r->sphere_poss + ti*(r->n_spheres_active*3) + active_sphere_index*3, 3, 1);
      //    if (active_sphere_index2<r->n_spheres_active)
      //       cd_mat_sub(v_from_other, r->sphere_poss + ti*(r->n_spheres_active*3) + active_sphere_index2*3, 3, 1);
      //    else
      //       cd_mat_sub(v_from_other, r->sphere_poss_inactive + (active_sphere_index2-r->n_spheres_active)*3, 3, 1);
      //    dist = cblas_dnrm2(3, v_from_other,1);
         
      //    /* skip spheres far enough away from us */
      //    if (dist > sact->radius + sact2->radius + r->epsilon_self) continue;
         
      //    if (c_grad)
      //    {
      //       /* make unit vector (g_grad) away from other sphere */
      //       g_grad[0] = v_from_other[0] / dist;
      //       g_grad[1] = v_from_other[1] / dist;
      //       g_grad[2] = v_from_other[2] / dist;
      //    }
         
      //    /* actual distance between sphere surfaces */
      //    dist -= sact->radius + sact2->radius;
         
      //    if (costp)
      //    {
      //       /* compute the cost */
      //       if (dist < 0.0)
      //          cost_sphere += x_vel_norm * r->obs_factor_self * (0.5 * r->epsilon_self - dist);
      //       else
      //          cost_sphere += x_vel_norm * r->obs_factor_self * (0.5/r->epsilon_self) * (dist - r->epsilon_self) * (dist - r->epsilon_self);
      //       /*printf("   from extra sphere-sphere interaction, sphere cost now: %f\n", cost_sphere);*/
      //    }
         
      //    if (c_grad)
      //    {
      //       /* convert sdf g_grad to x_grad (w.r.t. cost) according to dist */
      //       cd_mat_memcpy(x_grad, g_grad, 3, 1);
      //       if (dist < 0.0)
      //          cd_mat_scale(x_grad, 3, 1, -1.0);
      //       else if (dist < r->epsilon_self)
      //          cd_mat_scale(x_grad, 3, 1, dist/r->epsilon_self - 1.0);
      //       cd_mat_scale(x_grad, 3, 1, x_vel_norm * r->obs_factor_self);
            
      //       /* subtract from x_grad vector projection onto x_vel */
      //       if (x_vel_norm > 0.000001)
      //       {
      //          proj = cblas_ddot(3, x_grad,1, x_vel,1) / (x_vel_norm * x_vel_norm);
      //          cblas_daxpy(3, -proj, x_vel,1, x_grad,1);
      //       }
            
      //       /* J2 = J - jacobian of other sphere*/
      //       cd_mat_memcpy(r->J2, r->sphere_jacs + ti*r->n_spheres_active*3*c->n + active_sphere_index*3*c->n, 3, c->n);
      //       if (active_sphere_index2<r->n_spheres_active)
      //          cd_mat_sub(r->J2, r->sphere_jacs + ti*r->n_spheres_active*3*c->n + active_sphere_index2*3*c->n, 3, c->n);
            
      //       /* multiply into c_grad through JT */
      //       cblas_dgemv(CblasRowMajor, CblasTrans, 3, c->n,
      //          1.0, r->J2,c->n, x_grad,1, 1.0, c_grad,1);
      //    }
      // }
}

int replace_1_to_0(double * val, void * rptr)
{
   if (*val == 1.0)
   {
      *val = 0.0;
      return 1;
   }
   return 0;
}

#endif  // HIGHDOF_UTILS_H