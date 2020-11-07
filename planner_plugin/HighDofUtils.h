#ifndef HIGHDOF_UTILS_H
#define HIGHDOF_UTILS_H

#include <openrave/environment.h>

std::vector<double> jacobianTransposeApply( boost::multi_array<double,2> matrix , std::vector<float> vector)
{
    // MATH_ASSERT(matrix.dimensionality == 2);
    // MATH_ASSERT(matrix.shape()[0] == vector.size());
    std::vector<double> result;
    result.reserve(3);
    float accumulator = 0;
    // for (int i=0;i<result.size();i++)
    for (int i=0;i<3;i++)
    {
        for(int j=0;j<matrix.shape()[0];j++)
        {
            accumulator += matrix[j][i]*vector[j];
        }
    result.push_back(accumulator);
    accumulator = 0;
    }
    return result;
}

float sphereDistanceInField(Sdf sdf, Sphere sphere, std::vector<float>& grad, OpenRAVE::EnvironmentBasePtr& env)
{
   double dist = potential_params::max_dist;
   double sphere_center_on_grid[3];
   cd_mat_memcpy(sphere_center_on_grid, sphere.pos_worldframe, 3, 1);
   sdf.toGridFrame(sphere_center_on_grid, env);

   /* get sdf value (from interp) */   
   int err = cd_grid_double_interp(sdf.grid, sphere_center_on_grid, &dist);
   if (err)
      return potential_params::max_dist; /* not inside of this distance field at all! */
   dist -= sphere.radius;
      

   /* get sdf gradient */
   /* this will be a unit vector away from closest obs */
   double* gradient;
   cd_grid_double_grad(sdf.grid, sphere_center_on_grid, gradient);
   
   // TODO(maithili) Should this not be transformed as direction only?
   sdf.toWorldFrame(gradient, env, true);

   std::vector<float> gradient_out(gradient, gradient+3);
   grad = gradient_out;

   return dist;
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

OpenRAVE::AABB KinBodyComputeEnabledAABB(OpenRAVE::KinBodyConstPtr kb)
{
   OpenRAVE::Vector vmin, vmax;
   bool binitialized = false;
   OpenRAVE::AABB ab;
   const std::vector<OpenRAVE::KinBody::LinkPtr> & links = kb->GetLinks();
   for (std::vector<OpenRAVE::KinBody::LinkPtr>::const_iterator
        it=links.begin(); it!=links.end(); it++)
   {
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