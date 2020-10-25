#ifndef POTENTIAL_H
#define POTENTIAL_H

#include <Eigen/Dense>

#include "types.h"

class Potential
{
public:

    Potential(std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geom): geometries_{geom}
    {}
    
    Location getPotentialGradientAt(Location x);
    static float calculateGoalPotentialGradient();
    static float calculatePotentialGradient(float dist);

protected:

    std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geometries_;
    static const float max_dist_;
    static const float min_dist_;
    static const float max_potential_gradient_;
    static const float potential_power_;
    static const float goal_potential_gradient_;
};

const float Potential::max_dist_                = 2.5F;
const float Potential::min_dist_                = 0.4F;
const float Potential::max_potential_gradient_  = 10.0F;
const float Potential::potential_power_         = 2.0F;
const float Potential::goal_potential_gradient_ = 10.0F;

namespace
{
int sign(float x)
{
    if(x < 1e-6)
        return 0;
    else return (x>0?1:-1);
}

Location getPotentialGradientForCircle(Location x, double radius, OpenRAVE::geometry::RaveVector<double> center_vec)
{
    Location center;
    center << center_vec.x, center_vec.y;

    float distance = (x-center).norm() - radius;

    Location gradient = (x-center).normalized();
    gradient *= Potential::calculatePotentialGradient(distance);

    return gradient;
}

Location getPotentialGradientForBox(Location x, 
                                    OpenRAVE::geometry::RaveVector<double> extents_in, 
                                    OpenRAVE::geometry::RaveVector<double> translation, 
                                    float rotation)
{
    Location center;
    center << translation.x, translation.y;
    Location extents;
    center << extents_in.x, extents_in.y;
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(rotation), -sin(rotation), sin(rotation), cos(rotation);
    Location rel = rotation_matrix*(x-center);
    
    Location potential_distances;
    potential_distances << std::max(fabs(rel[0]) - extents[0], 0.0) * sign(rel[0]),
                           std::max(fabs(rel[1]) - extents[1], 0.0) * sign(rel[1]);
    Location gradient = potential_distances.normalized();
    gradient *= Potential::calculatePotentialGradient(potential_distances.norm());
    return gradient;
}
}

Location Potential::getPotentialGradientAt(Location x)
{
    Location gradient = Location::Zero();
    for(auto geomtry : geometries_)
    {
        Location geom_grad;
        if( (geomtry->GetType() == OpenRAVE::GT_Cylinder) || (geomtry->GetType() == OpenRAVE::GT_Sphere))
        {
            geom_grad = getPotentialGradientForCircle(x, geomtry->GetCylinderRadius(), geomtry->GetTransform().trans);
        }
        // else if( (geomtry->GetType() == OpenRAVE::GT_Box))
        // {
        //     geom_grad = getPotentialGradientForBox(x, geomtry->GetBoxExtents(), geomtry->GetTransform().trans, geomtry->GetTransform().rot.x);
        // }
        if (geom_grad.norm() > gradient.norm())
            gradient = geom_grad;
    }

    return gradient;
}

float Potential::calculateGoalPotentialGradient()
{
    return goal_potential_gradient_;
}

float Potential::calculatePotentialGradient(float dist)
{
    if(dist > max_dist_)
        return 0.0F;
    else
    {   
        float factor = max_potential_gradient_*pow(min_dist_,potential_power_);
        if (dist < min_dist_)
            return max_potential_gradient_;
        else
        {
            return (factor/pow(dist,potential_power_));
        }
    }
}

#endif