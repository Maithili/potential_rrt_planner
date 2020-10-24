#ifndef POTENTIAL_H
#define POTENTIAL_H

#include <Eigen/Dense>

#include "types.h"

class Potential
{
public:

    Potential(OpenRAVE::KinBody::Link::GeometryPtr geom): geometry_{geom}
    {}
    
    Location getPotentialGradientAt(Location x);
    static float calculateGoalPotentialGradient();
    static float calculatePotentialGradient(float dist);

protected:

    OpenRAVE::KinBody::Link::GeometryPtr geometry_;
    static const float max_dist_;
    static const float min_dist_;
    static const float max_potential_gradient_;
    static const float potential_power_;
    static const float goal_potential_gradient_;
};

const float Potential::max_dist_                = 1.5F;
const float Potential::min_dist_                = 0.1F;
const float Potential::max_potential_gradient_  = 10.0F;
const float Potential::potential_power_         = 2.0F;
const float Potential::goal_potential_gradient_ = 1.0F;

namespace
{
Location getPotentialGradientForCircle(Location x, double radius, OpenRAVE::geometry::RaveVector<double> center_vec)
{
    Location center;
    center << center_vec.x, center_vec.y;

    float distance = (x-center).norm() - radius;

    Location gradient = (x-center).normalized();
    gradient *= Potential::calculatePotentialGradient(distance);

    return gradient;
}
}

Location Potential::getPotentialGradientAt(Location x)
{
    if( (geometry_->GetType() == OpenRAVE::GT_Cylinder) || (geometry_->GetType() == OpenRAVE::GT_Sphere))
        return (getPotentialGradientForCircle(x, geometry_->GetCylinderRadius(), geometry_->GetTransform().trans));
    else
        return Location();
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