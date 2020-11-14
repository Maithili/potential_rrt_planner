#ifndef FLAT_POTENTIAL_H
#define FLAT_POTENTIAL_H

#include <Eigen/Dense>

#include "types.h"

class FlatPotential
{
public:

    FlatPotential()
    {}

    FlatPotential(std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geom): geometries_{geom}
    {}
    
    Location getPotentialGradientAt(Location x);

protected:

    std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geometries_;
};

namespace
{
float sign(float x)
{
    return ( x>0 ? 1.0 : -1.0);
}

Location getPotentialGradientForCircle(Location x, double radius, OpenRAVE::geometry::RaveVector<double> center_vec)
{
    Location center;
    center << center_vec.x, center_vec.y;

    float distance = (x-center).norm() - radius;

    Location gradient = (x-center).normalized();
    gradient *= calculatePotentialGradient(distance);

    return gradient;
}

Location getPotentialGradientForBox(Location point, 
                                    OpenRAVE::geometry::RaveVector<double> extents_in, 
                                    OpenRAVE::geometry::RaveVector<double> translation, 
                                    float rotation)
{
    Location center;
    center << translation.x, translation.y;
    Location extents;
    extents << extents_in.x, extents_in.y;
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(rotation), -sin(rotation), 
                       sin(rotation),  cos(rotation);
    Location rel;
    rel.topRows(2) = rotation_matrix.transpose() * (point-center).topRows(2);
    
    Location potential_distances;
    potential_distances << std::max(fabs(rel[0]) - extents[0], 0.0) * sign(rel[0]),
                           std::max(fabs(rel[1]) - extents[1], 0.0) * sign(rel[1]);
    Location gradient = Location::Zero();
    gradient.topRows(2) = rotation_matrix * potential_distances.topRows(2).normalized();
    gradient *= calculatePotentialGradient(potential_distances.norm());
    return gradient;
}
}

Location FlatPotential::getPotentialGradientAt(Location x)
{
    Location gradient = Location::Zero();
    Location sum_gradient = Location::Zero();
    for(auto geomtry : geometries_)
    {
        Location geom_grad = Location::Zero();
        if( (geomtry->GetType() == OpenRAVE::GT_Cylinder) || (geomtry->GetType() == OpenRAVE::GT_Sphere))
        {
            geom_grad = getPotentialGradientForCircle(x, geomtry->GetCylinderRadius(), geomtry->GetTransform().trans);
            sum_gradient += geom_grad;
        }
        else if( (geomtry->GetType() == OpenRAVE::GT_Box))
        {
            float angle = sqrt(OpenRAVE::geometry::axisAngleFromQuat(geomtry->GetTransform().rot).lengthsqr3());
            geom_grad = getPotentialGradientForBox(x, geomtry->GetBoxExtents(), geomtry->GetTransform().trans, angle);
            sum_gradient += geom_grad;
        }
        if (geom_grad.norm() > gradient.norm())
            gradient = geom_grad;
    }

    return sum_gradient;
    // return gradient;
}

#endif  // FLAT_POTENTIAL_H