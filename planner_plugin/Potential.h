#ifndef POTENTIAL_H
#define POTENTIAL_H

#include <Eigen/Dense>

#include "types.h"

class Potential
{
public:

    Potential(){}

    static float calculatePotentialGradient(float dist)
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

    static float calculateGoalPotentialGradient()
    {
        return -goal_potential_gradient_;
    }

    virtual float getPotentialGradientAt(Location x) const = 0;

    virtual Location getPotentialGradientDirectionAt(Location x) const = 0;

protected:
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
const float Potential::goal_potential_gradient_ = 10.0F;


class CirclePotential : public Potential
{
public:
    CirclePotential(Location center, float radius): 
        center_{center}, radius_{radius}
    {}

    float getPotentialGradientAt(Location x) const override
    { 
        return calculatePotentialGradient((x-center_).norm() - radius_); 
    }

    Location getPotentialGradientDirectionAt(Location x) const override
    { 
        return (x - center_).normalized(); 
    }
    
private:
    Location center_;
    float radius_;
};

#endif