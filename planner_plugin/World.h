#ifndef WORLD_H
#define WORLD_H

#include <openrave/environment.h>

#include "PrecomputedPotential.h"

class World
{

public:
    World(OpenRAVE::EnvironmentBasePtr& env): env_{env} {}
    
    void setStart(double* start) {start_location_ = Config(start);}            
    void setGoal(double* goal) {goal_location_ = Config(goal);}            
    void setLowerLimits(double* lower) {lower_limits_ = Config(lower);}            
    void setUpperLimits(double* upper) {upper_limits_ = Config(upper);}            
    
    Config getStart() const {return start_location_;}
    Config getGoal() const {return goal_location_;}
    Config getLowerLimits() const {return lower_limits_;}
    Config getUpperLimits() const {return upper_limits_;}

    enum class ExtendResult
    {
        Extended                 = 0,
        ReachedConnectGoal       = 1,
        CollidedAfterExtension   = 2,
        CollidedWithoutExtension = 3,
        ReachedPlannerGoal       = 4
    };

    virtual ExtendResult stepTowards(Config from, Config towards, std::vector<Config>& steps) = 0;
    
    bool isInCollision(Config config)
    {
        std::vector<OpenRAVE::RobotBasePtr> robots;
        env_->GetRobots(robots);
        OpenRAVE::RobotBasePtr robot = robots[0];
        std::vector<double> config_vector(config.data(), config.data()+config.rows());
        if (config_dim < 3) config_vector.push_back(0.0);
        robot->SetActiveDOFValues(config_vector);
        return (   env_->CheckCollision(robot)
                || robot->CheckSelfCollision());
    }

    Config getRandomConfig()
    {
        Config random_config = Config::Zero();
        for (int idx = 0; idx<config_dim; ++idx)
        {
            float f = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
            random_config(idx,0) = ((1-f) * lower_limits_[idx]) + (f * (upper_limits_[idx]));
        }
        potential_params::goal_potential_gradient = potential_params::potential_gradient_rand;
        return random_config;
    }

    Config getRandomConfigNotInCollision()
    {
        Config random_config;
        do
        {
            random_config = getRandomConfig();
        }
        while(isInCollision(random_config));
        return random_config;
    }


    bool isGoal(Config config) const
    {
        return ((config - goal_location_).norm() < node_distance_tolerance);
    }
    
    OpenRAVE::EnvironmentBasePtr env_;
    
    bool configInLimits(Config cfg)
    {
        for(int i=0; i<config_dim; ++i)
        {
            if(cfg[i] > upper_limits_[i])
                return false;
            if(cfg[i] < lower_limits_[i])
                return false;
        }
        return true;
    }

protected:
    Config start_location_;
    Config goal_location_;
    Config lower_limits_;
    Config upper_limits_;
};

class EuclideanWorld : public World
{
public:
    EuclideanWorld(OpenRAVE::EnvironmentBasePtr& env): World(env)
    {
        RAVELOG_INFO("Constructed euclidean world");
    }

    ExtendResult stepTowards(Config from, Config towards, std::vector<Config>& steps)
    {
        steps.clear();
        Config goal_gradient = (towards - from).normalized();
        for(int i = 0; i < num_baby_steps; ++i)
        {
            Config step = from + goal_gradient * step_size;
            if(isInCollision(step))
            {
                if(!silent) viz_objects_permanent.push_back(drawConfiguration(this->env_, step, Red, 3));
                return i==0 ? ExtendResult::CollidedWithoutExtension : 
                              ExtendResult::CollidedAfterExtension;
            }
            if(!silent)
            {
                viz_objects_permanent.push_back(drawConfiguration(this->env_, step, Blue, 2));
                viz_objects_permanent.push_back(drawEdge(this->env_, step, from));
            }
            from = step;
            steps.push_back(from);
            if(isGoal(step))
            {
                return ExtendResult::ReachedPlannerGoal;
            }
            if((step-towards).norm() < node_distance_tolerance)
            {
                return ExtendResult::ReachedConnectGoal;
            }
        }
        return ExtendResult::Extended;
    }
};

class EuclideanWorldWithPotential : public World
{
public:
    EuclideanWorldWithPotential(OpenRAVE::EnvironmentBasePtr& env): 
    World(env), potential_(env, 12.0, 12.0, 0.05)
    {
        RAVELOG_INFO("Constructed euclidean world with potentials");
    };

    ExtendResult stepTowards(Config from, Config towards, std::vector<Config>& steps)
    {
        steps.clear();
        Config step;
        for(int i=0; i<num_baby_steps; ++i)
        {
            bool step_success = smallStep(from, towards, step);
            if(!step_success)
            {
                if(!silent) viz_objects_permanent.push_back(drawConfiguration(this->env_, step, Pale, 3));
                return i==0 ? ExtendResult::CollidedWithoutExtension : 
                              ExtendResult::CollidedAfterExtension;
            }
            if(isInCollision(step))
            {
                if(!silent) viz_objects_permanent.push_back(drawConfiguration(this->env_, step, Red, 3));
                return i==0 ? ExtendResult::CollidedWithoutExtension : 
                              ExtendResult::CollidedAfterExtension;
            }
            if(!silent)
            {
                viz_objects_permanent.push_back(drawConfiguration(this->env_, step, Blue, 2));
                viz_objects_permanent.push_back(drawEdge(this->env_, step, from));
            }
            from = step;
            steps.push_back(from);
            if(isGoal(from))
            {
                return ExtendResult::ReachedPlannerGoal;
            }
            if((from-towards).norm() < node_distance_tolerance)
            {
                return ExtendResult::ReachedConnectGoal;
            }
        }
        return ExtendResult::Extended;
    }

private:

    bool smallStep(Config from, Config towards, Config& step_out)
    {
        Config goal_gradient = (towards - from).normalized();
        goal_gradient *= calculateGoalPotentialGradient();
        Config obstacle_gradient = getObstacleGradient(from);
        double norm = (goal_gradient + obstacle_gradient).norm();
        Config step = (goal_gradient + obstacle_gradient).normalized() * inner_step_size;
        step_out = (from + step);
        return norm > 1e-2;
    }

    Config getObstacleGradient(Config q)
    {
        Config gradient = Config::Zero();
        gradient.topRows(space_dim) = potential_.getPotentialGradientAt(q.topRows(space_dim));
        return gradient;
    }

    PrecomputedPotential potential_;
};

#endif