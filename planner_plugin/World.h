#ifndef WORLD_H
#define WORLD_H

#include <openrave/environment.h>

#include "Potential.h"

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

    bool isInCollision(Config config)
    {
        OpenRAVE::RobotBasePtr robot = env_->GetRobot("PR2");
        std::vector<double> config_vector(config.data(), config.data()+config.rows());
        config_vector.push_back(0.0);
        robot->SetActiveDOFValues(config_vector);
        return (   env_->CheckCollision(robot)
                || robot->CheckSelfCollision());
    };

    Config getRandomConfig()
    {
        Config random_config(start_location_);
        for (int idx = 0; idx<config_dim; ++idx)
        {
            float f = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
            random_config(idx,0) = ((1-f) * lower_limits_(idx,0)) + (f * (upper_limits_(idx,0)));
        }
        potential_params::goal_potential_gradient = potential_params::potential_gradient_rand;
        return random_config;
    }

    Config getRandomConfigNotInCollision();

    virtual Config stepTowards(Config from, Config towards) = 0;

    bool isGoal(Config config) const
    {
        return ((config - goal_location_).norm() < node_distance_tolerance);
    }
    
    OpenRAVE::EnvironmentBasePtr env_;
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
    {};

    Config stepTowards(Config from, Config towards)
    {
        Config goal_gradient = (towards - from).normalized();
        return (from + goal_gradient * step_size);
    }
};

class EuclideanWorldWithPotential : public World
{
public:
    EuclideanWorldWithPotential(OpenRAVE::EnvironmentBasePtr& env): World(env)
    {
        populatePotentials();
    };

    Config stepTowards(Config from, Config towards)
    {
        Config goal_gradient = (towards - from).normalized();
        goal_gradient *= Potential::calculateGoalPotentialGradient();
        Config obstacle_gradient = getObstacleGradient(from);
        Config step = (goal_gradient + obstacle_gradient).normalized() * step_size;
        return (from + step);
    }

private:
    Config getObstacleGradient(Config q)
    {
        Config gradient;
        for (auto& obstacle : obstacle_geometries_)
        {
            gradient += obstacle.getPotentialGradientAt(q);
        }
        return gradient;
    }

    void populatePotentials()
    {
        std::vector<OpenRAVE::KinBodyPtr> bodies;
        env_->GetBodies(bodies);
        OpenRAVE::KinBodyPtr obstacles;
        std::vector<OpenRAVE::KinBody::LinkPtr> links;
        obstacles = env_->GetKinBody("obstacles");
        links = obstacles->GetLinks();
        for (OpenRAVE::KinBody::LinkPtr l : links)
        {
            obstacle_geometries_.push_back(Potential(l->GetGeometries()));
        }
    }

    std::vector<Potential> obstacle_geometries_;
};

class ConfigWorld : public World
{

};

class ConfigWorldWithPotential : public World
{

};

#endif