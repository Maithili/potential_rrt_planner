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

    bool isInCollision(Config config);

    Config getRandomConfig()
    {
        Config random_config(start_location_);
        for (int idx = 0; idx<config_dim; ++idx)
        {
            float f = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
            random_config(idx,0) = ((1-f) * lower_limits_(idx,0)) + (f * (upper_limits_(idx,0)));
        }
        return random_config;
    }

    Config getRandomConfigNotInCollision();

    virtual Config stepTowards(Config from, Config towards) = 0;

    bool isGoal(Config config) const
    {
        return ((config - goal_location_).norm() < node_distance_tolerance);
    }

protected:
    OpenRAVE::EnvironmentBasePtr env_;
    Config start_location_;
    Config goal_location_;
    Config lower_limits_;
    Config upper_limits_;
};

class EuclideanWorld : public World
{

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
        return (from + goal_gradient + obstacle_gradient);
    }

private:
    Config getObstacleGradient(Config q)
    {
        Config gradient;
        for (auto& obstacle : obstacle_potentials_)
        {
            gradient += obstacle.getPotentialGradientAt(q) * obstacle.getPotentialGradientDirectionAt(q);
        }
        return gradient;
    }

    void populatePotentials(){};

    std::vector<Potential> obstacle_potentials_;
};

class ConfigWorld : public World
{

};

class ConfigWorldWithPotential : public World
{

};

#endif