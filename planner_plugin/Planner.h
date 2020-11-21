#ifndef PLANNER_H
#define PLANNER_H

#include <random>

#include "HighDofWorld.h"
#include "types.h"
#include "ConfigTree.h"

class Planner
{
public:
    Planner()
    {}

    virtual bool plan(int max_iterations) = 0;

    void setWorld(World* world_ptr_in)
    {
        world_ptr_ = world_ptr_in;
    }

    virtual std::vector<std::vector<double> > getPath() const = 0;

protected:
    World* world_ptr_;
};

#endif
