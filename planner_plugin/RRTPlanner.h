#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include "Planner.h"

class RRTPlanner : public Planner
{
public:

    RRTPlanner(): goal_bias_{0.01}, goal_node_{nullptr}
    {}

    virtual bool plan(int max_iterations) override;
    
    virtual std::vector<std::vector<double> > getPath() const override;

    void drawTree() 
    { 
        viz_tree.clear();
        search_tree_.draw(world_ptr_->env_, viz_tree); 
    }

protected:
    virtual World::ExtendResult extendTo(Config connect_goal);
    bool reachToGoal();

    float goal_bias_;
    ConfigTree search_tree_;
    std::shared_ptr<Node> goal_node_;
};

bool RRTPlanner::plan(int max_iterations)
{
    search_tree_.setRoot(world_ptr_->getStart());
    for (int iteration=0; iteration<max_iterations; ++iteration)
    {
        if (!silent && iteration%20 == 0) drawTree();
        potential_params::goal_potential_gradient = potential_params::potential_gradient_goal;
        Config connect_goal = reachToGoal()? world_ptr_->getGoal() 
                                : world_ptr_->getRandomConfigNotInCollision();
        World::ExtendResult result = extendTo(connect_goal);
        if (result == World::ExtendResult::ReachedPlannerGoal)
        {
            if (!silent) drawTree();
            std::cout<<"Reached node at : "<<goal_node_->getConfiguration().transpose()<<std::endl;
            std::cout<<"Iterations : "<<iteration<<std::endl;
            return true;
        }
    }
    std::cout<<"Failed search after reaching max iterations of "<<max_iterations<<std::endl;
    return false;
}

World::ExtendResult RRTPlanner::extendTo(Config connect_goal)
{
    std::shared_ptr<Node> closest_node = search_tree_.getClosestNode(connect_goal);
    std::vector<Config> steps;
    float distance;
    World::ExtendResult result = world_ptr_->stepTowards(closest_node->getConfiguration(), connect_goal, steps, distance);
    if(result != World::ExtendResult::CollidedWithoutExtension)
    {
        if (steps.empty())
        {
            std::cout<<"Something's wrong..Getting ok status but no nodes from stepTowards!!!!!"<<std::endl;
        }
        closest_node = search_tree_.addChildNode(closest_node, steps, distance);
    }
    if(result == World::ExtendResult::ReachedPlannerGoal)
        goal_node_ = closest_node;

    return result;
}

std::vector<std::vector<double> > RRTPlanner::getPath() const
{
    std::vector<std::vector<double> > path = search_tree_.getPathToRoot(goal_node_, world_ptr_->env_);    
    std::reverse(path.begin(), path.end());
    return path;
}

bool RRTPlanner::reachToGoal()
{
    float random_fraction = ( static_cast<float>(rand()) / static_cast<float>(RAND_MAX) );
    return(random_fraction < goal_bias_);
}

#endif
