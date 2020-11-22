#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include "Planner.h"

class RRTStarPlanner : public RRTPlanner
{
public:

    RRTStarPlanner():rewire_radius{1.0}
    {}

private:
    World::ExtendResult extendTo(Config connect_goal) override;
    void rewire(Config new_config);

    float rewire_radius;
};

World::ExtendResult RRTStarPlanner::extendTo(Config connect_goal)
{
    std::shared_ptr<Node> closest_node = search_tree_.getClosestNode(connect_goal);
    std::vector<Config> steps;
    World::ExtendResult result = world_ptr_->stepTowards(closest_node->getConfiguration(), connect_goal, steps);
    if (result != World::ExtendResult::CollidedWithoutExtension)
        rewire(steps.back());
    if(result == World::ExtendResult::ReachedPlannerGoal)
        goal_node_ = closest_node;

    return result;
}

void RRTStarPlanner::rewire(Config new_config)
{
    std::function<float(Node)> distance_to_connect_goal = [&new_config](Node node) 
        { return (node.getConfiguration() - new_config).norm(); };
    std::vector<std::shared_ptr<Node> > close_nodes = search_tree_.getNodesWithinDistance(distance_to_connect_goal, rewire_radius);
    std::shared_ptr<Node> optimal_parent;
    float optimal_distance = HUGE_VAL;
    std::vector<std::vector<Config> > optimal_steps;
    for (std::shared_ptr<Node> n : close_nodes)
    {
        float distance = n->getDistanceFromRoot() + (n->getConfiguration()-new_config).norm();
        if(distance < optimal_distance)
        {
            World::ExtendResult result;
            std::vector<std::vector<Config> > all_steps;
            do 
            {
                std::vector<Config> steps;
                result = world_ptr_->stepTowards(n->getConfiguration(), new_config, steps);
                all_steps.push_back(steps);
            }
            while (result == World::ExtendResult::Extended);

            if(result == World::ExtendResult::ReachedConnectGoal)
            {
                optimal_parent = n;
                optimal_distance = distance;
                optimal_steps = all_steps;
            }
        }
    }
    for (std::vector<Config> steps : optimal_steps)
    {
        optimal_parent = search_tree_.addChildNode(optimal_parent, steps.back());
        optimal_parent->setIntermediateSteps(steps);
    }
}

#endif