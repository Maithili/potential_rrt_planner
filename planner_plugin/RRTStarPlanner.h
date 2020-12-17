#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include "Planner.h"

class RRTStarPlanner : public RRTPlanner
{
public:

    RRTStarPlanner():gamma_{(step_size*15)}, rewire_radius_{gamma_}, num_vertices_{1}
    {}

private:
    World::ExtendResult extendTo(Config connect_goal) override;
    void getOptimalParent(Config new_config, 
                          std::vector<std::shared_ptr<Node> > close_nodes, 
                          std::shared_ptr<Node>& optimal_parent_out, 
                          std::vector<Config>& optimal_steps_out,
                          float& optimal_distance);
    void rewire(std::vector<std::shared_ptr<Node> > close_nodes, 
                const std::shared_ptr<Node> new_node);
    
    void adaptRadius()
    {
        rewire_radius_ = gamma_ * pow((log(num_vertices_)/num_vertices_), 1/config_dim);
        if (rewire_radius_ < 0.1) rewire_radius_ = 0.1;
    }

    const float gamma_;
    int num_vertices_;
    float rewire_radius_;
};

World::ExtendResult RRTStarPlanner::extendTo(Config connect_goal)
{
    std::shared_ptr<Node> closest_node = search_tree_.getClosestNode(connect_goal);
    std::vector<Config> steps;
    float distance;
    World::ExtendResult result = world_ptr_->stepTowards(closest_node->getConfiguration(), connect_goal, steps, distance);
    if (result != World::ExtendResult::CollidedWithoutExtension)
    {
        Config new_config = steps.back();
        std::vector<std::shared_ptr<Node> > close_nodes = search_tree_.getNodesWithinDistance(new_config, rewire_radius_);
        getOptimalParent(new_config, close_nodes, closest_node, steps, distance);
        std::shared_ptr<Node> new_node;
        new_node = search_tree_.addChildNode(closest_node, steps, distance);
        ++num_vertices_;
        rewire(close_nodes, new_node);
    }
    if(result == World::ExtendResult::ReachedPlannerGoal)
        goal_node_ = closest_node;

    adaptRadius();

    return result;
}

void RRTStarPlanner::getOptimalParent(Config new_config, 
                                      std::vector<std::shared_ptr<Node> > close_nodes, 
                                      std::shared_ptr<Node>& optimal_parent_out, 
                                      std::vector<Config>& optimal_steps_out,
                                      float& optimal_distance)
{    
    for (std::shared_ptr<Node> n : close_nodes)
    {
        float min_possible_distance = n->getDistanceFromRoot() + (n->getConfiguration()-new_config).norm();
        if(min_possible_distance < optimal_distance)
        {
            World::ExtendResult result;
            std::vector<Config> steps;
            float distance;
            result = world_ptr_->stepTowards(n->getConfiguration(), new_config, steps, distance, true);
            distance += n->getDistanceFromRoot();

            if(result == World::ExtendResult::ReachedConnectGoal && distance < optimal_distance)
            {
                optimal_parent_out = n;
                optimal_distance = distance;
                optimal_steps_out = steps;
            }
        }
    }
}

void RRTStarPlanner::rewire(std::vector<std::shared_ptr<Node> > close_nodes, 
                            const std::shared_ptr<Node> new_node)
{
    float new_node_dist = new_node->getDistanceFromRoot();
    for (std::shared_ptr<Node> n : close_nodes)
    {
        float min_possible_distance = new_node_dist + (n->getConfiguration()-new_node->getConfiguration()).norm();
        if(min_possible_distance < n->getDistanceFromRoot())
        {
            World::ExtendResult result;
            std::vector<Config> steps;
            float distance;
            result = world_ptr_->stepTowards(new_node->getConfiguration(), n->getConfiguration(), steps, distance, true);
            distance += new_node_dist;

            if(result == World::ExtendResult::ReachedConnectGoal && distance < n->getDistanceFromRoot())
            {
                new_node->addChild(n);
                if(! n->getParent()->removeChild(n)) {std::cout<<"Cannot remove child"<<std::endl; continue;}
                n->setParent(new_node);
                n->setIntermediateSteps(steps);
                float distance_decrement = n->getDistanceFromRoot() - distance;
                n->setDistance(distance);
                std::vector<std::shared_ptr<Node> > close_nodes;
                std::vector<std::shared_ptr<Node> > nodes_to_check = n->getChildren();
                std::shared_ptr<Node> nodeptr;
                while(!nodes_to_check.empty())
                {
                    nodeptr = nodes_to_check.back();
                    nodes_to_check.pop_back();
                    nodeptr->setDistance(nodeptr->getDistanceFromRoot() - distance_decrement);
                    auto children = nodeptr->getChildren();
                    nodes_to_check.insert(nodes_to_check.end(), children.begin(), children.end());
                }
            }
        }
    }
}

#endif