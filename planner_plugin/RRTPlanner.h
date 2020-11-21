#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include "Planner.h"

class RRTPlanner : public Planner
{
public:

    enum class ConnectResult
    {
        Running            = 0,
        ReachedGoal        = 1,
        ReachedConnectGoal = 2,
        Collided           = 3
    };

    RRTPlanner(): goal_bias_{0.3}, goal_node_{nullptr}
    {}

    bool plan(int max_iterations) override;
    
    std::vector<std::vector<double> > getPath() const override;

private:
    ConnectResult extendTo(Config connect_goal);
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
        potential_params::goal_potential_gradient = potential_params::potential_gradient_goal;
        Config connect_goal = reachToGoal()? world_ptr_->getGoal() 
                                : world_ptr_->getRandomConfigNotInCollision();
        ConnectResult result = extendTo(connect_goal);
        if (result == ConnectResult::ReachedGoal)
        {
            std::cout<<"Reached node at : "<<goal_node_->getConfiguration().transpose()<<std::endl;
            std::cout<<"Iterations : "<<iteration<<std::endl;
            return true;
        }
    }
    std::cout<<"Failed search after reaching max iterations of "<<max_iterations<<std::endl;
    return false;
}

RRTPlanner::ConnectResult RRTPlanner::extendTo(Config connect_goal)
{
    std::function<float(Node)> distance_to_connect_goal = [&connect_goal](Node node) 
        { return (node.getConfiguration() - connect_goal).norm(); };
    std::shared_ptr<Node> closest_node = search_tree_.getClosestNode(distance_to_connect_goal);

    int steps_allowed = 1;
    ConnectResult result(ConnectResult::Running);
    while (result == ConnectResult::Running && steps_allowed > 0)
    {
        std::vector<Config> intermediate_steps;
        Config new_config;
        bool success = world_ptr_->stepTowards(closest_node->getConfiguration(), connect_goal, intermediate_steps, new_config);
        if(!success)
        {
            result = ConnectResult::Collided;
            break;
        }

        closest_node = search_tree_.addChildNode(closest_node, new_config);
        closest_node->setIntermediateSteps(intermediate_steps);

        if(!silent)
        {
            Config n1 = closest_node->getConfiguration();
            viz_objects_permanent.push_back(drawConfiguration(world_ptr_->env_, n1, Blue));
        }

        if(world_ptr_->isGoal(new_config))
        {
            result = ConnectResult::ReachedGoal;
            goal_node_ = closest_node;
        }
        else if(distance_to_connect_goal(*closest_node) < node_distance_tolerance)
        {
            result = ConnectResult::ReachedConnectGoal;
        }
        --steps_allowed;
    }
    
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
