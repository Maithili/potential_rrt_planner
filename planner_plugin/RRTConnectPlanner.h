#ifndef RRT_CONNECT_PLANNER_H
#define RRT_CONNECT_PLANNER_H

#include "Planner.h"

class RRTConnectPlanner : public Planner
{
public:

    enum class ConnectResult
    {
        Running            = 0,
        ReachedGoal        = 1,
        Collided           = 2
    };

    RRTConnectPlanner(): 
        goal_node_{nullptr, nullptr}
    {}

    bool plan(int max_iterations) override;
    
    std::vector<std::vector<double> > getPath() const override;

private:
    std::shared_ptr<Node> extendTo(Config connect_goal, int tree_idx);
    ConnectResult connectTo(Config connect_goal, int tree_idx);

    ConfigTree search_tree_[2];
    std::shared_ptr<Node> goal_node_[2];
};

bool RRTConnectPlanner::plan(int max_iterations)
{
    search_tree_[0].setRoot(world_ptr_->getStart());
    search_tree_[1].setRoot(world_ptr_->getGoal());
    Config connect_goal = world_ptr_->getGoal();
    int conn_tree_ = 0;
    for (int iteration=0; iteration<max_iterations; ++iteration)
    {
        Config random_config = world_ptr_->getRandomConfigNotInCollision();
        goal_node_[1-conn_tree_] = extendTo(random_config, 1-conn_tree_);
        if (goal_node_[1-conn_tree_]==nullptr) continue;
        ConnectResult connect_result = connectTo(goal_node_[1-conn_tree_]->getConfiguration(), conn_tree_);
        if (connect_result == ConnectResult::ReachedGoal)
        {
            std::cout<<"Nodes matched at : "<<goal_node_[0]->getConfiguration().transpose()
                                   <<" and "<<goal_node_[1]->getConfiguration().transpose()<<std::endl;
            std::cout<<"Iterations : "<<iteration<<std::endl;
            return true;
        }
        conn_tree_ = 1 - conn_tree_;
    }
    std::cout<<std::endl;
    std::cout<<"Failed search after reaching max iterations of "<<max_iterations<<std::endl;
    return false;
}

std::vector<std::vector<double> > RRTConnectPlanner::getPath() const
{
    std::vector<std::vector<double> > path = search_tree_[0].getPathToRoot(goal_node_[0], world_ptr_->env_);
    std::reverse(path.begin(), path.end());
    std::vector<std::vector<double> > second_path = search_tree_[1].getPathToRoot(goal_node_[1], world_ptr_->env_);
    path.insert(path.end(),second_path.begin(),second_path.end());
    return path;
}

std::shared_ptr<Node> RRTConnectPlanner::extendTo(Config connect_goal, int tree_idx)
{
    std::function<float(Node)> distance_to_goal = [&connect_goal](Node node) 
        { return (node.getConfiguration() - connect_goal).norm(); };
    ConfigTree* tree = &search_tree_[tree_idx];
    std::shared_ptr<Node> closest_node = tree->getClosestNode(distance_to_goal);

    int steps_allowed = 1;

    std::vector<Config> intermediate_steps;
    Config new_config;
    bool success = world_ptr_->stepTowards(closest_node->getConfiguration(), connect_goal, intermediate_steps, new_config);
    
    if(!success)
    {
        return nullptr;
    }

    closest_node = tree->addChildNode(closest_node, new_config);
    closest_node->setIntermediateSteps(intermediate_steps);

    if(!silent)
    {
        Config n1 = closest_node->getConfiguration();
        viz_objects_permanent.push_back(drawConfiguration(world_ptr_->env_, n1, Blue));
    }

    return closest_node;
}

RRTConnectPlanner::ConnectResult RRTConnectPlanner::connectTo(Config connect_goal, int tree_idx)
{
    ConfigTree* tree = &search_tree_[tree_idx];
    std::function<float(Node)> distance_to_connect_goal = [&connect_goal](Node node) 
        { return (node.getConfiguration() - connect_goal).norm(); };
    std::shared_ptr<Node> closest_node = tree->getClosestNode(distance_to_connect_goal);

    int steps_allowed = 100;
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

        closest_node = tree->addChildNode(closest_node, new_config);
        closest_node->setIntermediateSteps(intermediate_steps);
        std::cout<<"Temp"<<closest_node->getConfiguration()<<std::endl;

        if(!silent)
        {   
            Config n1 = closest_node->getConfiguration();
            viz_objects_permanent.push_back(drawConfiguration(world_ptr_->env_, n1, Blue));
        }

        if(distance_to_connect_goal(*closest_node) < node_distance_tolerance)
        {
            result = ConnectResult::ReachedGoal;
            goal_node_[tree_idx] = closest_node;
        }
        --steps_allowed;
    }

    std::cout<<steps_allowed<<" remaining steps"<<std::endl;
    return result;
}

#endif
