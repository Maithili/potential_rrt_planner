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

    void drawTree() 
    { 
        viz_tree.clear();
        search_tree_[0].draw(world_ptr_->env_, viz_tree); 
        search_tree_[1].draw(world_ptr_->env_, viz_tree); 
    }

private:
    std::shared_ptr<Node> extendTo(Config connect_goal, int tree_idx);
    World::ExtendResult connectTo(Config connect_goal, int tree_idx);

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
        if (!silent && iteration%20 == 0) drawTree();
        Config random_config = world_ptr_->getRandomConfigNotInCollision();
        goal_node_[1-conn_tree_] = extendTo(random_config, 1-conn_tree_);
        if (goal_node_[1-conn_tree_]==nullptr) continue;
        World::ExtendResult connect_result = connectTo(goal_node_[1-conn_tree_]->getConfiguration(), conn_tree_);
        if (connect_result == World::ExtendResult::ReachedConnectGoal)
        {
            if (!silent) drawTree();
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
    ConfigTree* tree = &search_tree_[tree_idx];
    std::shared_ptr<Node> closest_node = tree->getClosestNode(connect_goal);

    std::vector<Config> steps;
    float distance;
    World::ExtendResult result = world_ptr_->stepTowards(closest_node->getConfiguration(), connect_goal, steps, distance);
    if(result != World::ExtendResult::CollidedWithoutExtension)
    {
        closest_node = tree->addChildNode(closest_node, steps, distance);
        return closest_node;
    }

    return nullptr;
}

World::ExtendResult RRTConnectPlanner::connectTo(Config connect_goal, int tree_idx)
{
    ConfigTree* tree = &search_tree_[tree_idx];
    std::shared_ptr<Node> closest_node = tree->getClosestNode(connect_goal);
    std::vector<Config> steps;
    int steps_allowed = 100;
    World::ExtendResult result;
    do
    {
        float distance;
        result = world_ptr_->stepTowards(closest_node->getConfiguration(), connect_goal, steps,distance);
        if(result != World::ExtendResult::CollidedWithoutExtension)
        {
            closest_node = tree->addChildNode(closest_node, steps, distance);
        }
    }
    while (result == World::ExtendResult::Extended && steps_allowed > 0);

    if(result == World::ExtendResult::ReachedConnectGoal)
        goal_node_[tree_idx] = closest_node;
    return result;
}

#endif
