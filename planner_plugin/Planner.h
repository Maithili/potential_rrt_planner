#ifndef PLANNER_H
#define PLANNER_H

#include <random>

#include "World.h"
#include "types.h"
#include "ConfigTree.h"

class Planner
{
public:
    Planner(): goal_bias_{0.999}
    {}

    virtual void plan(int max_iterations) = 0;

    virtual std::vector<std::vector<double> > getPath() const = 0;

    float goal_bias_;
};

class RRTPlanner : public Planner
{
public:

    enum class ConnectResult
    {
        Running            = 0,
        ReachedGoal        = 1,
        ReachedConnectGoal = 2,
        Stuck              = 3,
        Collided           = 4
    };

    RRTPlanner(): 
        search_tree_{}, goal_node_{nullptr}
    {}

    void setWorld(World* world_ptr_in)
    {
        world_ptr_ = world_ptr_in;
    }
    
    void plan(int max_iterations) override;
    
    std::vector<std::vector<double> > getPath() const override;

private:
    ConnectResult connectTo(Config connect_goal);
    bool reachToGoal();

    World* world_ptr_;
    ConfigTree search_tree_;
    std::shared_ptr<Node> goal_node_;
};

void RRTPlanner::plan(int max_iterations)
{
    search_tree_.setRoot(world_ptr_->getStart());
    for (int iteration=0; iteration<max_iterations; ++iteration)
    {
        // search_tree_.draw(world_ptr_->env_);
        potential_params::goal_potential_gradient = potential_params::potential_gradient_goal;
        Config connect_goal = reachToGoal()? world_ptr_->getGoal() 
                                : world_ptr_->getRandomConfig();
        ConnectResult result = connectTo(connect_goal);
        if (result == ConnectResult::ReachedGoal)
        {
            std::cout<<"Reached node at : "<<goal_node_->getConfiguration().transpose()<<std::endl;
            std::cout<<"Iterations : "<<iteration<<std::endl;
            return;
        }
    }
    std::cout<<"Failed search after reaching max iterations of "<<max_iterations<<std::endl;
}

RRTPlanner::ConnectResult RRTPlanner::connectTo(Config connect_goal)
{
    // viz_objects_permanent.push_back(drawConfiguration(world_ptr_->env_, connect_goal, Pale, 8));
    std::function<float(Node)> distance_to_goal = [&connect_goal](Node node) 
        { return (node.getConfiguration() - connect_goal).norm(); };
    std::shared_ptr<Node> closest_node = search_tree_.getClosestNode(distance_to_goal);

    int steps_allowed = 100;
    ConnectResult result(ConnectResult::Running);
    while (result == ConnectResult::Running && steps_allowed > 0)
    {
        Config new_config = world_ptr_->stepTowards(closest_node->getConfiguration(), connect_goal);
        if(world_ptr_->isInCollision(new_config))
        {
            viz_objects_permanent.push_back(drawConfiguration(world_ptr_->env_, new_config.topRows(space_dim), Red, 3));
            result = ConnectResult::Collided;
            return result;
        }

        if((new_config - closest_node->getConfiguration()).norm() < node_distance_tolerance)
        {
            result = ConnectResult::Stuck;
        }

        closest_node = search_tree_.addChildNode(closest_node, new_config);

        viz_objects_permanent.push_back(drawConfiguration(world_ptr_->env_, closest_node->getConfiguration().topRows(space_dim), Blue));
        viz_objects_permanent.push_back(drawEdge(world_ptr_->env_, closest_node->getConfiguration().topRows(space_dim), closest_node->getParent()->getConfiguration().topRows(space_dim)));

        if(world_ptr_->isGoal(new_config))
        {
            result = ConnectResult::ReachedGoal;
            goal_node_ = closest_node;
        }
        else if((new_config - connect_goal).norm() < node_distance_tolerance)
        {
            result = ConnectResult::ReachedConnectGoal;
        }
        --steps_allowed;
    }
    
    return result;
}

std::vector<std::vector<double> > RRTPlanner::getPath() const
{
    std::vector<std::vector<double> > path;
    std::shared_ptr<Node> current_node = goal_node_;
    while (current_node != nullptr)
    {
        Config config = current_node->getConfiguration();
        viz_objects.push_back(drawConfiguration(world_ptr_->env_, config.topRows(space_dim), Green, 3));
        std::vector<double> config_stl(config.data(), config.data()+config.rows());
        path.push_back(config_stl);
        current_node = current_node->getParent();
    }

    return path;
}

bool RRTPlanner::reachToGoal()
{
    float random_fraction = ( static_cast<float>(rand()) / static_cast<float>(RAND_MAX) );
    return(random_fraction < goal_bias_);
}


#endif
