#ifndef PLANNER_H
#define PLANNER_H

#include <random>

#include "World.h"
#include "types.h"
#include "ConfigTree.h"

class Planner
{
public:
    Planner(): goal_bias_{0.5}, step_length_{1.0}
    {}

    virtual void plan(int max_iterations) = 0;

    virtual std::vector<std::vector<double> > getPath() const = 0;

protected:
    float    goal_bias_;
    float    step_length_;
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

    RRTPlanner(World* world_ptr_in): 
        world_ptr_{world_ptr_in}, reached_goal_{false}, search_tree_{Node()},
        latest_node_{}
    {
        latest_node_ = std::make_shared<Node>(world_ptr_->getStart());
        search_tree_.setRoot(latest_node_);
    }
    
    void plan(int max_iterations) override;
    
    std::vector<std::vector<double> > getPath() const override;

    void setParams(float goal_bias = 0.5, float step_length = 1)
    {
        goal_bias_ = goal_bias; step_length_ = step_length;
    }

private:
    ConnectResult connectTo(Config connect_goal);
    bool reachToGoal();

    World*                      world_ptr_;
    ConfigTree<Node>            search_tree_;
    std::shared_ptr<Node>       latest_node_;
    bool                        reached_goal_;
};

void RRTPlanner::plan(int max_iterations)
{
    for (int iteration=0; iteration<max_iterations; ++iteration)
    {
        std::cout<<"Starting iteration number : "<<iteration<<std::endl;
        Config connect_goal = reachToGoal()? world_ptr_->getGoal() 
                                : world_ptr_->getRandomConfig();
        std::cout<<"Connecting to : "<<connect_goal.transpose()<<std::endl;
        ConnectResult result = connectTo(connect_goal);
        if (result == ConnectResult::ReachedGoal)
        {
            reached_goal_ = true;
            break;
        }
    }
}

RRTPlanner::ConnectResult RRTPlanner::connectTo(Config connect_goal)
{
    std::cout<<"Calling ConnectTo()..."<<std::endl;
    std::function<float(Node)> distance_to_goal = [&connect_goal](Node node) 
        { return (node.getConfiguration() - connect_goal).norm(); };
    latest_node_ = search_tree_.getClosestNode(distance_to_goal);
    std::cout<<"Found closest node : "<<latest_node_->getConfiguration().transpose()<<std::endl;
    
    ConnectResult result(ConnectResult::Running);
    while (result == ConnectResult::Running)
    {
        Config new_config = world_ptr_->stepTowards(latest_node_->getConfiguration(), connect_goal);

        if(world_ptr_->isGoal(new_config))
        {
            result = ConnectResult::ReachedGoal;
        }
        else if((new_config - connect_goal).norm() < node_distance_tolerance)
        {
            result = ConnectResult::ReachedConnectGoal;
        }
        else if((new_config - latest_node_->getConfiguration()).norm() < node_distance_tolerance)
        {
            result = ConnectResult::Stuck;
        }
        latest_node_ = latest_node_->addChild(new_config);
    }
    
    std::cout<<"Loook at meeee!!!!!!!!!!! "<<std::endl;
    return result;
}

std::vector<std::vector<double> > RRTPlanner::getPath() const
{
    if(!reached_goal_)
    {
        std::vector<std::vector<double> > empty_path;
        return empty_path;
    }

    std::vector<std::vector<double> > path;
    std::shared_ptr<Node> current_node = latest_node_;
    while (current_node->getParent())
    {
        Config config = current_node->getConfiguration();
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
