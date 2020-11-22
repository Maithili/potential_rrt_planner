#ifndef CONFIG_TREE_H
#define CONFIG_TREE_H

#include <limits>
#include "types.h"

class ConfigTree
{
public:
    ConfigTree(){}

    void setRoot(Config root_config) 
    {
        root_ = std::make_shared<Node>(root_config);
    }

    std::shared_ptr<Node> getRoot() {return root_;}

    std::shared_ptr<Node> addChildNode(std::shared_ptr<Node> parent, Config child_config)
    {
        std::shared_ptr<Node> child = std::make_shared<Node>(parent, child_config);
        parent->addChild(child);
        return child;
    }

    std::shared_ptr<Node> getClosestNode(Config target)
    {
        float min_dist = std::numeric_limits<float>::max();
        std::shared_ptr<Node> closest_node = nullptr;
        std::vector<std::shared_ptr<Node> > nodes_to_check;
        std::shared_ptr<Node> nodeptr =  root_;
        nodes_to_check.push_back(nodeptr);

        while(!nodes_to_check.empty())
        {
            nodeptr = nodes_to_check.back();
            nodes_to_check.pop_back();
            float d = (nodeptr->getConfiguration() - target).norm();
            if(d < min_dist)
            {
                closest_node = nodeptr;
                min_dist = d;
            }
            auto children = nodeptr->getChildren();
            nodes_to_check.insert(nodes_to_check.end(), children.begin(), children.end());
        }
        if(closest_node == nullptr)
            std::cout<<"Closest node failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        return closest_node;
    }

    std::vector<std::shared_ptr<Node> > getNodesWithinDistance(std::function<float(Node)> distance_to_target, float threshold)
    {
        std::vector<std::shared_ptr<Node> > close_nodes;
        float min_dist = std::numeric_limits<float>::max();
        std::vector<std::shared_ptr<Node> > nodes_to_check;
        std::shared_ptr<Node> nodeptr =  root_;
        nodes_to_check.push_back(nodeptr);

        while(!nodes_to_check.empty())
        {
            nodeptr = nodes_to_check.back();
            nodes_to_check.pop_back();
            float d = distance_to_target(*nodeptr);
            if(d < threshold)
            {
                close_nodes.push_back(nodeptr);
            }
            auto children = nodeptr->getChildren();
            nodes_to_check.insert(nodes_to_check.end(), children.begin(), children.end());
        }
        return close_nodes;
    }

    std::vector<std::vector<double> > getPathToRoot(std::shared_ptr<Node> leaf, OpenRAVE::EnvironmentBasePtr env) const
    {
        std::vector<std::vector<double> > path;
        std::vector<Config> config_path;
        while (leaf != root_)
        {
            std::vector<Config> steps = leaf->getIntermediateSteps();
            config_path.insert(config_path.end(),steps.rbegin(), steps.rend());
            if(leaf->getParent() == nullptr)
            {
                std::cout<<"Found nullpointer before reaching root"<<std::endl;
                break;
            }
            leaf = leaf->getParent();
        }
        Config prev = config_path.front();
        for(auto& cfg : config_path)
        {
            std::vector<double> config_stl;
            copyToVector(cfg, config_stl);
            path.push_back(config_stl);
            if(!silent)
            {
                viz_objects_permanent.push_back(drawConfiguration(env, cfg, Green, 3));
                viz_objects_permanent.push_back(drawEdge(env, cfg, prev, Green));
                prev = cfg;
            }
        }
        return path;
    }

private:
    std::shared_ptr<Node> root_;
};

#endif