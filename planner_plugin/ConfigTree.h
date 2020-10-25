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
        // tree_.clear();
        // tree_.push_back(root);
    }

    // Node& getRoot()
    // {
    //     return *root_;
    // }

    std::shared_ptr<Node> addChildNode(std::shared_ptr<Node> parent, Config child_config)
    {
        std::shared_ptr<Node> child = std::make_shared<Node>(parent, child_config);
        parent->addChild(child);
        return child;
    }

    std::shared_ptr<Node> getClosestNode(std::function<float(Node)> distance_to_target)
    {
        float min_dist = std::numeric_limits<float>::max();
        std::shared_ptr<Node> closest_node;
        
        std::vector<std::shared_ptr<Node> > nodes_to_check;
        std::shared_ptr<Node> nodeptr =  root_;
        nodes_to_check.push_back(nodeptr);

        while(!nodes_to_check.empty())
        {
            nodeptr = nodes_to_check.back();
            nodes_to_check.pop_back();
            float d = distance_to_target(*nodeptr);
            if(d < min_dist)
            {
                closest_node = nodeptr;
                min_dist = d;
            }
            auto children = nodeptr->getChildren();
            nodes_to_check.insert(nodes_to_check.end(), children.begin(), children.end());
        }
        return closest_node;
    }

    void draw(OpenRAVE::EnvironmentBasePtr env)
    {
        std::vector<std::shared_ptr<Node> > nodes_to_check;
        std::shared_ptr<Node> nodeptr =  root_;
        nodes_to_check.push_back(nodeptr);
        std::vector<OpenRAVE::GraphHandlePtr> handles;
        while(!nodes_to_check.empty())
        {
            nodeptr = nodes_to_check.back();
            nodes_to_check.pop_back();
            
            handles.push_back(drawConfiguration(env, nodeptr->getConfiguration(), Blue));
            if(nodeptr->getParent() != nullptr)
                handles.push_back(drawEdge(env, nodeptr->getConfiguration(), nodeptr->getParent()->getConfiguration()));

            auto children = nodeptr->getChildren();
            nodes_to_check.insert(nodes_to_check.end(), children.begin(), children.end());
        }
        sleep(0.1);
        viz_objects = handles;
    }

    // void printAll()
    // {
    //     for (auto& node : tree_)
    //     {
    //         std::cout<<&node<<" : "<<node.getParent()<<std::endl;
    //         if(&node == node.getParent())
    //             std::cout<<"Noooooooooooooooooooooooooooooooooooo"<<std::endl;
    //     }
    // }

private:
    std::shared_ptr<Node> root_;
    // std::vector<Node> tree_;
};

#endif