#ifndef CONFIG_TREE_H
#define CONFIG_TREE_H

#include <limits>
#include "types.h"

class ConfigTree
{
public:
    ConfigTree(){}

    void setRoot(Node root) 
    {
        tree_.clear();
        tree_.push_back(root);
    }

    Node& getRoot()
    {
        return (tree_.front());
    }

    Node& addChildNode(Node* parent, Config child_config)
    {
        std::cout<<&(tree_.front())<<" : "<<tree_.front().getParent()<<std::endl;
        tree_.push_back(Node(child_config, parent));
        parent->addChild(&tree_.back());
        return tree_.back();
    }

    Node& getClosestNode(std::function<float(Node)> distance_to_target)
    {
        float min_dist = std::numeric_limits<float>::max();
        Node& closest_node =  tree_.front();

        for (auto& node : tree_)
        {
            float d = distance_to_target(node);
            if(d < min_dist)
            {
                closest_node = node;
                min_dist = d;
            }
        }
        return closest_node;
    }

    void draw(OpenRAVE::EnvironmentBasePtr env)
    {
        std::vector<OpenRAVE::GraphHandlePtr> handles;
        for (auto& node : tree_)
        {
            handles.push_back(drawConfiguration(env, node.getConfiguration(), Blue));
            if(node.getParent() != nullptr)
                handles.push_back(drawEdge(env, node.getConfiguration(), node.getParent()->getConfiguration()));
        }
        sleep(1);
        viz_objects = handles;
    }

    void printAll()
    {
        for (auto& node : tree_)
        {
            std::cout<<&node<<" : "<<node.getParent()<<std::endl;
            if(&node == node.getParent())
                std::cout<<"Noooooooooooooooooooooooooooooooooooo"<<std::endl;
        }
    }

private:
    std::vector<Node> tree_;
};

#endif