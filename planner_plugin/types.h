#ifndef TYPES_H
#define TYPES_H

#include<iostream>
#include <openrave/plugin.h>

static constexpr int config_dim = 2;
static constexpr int space_dim = 2;
static constexpr float node_distance_tolerance = 0.01;

using Config = Eigen::Matrix<double,config_dim,1>;
using Location = Eigen::Matrix<double,space_dim,1>;

std::vector<OpenRAVE::GraphHandlePtr> visualizer_objects;

class Node
{

public:
    Node(){}

    //Use this for the tree root
    Node(const Config& config): 
        parent_{nullptr}, value_{config}, distance_{0.0F}
    {}

    std::shared_ptr<Node> addChild(const Config& config)
    {
        std::shared_ptr<Node> child (new Node(config, this));
        children_.push_back(child);
        return child;
    }

    std::vector<std::shared_ptr<Node> > getChildren()
    {
        return children_;
    }

    std::shared_ptr<Node> getParent()
    {
        return parent_;
    }

    Config getConfiguration() const {return value_;}

private:
    // this is used to create child node
    Node(const Config& config, Node* parent):
        parent_{parent}, value_{config}, 
        distance_{parent->distance_ + (parent->value_ - this->value_).norm()}
    {}

    float                               distance_;
    Config                              value_;
    std::shared_ptr<Node>               parent_;
    std::vector<std::shared_ptr<Node> > children_;
};

#endif