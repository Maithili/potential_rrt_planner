#ifndef TYPES_H
#define TYPES_H

#include<iostream>
#include <openrave/plugin.h>

static constexpr int config_dim = 2;
static constexpr int space_dim = 2;

static constexpr float node_distance_tolerance = 0.01;
static constexpr float step_size = 0.1;

using Config = Eigen::Matrix<double,config_dim,1>;
using Location = Eigen::Matrix<double,space_dim,1>;

std::vector<OpenRAVE::GraphHandlePtr> viz_objects;

class Node
{

public:
    Node(){}

    //Use this for the tree root
    Node(const Config& config): 
        parent_{nullptr}, value_{config}, distance_{0.0F}
    {}
    
    Node(const Config& config, Node* parent):
        parent_{parent}, value_{config}, 
        distance_{parent->distance_ + (parent->value_ - this->value_).norm()}
    {}

    void print() const
    {
        std::cout<<"Location : "<<value_.transpose()<<std::endl;
        std::cout<<"Parent : "<<parent_->getConfiguration().transpose()<<std::endl;
        std::cout<<"Number of children : "<<children_.size()<<std::endl;
    }

    void addChild(Node* child)
    {
        children_.push_back(child);
    }

    std::vector<Node*> getChildren()
    {
        return children_;
    }

    Node* getParent()
    {
        return parent_;
    }

    Config getConfiguration() const {return value_;}


private:

    float               distance_;
    Config              value_;
    Node*               parent_;
    std::vector<Node*>  children_;
};

enum class Color
{
    Black = 0,
    Red = 1,
    Blue = 2,
    Green = 3,
};

void drawConfiguration(OpenRAVE::EnvironmentBasePtr env, Location point_eigen, Color color = Color::Black)
{
    float color_arr[] {0.0,0.0,0.0};

    float point3D[3];
    point3D[0] = point_eigen(0);
    point3D[1] = point_eigen(1);
    point3D[2] = space_dim < 3 ? 0.1 : point_eigen(2);

    viz_objects.push_back(env->plot3(point3D, 1, 4, 9 ,color_arr));
}

#endif