#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <openrave/plugin.h>

static constexpr int config_dim = 2;
static constexpr int space_dim = 2;

static constexpr float step_size = 0.2;
static constexpr float node_distance_tolerance = step_size * 1.5;

typedef struct Color
{   
    float r = 0.2;
    float g = 0.2;
    float b = 0.2;
    float code[3];
    float* operator()()
    {
        code[0]=r; code[1]=g; code[2]=b;
        return code;
    }
};
Color Black{0.0, 0.0, 0.0};
Color Red  {0.8, 0.1, 0.1};
Color Green{0.5, 0.8, 0.2};
Color Blue {0.0, 0.0, 0.7};
Color Pale {0.5, 0.5, 0.6};

using Config = Eigen::Matrix<double,config_dim,1>;
using Location = Eigen::Matrix<double,space_dim,1>;

std::vector<OpenRAVE::GraphHandlePtr> viz_objects;
std::vector<OpenRAVE::GraphHandlePtr> viz_objects_permanent;

class Node
{

public:
    // Node(){}

    //Use this for the tree root
    Node(const Config& config): 
        parent_{nullptr}, value_{config}, distance_{0.0F}
    {}
    
    Node(std::shared_ptr<Node> parent, const Config& config):
        parent_{parent}, value_{config}, 
        distance_{parent->distance_ + (parent->value_ - this->value_).norm()}
    {}

    void print() const
    {
        std::cout<<"Location : "<<value_.transpose()<<std::endl;
        std::cout<<"Parent : "<<parent_->getConfiguration().transpose()<<std::endl;
        std::cout<<"Number of children : "<<children_.size()<<std::endl;
    }

    void addChild(std::shared_ptr<Node>& child)
    {
        children_.push_back(child);
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

    float               distance_;
    Config              value_;
    std::shared_ptr<Node>               parent_;
    std::vector<std::shared_ptr<Node> >  children_;
};

OpenRAVE::GraphHandlePtr drawConfiguration(OpenRAVE::EnvironmentBasePtr env, Location point_eigen, Color color = Color(), float size = 5)
{
    float point3D[3];
    point3D[0] = point_eigen(0);
    point3D[1] = point_eigen(1);
    point3D[2] = space_dim < 3 ? 0.1 : point_eigen(2);

    return (env->plot3(point3D, 1, sizeof(point3D[0])*3, size ,color()));
}

OpenRAVE::GraphHandlePtr drawEdge(OpenRAVE::EnvironmentBasePtr env, Location point1, Location point2, Color color = Color())
{
    float point3D[5];
    point3D[0] = point1(0);
    point3D[1] = point1(1);
    point3D[2] = space_dim < 3 ? 0.1 : point1(2);

    point3D[3] = point2(0);
    point3D[4] = point2(1);
    point3D[5] = space_dim < 3 ? 0.1 : point2(2);

    return (env->drawlinestrip(&point3D[0], 2, sizeof(point3D[0])*3, 0.5, color()));
}
#endif