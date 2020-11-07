#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <openrave/plugin.h>

extern "C" {
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include "libcd/chomp.h"
#include "libcd/grid.h"
#include "libcd/grid_flood.h"
#include "libcd/kin.h"
#include "libcd/mat.h"
#include "libcd/os.h"
#include "libcd/spatial.h"
#include "libcd/util.h"
#include "libcd/util_shparse.h"
}

#define HUGE_VAL 1000.0

static constexpr int config_dim = 3;
static constexpr int space_dim = 2;

using Config = Eigen::Matrix<double,config_dim,1>;
using Location = Eigen::Matrix<double,space_dim,1>;

static constexpr float step_size = 0.2;
static constexpr float inner_step_size = step_size;
static constexpr int num_baby_steps = 5;
static constexpr float outer_step_size = step_size*static_cast<float>(num_baby_steps);
static constexpr float node_distance_tolerance = step_size * 1.5;

namespace potential_params
{
    const float max_dist                = 3.5F;
    const float min_dist                = 0.3F;
    const float potential_power         = 1.5F;
    const float max_potential_gradient  = 10.0F;
    const float potential_gradient_goal = 15.0F;
    const float potential_gradient_rand = 8.0F;
    float goal_potential_gradient       = potential_gradient_goal;
}

struct Sdf
{
   char kinbody_name[256]; /* the grid frame is AABB of this robot */
   double pose_kinbody[7]; /* pose of the grid w.r.t. the kinbody frame */
   /* from world_pt to gsdf_pt */
   int toGridFrame(double* pose, OpenRAVE::EnvironmentBasePtr& env, bool only_direction = false)
   {
      OpenRAVE::Transform t = env->GetKinBody(this->kinbody_name)->GetTransform();
      double pose_world_gsdf[7];
      if(only_direction)
      {
        pose_world_gsdf[0] = 0.0;
        pose_world_gsdf[1] = 0.0;
        pose_world_gsdf[2] = 0.0;
      }
      else
      {
        pose_world_gsdf[0] = t.trans.x;
        pose_world_gsdf[1] = t.trans.y;
        pose_world_gsdf[2] = t.trans.z;
      }
      pose_world_gsdf[3] = t.rot.y;
      pose_world_gsdf[4] = t.rot.z;
      pose_world_gsdf[5] = t.rot.w;
      pose_world_gsdf[6] = t.rot.x;
      cd_kin_pose_compose(pose_world_gsdf, this->pose_kinbody, pose_world_gsdf);
      cd_kin_pose_invert(pose_world_gsdf, pose_gsdf_world);
      cd_kin_pose_compose(pose_gsdf_world, pose, pose); /* world_pt to gsdf_pt */
   }
   int toWorldFrame(double* pose, OpenRAVE::EnvironmentBasePtr& env, bool only_direction = false)
   {
      OpenRAVE::Transform t = env->GetKinBody(this->kinbody_name)->GetTransform();
      double pose_world_gsdf[7];
      if(only_direction)
      {
        pose_world_gsdf[0] = 0.0;
        pose_world_gsdf[1] = 0.0;
        pose_world_gsdf[2] = 0.0;
      }
      else
      {
        pose_world_gsdf[0] = t.trans.x;
        pose_world_gsdf[1] = t.trans.y;
        pose_world_gsdf[2] = t.trans.z;
      }
      pose_world_gsdf[3] = t.rot.y;
      pose_world_gsdf[4] = t.rot.z;
      pose_world_gsdf[5] = t.rot.w;
      pose_world_gsdf[6] = t.rot.x;
      cd_kin_pose_compose(pose_world_gsdf, this->pose_kinbody, pose_world_gsdf);
      cd_kin_pose_compose(pose_world_gsdf, pose, pose); /* gsdf_pt to world_pt */
   }
   int originInWorld(double* pose, OpenRAVE::EnvironmentBasePtr& env)
   {
       OpenRAVE::Transform t = env->GetKinBody(this->kinbody_name)->GetTransform();
       double pose_world_gsdf[7];
       pose_world_gsdf[0] = t.trans.x;
       pose_world_gsdf[1] = t.trans.y;
       pose_world_gsdf[2] = t.trans.z;
       pose_world_gsdf[3] = t.rot.y;
       pose_world_gsdf[4] = t.rot.z;
       pose_world_gsdf[5] = t.rot.w;
       pose_world_gsdf[6] = t.rot.x;
       cd_kin_pose_compose(pose, this->pose_kinbody, pose);
   }
   double pose_gsdf_world[7];
   cd_grid * grid;
};

struct Sphere
{
    /* parsed from xml */
    int linkindex;
    char linkname[32];
    double pos_linkframe[3];
    double pos_worldframe[3];
    double radius;
    void getJacobian(boost::multi_array<double,2>& jacobian_out, OpenRAVE::RobotBasePtr& robot)
    {
        OpenRAVE::geometry::RaveVector<double> pose(pos_linkframe[0], pos_linkframe[1], pos_linkframe[2]);
        robot->CalculateJacobian(this->linkindex, pose, jacobian_out);
    }
};

struct Spheres
{
    std::vector<Sphere> list;
    char robotname[32];

    void setBarretWAM(OpenRAVE::EnvironmentBasePtr& env)
    {
        strcpy(robotname, "BarrettWAM");
        Sphere s;
        strcpy( s.linkname, "wam0"     ) ; s.pos_linkframe[0] = 0.22; s.pos_linkframe[1] = 0.14 ; s.pos_linkframe[2] = 0.346; s.radius=0.15; this->list.push_back(s);
        strcpy( s.linkname, "wam2"     ) ; s.pos_linkframe[0] = 0.0 ; s.pos_linkframe[1] = 0.0  ; s.pos_linkframe[2] = 0.2  ; s.radius=0.06; this->list.push_back(s);
        strcpy( s.linkname, "wam2"     ) ; s.pos_linkframe[0] = 0.0 ; s.pos_linkframe[1] = 0.0  ; s.pos_linkframe[2] = 0.3  ; s.radius=0.06; this->list.push_back(s);
        strcpy( s.linkname, "wam2"     ) ; s.pos_linkframe[0] = 0.0 ; s.pos_linkframe[1] = 0.0  ; s.pos_linkframe[2] = 0.4  ; s.radius=0.06; this->list.push_back(s);
        strcpy( s.linkname, "wam2"     ) ; s.pos_linkframe[0] = 0.0 ; s.pos_linkframe[1] = 0.0  ; s.pos_linkframe[2] = 0.5  ; s.radius=0.06; this->list.push_back(s);
        strcpy( s.linkname, "wam3"     ) ; s.pos_linkframe[0] = 0.0 ; s.pos_linkframe[1] = 0.0  ; s.pos_linkframe[2] = 0.0  ; s.radius=0.06; this->list.push_back(s);
        strcpy( s.linkname, "wam4"     ) ; s.pos_linkframe[0] = 0.0 ; s.pos_linkframe[1] = 0.0  ; s.pos_linkframe[2] = 0.2  ; s.radius=0.06; this->list.push_back(s);
        strcpy( s.linkname, "wam4"     ) ; s.pos_linkframe[0] = 0.0 ; s.pos_linkframe[1] = 0.0  ; s.pos_linkframe[2] = 0.1  ; s.radius=0.06; this->list.push_back(s);
        strcpy( s.linkname, "wam4"     ) ; s.pos_linkframe[0] = 0.0 ; s.pos_linkframe[1] = 0.0  ; s.pos_linkframe[2] = 0.3  ; s.radius=0.06; this->list.push_back(s);
        strcpy( s.linkname, "wam6"     ) ; s.pos_linkframe[0] = 0.0 ; s.pos_linkframe[1] = 0.0  ; s.pos_linkframe[2] = 0.1  ; s.radius=0.06; this->list.push_back(s);
        strcpy( s.linkname, "Finger0-1") ; s.pos_linkframe[0] = 0.05; s.pos_linkframe[1] = -0.01; s.pos_linkframe[2] = 0.0  ; s.radius=0.04; this->list.push_back(s);
        strcpy( s.linkname, "Finger1-1") ; s.pos_linkframe[0] = 0.05; s.pos_linkframe[1] = -0.01; s.pos_linkframe[2] = 0.0  ; s.radius=0.04; this->list.push_back(s);
        strcpy( s.linkname, "Finger2-1") ; s.pos_linkframe[0] = 0.05; s.pos_linkframe[1] = -0.01; s.pos_linkframe[2] = 0.0  ; s.radius=0.04; this->list.push_back(s);
        strcpy( s.linkname, "Finger0-2") ; s.pos_linkframe[0] = 0.05; s.pos_linkframe[1] =  0.0 ; s.pos_linkframe[2] = 0.0  ; s.radius=0.04; this->list.push_back(s);
        strcpy( s.linkname, "Finger1-2") ; s.pos_linkframe[0] = 0.05; s.pos_linkframe[1] =  0.0 ; s.pos_linkframe[2] = 0.0  ; s.radius=0.04; this->list.push_back(s);
        strcpy( s.linkname, "Finger2-2") ; s.pos_linkframe[0] = 0.05; s.pos_linkframe[1] =  0.0 ; s.pos_linkframe[2] = 0.0  ; s.radius=0.04; this->list.push_back(s);
        //  TODO(maithili)  : set linkindex
        for (Sphere temp : this->list)
        {
            temp.linkindex = env->GetRobot(robotname)->GetLink(temp.linkname)->GetIndex();
        }
    }
};

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
Color Green{0.2, 0.8, 0.2};
Color Blue {0.2, 0.1, 0.7};
Color Pale {0.6, 0.5, 0.6};

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

    void setIntermediateSteps(std::vector<Config> in) {intermediate_steps_ = in;}
    std::vector<Config> getIntermediateSteps() {return intermediate_steps_;}

private:

    float               distance_;
    Config              value_;
    std::vector<Config> intermediate_steps_;
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

template<int S>
void copyToVector(const Eigen::Matrix<double, S, 1>& eigen, std::vector<double>& vector)
{
    vector.clear();
    vector.reserve(S);
    for (int i=0;i<S;++i)
        vector.push_back(eigen[i]);
}

template<int S>
void copyToEigen(const std::vector<double>& vector, Eigen::Matrix<double, S, 1>& eigen)
{
    for (int i=0;i<S;++i)
        eigen[i]=vector[i];
}

#endif