#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <openrave/plugin.h>
#include <openrave/openrave.h>

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

#define PLANAR
// #define ARM

// #define SMALLWORLD
#define LARGEWORLD

#define HUGE_VAL 1000.0

static constexpr bool silent = true;


#ifdef PLANAR
#ifdef SMALLWORLD

static constexpr float x_min = -5;
static constexpr float y_min = -5;
static constexpr float x_max = 5;
static constexpr float y_max = 5;

static constexpr float step_size = 0.2;
static constexpr float inner_step_size = step_size;
static constexpr int num_baby_steps = 5;
static constexpr float outer_step_size = step_size*static_cast<float>(num_baby_steps);
static constexpr float node_distance_tolerance = step_size * 1.5;
namespace potential_params
{
    const float max_dist                = 3.5F;
    const float min_dist                = 0.2F;
    const float potential_power         = 1.5F;
    const float max_potential_gradient  = 10.0F;
    const float potential_gradient_goal = 15.0F;
    const float potential_gradient_rand = 8.0F;
    float goal_potential_gradient       = potential_gradient_goal;
}
static constexpr int config_dim = 3;
static constexpr int space_dim = 2;
#endif

#ifdef LARGEWORLD
static constexpr float x_min = -50;
static constexpr float y_min = -50;
static constexpr float x_max = 50;
static constexpr float y_max = 50;

static constexpr float step_size = 1.0;
static constexpr float inner_step_size = step_size;
static constexpr int num_baby_steps = 5;
static constexpr float outer_step_size = step_size*static_cast<float>(num_baby_steps);
static constexpr float node_distance_tolerance = step_size * 1.5;
namespace potential_params
{
    const float max_dist                = 25.F;
    const float min_dist                = 2.F;
    const float potential_power         = 2.F;
    const float max_potential_gradient  = 50.F;
    const float potential_gradient_goal = 50.F;
    const float potential_gradient_rand = 50.F;
    float goal_potential_gradient       = potential_gradient_goal;
}
static constexpr int config_dim = 3;
static constexpr int space_dim = 2;
#endif
#endif

#ifdef ARM
static constexpr float step_size = 0.5;
static constexpr float inner_step_size = step_size;
static constexpr int num_baby_steps = 5;
static constexpr float outer_step_size = step_size*static_cast<float>(num_baby_steps);
static constexpr float node_distance_tolerance = step_size * 5;
namespace potential_params
{
    const float max_dist                = 1.5F;
    const float min_dist                = 0.0F;
    const float potential_power         = 1.5F;
    const float max_potential_gradient  = 10.0F;
    const float potential_gradient_goal = 15.0F;
    const float potential_gradient_rand = 8.0F;
    float goal_potential_gradient       = potential_gradient_goal;
}
static constexpr int config_dim = 7;
static constexpr int space_dim = 3;
#endif

using Config = Eigen::Matrix<double,config_dim,1>;
using Location = Eigen::Matrix<double,space_dim,1>;

float calculateGoalPotentialGradient()
{
    return potential_params::goal_potential_gradient;
}

float calculatePotentialGradient(float dist)
{
    if(dist > potential_params::max_dist)
        return 0.0F;
    else
    {   
        float factor = potential_params::max_potential_gradient
                     * pow(potential_params::min_dist,potential_params::potential_power);
        if (dist < potential_params::min_dist)
            return potential_params::max_potential_gradient;
        else
        {
            return (factor/pow(dist,potential_params::potential_power));
        }
    }
}

struct Sdf
{
   char kinbody_name[256]; /* the grid frame is AABB of this robot */
   double pose_kinbody[7]; /* pose of the grid w.r.t. the kinbody frame */
   cd_grid * grid;
   /* from world_pt to gsdf_pt */
   void poseToGridFrame(double* pose, OpenRAVE::EnvironmentBasePtr& env) const
   {
      OpenRAVE::Transform t = env->GetKinBody(this->kinbody_name)->GetTransform();
      double pose_world_gsdf[7];
      double pose_gsdf_world[7];
      pose_world_gsdf[0] = t.trans.x;
      pose_world_gsdf[1] = t.trans.y;
      pose_world_gsdf[2] = t.trans.z;
      pose_world_gsdf[3] = t.rot.y;
      pose_world_gsdf[4] = t.rot.z;
      pose_world_gsdf[5] = t.rot.w;
      pose_world_gsdf[6] = t.rot.x;
      cd_kin_pose_compose(pose_world_gsdf, this->pose_kinbody, pose_world_gsdf);
      cd_kin_pose_invert(pose_world_gsdf, pose_gsdf_world);
      cd_kin_pose_compose(pose_gsdf_world, pose, pose); /* world_pt to gsdf_pt */
   }
   void pointToGridFrame(double* point, OpenRAVE::EnvironmentBasePtr& env) const
   {
      double pose[7];
      pose[0] = point[0];
      pose[1] = point[1];
      pose[2] = point[2];
      pose[3] = 0.0;
      pose[4] = 0.0;
      pose[5] = 0.0;
      pose[6] = 0.0;
      poseToGridFrame(pose, env);
      point[0] = pose[0];
      point[1] = pose[1];
      point[2] = pose[2];
   }
   void poseToWorldFrame(double* pose, OpenRAVE::EnvironmentBasePtr& env) const
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
      cd_kin_pose_compose(pose_world_gsdf, this->pose_kinbody, pose_world_gsdf);
      cd_kin_pose_compose(pose_world_gsdf, pose, pose); /* gsdf_pt to world_pt */
   }
   void pointToWorldFrame(double* point, OpenRAVE::EnvironmentBasePtr& env) const
   {
      double pose[7];
      pose[0] = point[0];
      pose[1] = point[1];
      pose[2] = point[2];
      pose[3] = 0.0;
      pose[4] = 0.0;
      pose[5] = 0.0;
      pose[6] = 0.0;
      poseToWorldFrame(pose, env);
      point[0] = pose[0];
      point[1] = pose[1];
      point[2] = pose[2];
   }
   void originInWorld(double* pose, OpenRAVE::EnvironmentBasePtr& env) const
   {
       OpenRAVE::Transform t = env->GetKinBody(this->kinbody_name)->GetTransform();
       pose[0] = t.trans.x;
       pose[1] = t.trans.y;
       pose[2] = t.trans.z;
       pose[3] = t.rot.y;
       pose[4] = t.rot.z;
       pose[5] = t.rot.w;
       pose[6] = t.rot.x;
   }
};

struct Sphere
{
    /* parsed from xml */
    int linkindex;
    char linkname[32];
    double pos_linkframe[3];
    double pos_worldframe[3];
    double radius;
    void getJacobian(boost::multi_array<double,2>& jacobian_out, OpenRAVE::RobotBasePtr& robot) const
    {
        OpenRAVE::geometry::RaveVector<double> pose(pos_linkframe[0], pos_linkframe[1], pos_linkframe[2]);
        robot->CalculateActiveJacobian(this->linkindex, pose, jacobian_out);
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
        for (Sphere& temp : this->list)
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

std::vector<OpenRAVE::GraphHandlePtr> viz_tree;
std::vector<OpenRAVE::GraphHandlePtr> viz_objects_permanent;

class Node
{

public:
    // Node(){}

    //Use this for the tree root
    Node(const Config config): 
        parent_{nullptr}, value_{config}, distance_{0.0F}
    {}
    
    Node(std::shared_ptr<Node> parent, const Config config, float distance=-1):
        parent_{parent}, value_{config}
    {
        distance_ = distance > 0 ? parent->getDistanceFromRoot() + distance
                                 : parent->getDistanceFromRoot() + (parent->getConfiguration() - this->value_).norm();
    }

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

    bool removeChild(std::shared_ptr<Node> child_to_remove)
    {
        std::vector<std::shared_ptr<Node> >::iterator remove_it = 
                     std::find_if(children_.begin(), children_.end(),
                     [&child_to_remove](const std::shared_ptr<Node>& x) 
                        { return x->getConfiguration().isApprox(child_to_remove->getConfiguration());});
        
        if (remove_it == children_.end())
            return false;

        children_.erase(remove_it);
        return true;
    }

    std::shared_ptr<Node> getParent() const
    {
        return parent_;
    }

    void setParent(std::shared_ptr<Node> in)
    {
        parent_ = in;
    }

    void setDistance(float in) {distance_=in;}

    float getDistanceFromRoot() const {return distance_;}

    Config getConfiguration() const {return value_;}

    void setIntermediateSteps(std::vector<Config> in) {intermediate_steps_ = in;}
    std::vector<Config> getIntermediateSteps() const {return intermediate_steps_;}

private:

    float               distance_;
    Config              value_;
    std::vector<Config> intermediate_steps_;
    std::shared_ptr<Node>                parent_;
    std::vector<std::shared_ptr<Node> >  children_;
};

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
    if (vector.size() != S)
        RAVELOG_WARN("Trying to copy vector to eigen with inappropriate dimensions");
    for (int i=0;i<S;++i)
        eigen[i]=vector[i];
}

Location getEndeffector(OpenRAVE::EnvironmentBasePtr env, Config angles)
{
    Location loc = Location::Zero();
    std::vector<OpenRAVE::RobotBasePtr> robots;
    env->GetRobots(robots);
    OpenRAVE::RobotBasePtr robot = robots.front();
    OpenRAVE::RobotBase::RobotStateSaver save_state(robot);
    std::vector<double> config;
    copyToVector(angles, config);
    robot->SetActiveDOFValues(config);
    OpenRAVE::Transform t = robots.front()->GetActiveManipulator()->GetEndEffectorTransform();
    loc << t.trans.x, t.trans.y, t.trans.z;
    return loc;
}

OpenRAVE::GraphHandlePtr drawConfiguration(OpenRAVE::EnvironmentBasePtr env, Config point_eigen, Color color = Color(), float size = 5)
{
    Location point;
    point = (config_dim > 3) ? getEndeffector(env,point_eigen) : point_eigen.topRows(space_dim);
    float point3D[3];
    point3D[0] = point(0);
    point3D[1] = point(1);
    point3D[2] = space_dim < 3 ? 0.1 : point(2);

    return (env->plot3(point3D, 1, sizeof(point3D[0])*3, size ,color()));
}

OpenRAVE::GraphHandlePtr drawEdge(OpenRAVE::EnvironmentBasePtr env, Config point_eigen1, Config point_eigen2, Color color = Color(), float size = 0.5)
{
    Location point1;
    point1 = (config_dim > 3) ? getEndeffector(env,point_eigen1) : point_eigen1.topRows(space_dim);
    Location point2;
    point2 = (config_dim > 3) ? getEndeffector(env,point_eigen2) : point_eigen2.topRows(space_dim);
    
    float point3D[6];
    point3D[0] = point1(0);
    point3D[1] = point1(1);
    point3D[2] = space_dim < 3 ? 0.1 : point1(2);

    point3D[3] = point2(0);
    point3D[4] = point2(1);
    point3D[5] = space_dim < 3 ? 0.1 : point2(2);

    return (env->drawlinestrip(&point3D[0], 2, sizeof(point3D[0])*3, size, color()));
}

void showRobot(OpenRAVE::EnvironmentBasePtr env)
{
    if(silent) return;
    std::vector<OpenRAVE::RobotBasePtr> robots;
    env->GetRobots(robots);
    OpenRAVE::RobotBasePtr robot = robots.front();
    OpenRAVE::RobotBase::RobotStateSaver save_state(robot);
    std::vector<double> config_vector;
    robot->GetDOFValues(config_vector);
    robot->GetController()->SetDesired(config_vector);
    env->UpdatePublishedBodies();
    sleep(0.1);
}

void showRobotAt(Config cfg, OpenRAVE::EnvironmentBasePtr env)
{
    if(silent) return;
    std::vector<OpenRAVE::RobotBasePtr> robots;
    env->GetRobots(robots);
    OpenRAVE::RobotBasePtr robot = robots.front();
    OpenRAVE::RobotBase::RobotStateSaver save_state(robot);
    std::vector<double> config_vector;
    copyToVector(cfg, config_vector);
    robot->SetActiveDOFValues(config_vector);
    robot->GetDOFValues(config_vector);
    robot->GetController()->SetDesired(config_vector);
    env->UpdatePublishedBodies();
    sleep(0.1);
}

#endif