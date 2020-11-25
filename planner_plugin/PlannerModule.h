#ifndef PLANNER_MODULE_H
#define PLANNER_MODULE_H

#include <boost/bind.hpp>
#include <openrave/planningutils.h>
#include "RRTPlanner.h"
#include "RRTConnectPlanner.h"
#include "RRTStarPlanner.h"

using namespace std;
using namespace OpenRAVE;

class TestPlannerModule : public ModuleBase
{
public:
    TestPlannerModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv)
    {
        RegisterCommand("PlannerCommand",boost::bind(&TestPlannerModule::runCommand,this,_1,_2),
                        "this is an example");
                        // "algo 1/2; goal x,y,th ; goalbias 1; done");
    }

    virtual ~TestPlannerModule() {}

    bool runCommand(std::ostream& sout, std::istream& sinput)
    {
        sout<<"Received command (TestPlannerModule)!!!";
        return true;
    }

};

class PlannerModule : public ModuleBase
{
public:
    PlannerModule(EnvironmentBasePtr penv, std::istream& ss) : 
        ModuleBase(penv), env_(penv), world_(env_),
        world_potential_(env_), world_high_dof_(env_)
    {
        RegisterCommand("PlannerCommand",boost::bind(&PlannerModule::runCommand,this,_1,_2),
                        "algo 1/2; goal x,y,th ; goalbias 1; done");
        RegisterCommand("TestCommand",boost::bind(&PlannerModule::runTest,this,_1,_2),"blah");
    }

    virtual ~PlannerModule() {}

    bool runCommand(std::ostream& sout, std::istream& sinput);
    
    bool runTest(std::ostream& sout, std::istream& sinput)
    { 
        std::cout<< "Test run for Planner Module"<<std::endl;
        // std::cout<< "Input : "<<sinput<<std::endl;
        return true;
    }

    void parseInput(std::istream& sinput);

private:
    EnvironmentBasePtr env_;

    EuclideanWorld world_;
    EuclideanWorldWithPotential world_potential_;
    HighDofWorld world_high_dof_;

    // RRTPlanner planner_;
    // RRTConnectPlanner planner_;
    RRTStarPlanner planner_;

    std::vector<double> start_;
    std::vector<double> goal_;
    int algo_;
    std::vector<std::vector<double> > path_;
};

void moveRobot(std::vector<std::vector<double> > path, EnvironmentBasePtr& env, std::ostream& sout){
    OpenRAVE::TrajectoryBasePtr traj = RaveCreateTrajectory(env,""); 
    std::vector<OpenRAVE::RobotBasePtr> robots;
    env->GetRobots(robots);
    OpenRAVE::RobotBasePtr robot = robots[0];
    traj->Init(robot->GetActiveConfigurationSpecification("linear"));
    size_t i = 0;
    for (auto it = path.begin(); it!=path.end(); ++it)
    {
        traj->Insert(i++,*it);
    }
    std::vector<dReal> vel;
    std::vector<dReal> acc;
    robot->GetActiveDOFVelocityLimits(vel);
    robot->GetActiveDOFAccelerationLimits(acc);
    OpenRAVE::planningutils::RetimeAffineTrajectory(traj,vel,acc);
    
    if(!silent)
    {
        std::cout<<"Total time for trajectory : "<<traj->GetDuration()<<std::endl;
        std::cout<<"DOFs in configuration : "<<robot->GetActiveConfigurationSpecification("linear").GetDOF()<<std::endl;
        std::cout<<"Number of waypoints in trajectory : "<<traj->GetNumWaypoints()<<std::endl;

        // robot->GetController()->SetPath(traj);
        // robot->SetMotion(traj);
    }

    traj->serialize(sout);
}

bool PlannerModule::runCommand(std::ostream& sout, std::istream& sinput)
{
    int max_iterations = 10000;
    parseInput(sinput);
    std::vector<double> lower_limit;
    std::vector<double> upper_limit;
    
    #ifdef PLANAR
    lower_limit.push_back(-5.0);
    lower_limit.push_back(-5.0);
    upper_limit.push_back(5.0);
    upper_limit.push_back(5.0);
    #endif

    std::vector<OpenRAVE::RobotBasePtr> robots;
    env_->GetRobots(robots);
    robots.front()->GetActiveDOFValues(start_);
    
    #ifdef ARM
    robots.front()->GetActiveDOFLimits(lower_limit, upper_limit); 
    #endif

    // for (int i=0; i<config_dim; ++i)
    // {
    //     lower_limit[i] = min_config_for_search[i] > lower_limit[i] ?
    //                      min_config_for_search[i] : lower_limit[i];
    //     upper_limit[i] = max_config_for_search[i] < upper_limit[i] ?
    //                      max_config_for_search[i] : upper_limit[i];
    // }

    World* chosen_world = nullptr;
    switch(algo_)
    {
        case(1):
            std::cout<<"RRT with potential on robot arm"<<std::endl;
            chosen_world = &world_high_dof_;
            break;
        case(2): 
            std::cout<<"RRT with potential"<<std::endl;
            chosen_world = &world_potential_;
            break;
        case(3): 
            std::cout<<"RRT"<<std::endl;
            chosen_world = &world_;
            break;
        default:
            std::cout<<"Wrong algorithm asked!!!"<<std::endl;
    }
    planner_.setWorld(chosen_world);
    chosen_world->setStart(start_.data());
    chosen_world->setGoal(goal_.data());
    chosen_world->setLowerLimits(lower_limit.data());
    chosen_world->setUpperLimits(upper_limit.data());

    std::cout<<"-----------Planning Problem-------------"<<std::endl;
    std::cout<<"   Start configuration : "<<chosen_world->getStart().transpose()<<std::endl;
    std::cout<<"   Goal configuration  : "<<chosen_world->getGoal().transpose()<<std::endl;
    std::cout<<"   Lower limit : "<<chosen_world->getLowerLimits().transpose()<<std::endl;
    std::cout<<"   Upper limit : "<<chosen_world->getUpperLimits().transpose()<<std::endl;
    // std::cout<<"   Goal bias : "<<planner_.goal_bias_<<std::endl;
    std::cout<<"   Step size : "<<step_size<<std::endl;
    std::cout<<"   Tolerance : "<<node_distance_tolerance<<std::endl;
    std::cout<<"----------------------------------------"<<std::endl;

    viz_objects_permanent.push_back(drawConfiguration(env_, chosen_world->getStart(), Green, 10));
    viz_objects_permanent.push_back(drawConfiguration(env_, chosen_world->getGoal(), Green, 10));

    if(chosen_world->isInCollision(chosen_world->getStart()))
    {
        std::cout<<"Start configuration in collision!!"<<std::endl;
        return false;
    }
    if(chosen_world->isInCollision(chosen_world->getGoal()))
    {
        std::cout<<"Goal configuration in collision!!"<<std::endl;
        return false;
    }

    viz_tree.clear();
    viz_objects_permanent.clear();

    if(planner_.plan(max_iterations))
        path_ = planner_.getPath();

    moveRobot(path_, env_, sout);

    return true;
}

void PlannerModule::parseInput(std::istream& sinput)
{
    std::string input;
    std::string temp;
    temp = ",";
    input = " ";
    int ctr = 0;
    float seed;

    while (input!="done")
    {
        ctr++;
        sinput>>input;
        temp = ",";
        if (input == "goal")
        {
            float n;
            while(temp!=";")
            {
                sinput>>n;
                goal_.push_back(n);
                sinput>>temp;
            }
        }
        else if(input == "algo")
        {
            sinput>>algo_;
            sinput>>temp;
        }
        else if(input == "seed")
        {
            sinput>>seed;
            sinput>>temp;
        }
        if (ctr>10) break;
    }
    srand(seed);
}

#endif