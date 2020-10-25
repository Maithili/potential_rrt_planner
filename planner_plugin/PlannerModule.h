#ifndef PLANNER_MODULE_H
#define PLANNER_MODULE_H

#include <boost/bind.hpp>
#include "Planner.h"

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
        ModuleBase(penv), env_(penv), world_potential_{env_}, world_(env_), planner_()
    {
        RegisterCommand("PlannerCommand",boost::bind(&PlannerModule::runCommand,this,_1,_2),
                        "algo 1/2; goal x,y,th ; goalbias 1; done");
    }

    virtual ~PlannerModule() {}

    bool runCommand(std::ostream& sout, std::istream& sinput);

    void parseInput(std::istream& sinput);

private:
    EnvironmentBasePtr env_;
    EuclideanWorldWithPotential world_potential_;
    EuclideanWorld world_;
    RRTPlanner planner_;
    std::vector<double> start_;
    std::vector<double> goal_;
    int algo_;
    std::vector<std::vector<double> > path_;
};

bool PlannerModule::runCommand(std::ostream& sout, std::istream& sinput)
{
    int max_iterations = 10000;
    parseInput(sinput);
    env_ = GetEnv();
    std::vector<double> lower_limit;
    lower_limit.push_back(-5.0);
    lower_limit.push_back(-5.0);
    std::vector<double> upper_limit;
    upper_limit.push_back(5.0);
    upper_limit.push_back(5.0);
    env_->GetRobot("PR2")->GetActiveDOFValues(start_);
    // env_->GetRobot("PR2")->GetActiveDOFLimits(lower_limit, upper_limit); 

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
            std::cout<<"RRT with potential"<<std::endl;
            chosen_world = &world_potential_;
            break;
        case(2): 
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
    std::cout<<"   Goal bias : "<<planner_.goal_bias_<<std::endl;
    std::cout<<"   Step size : "<<step_size<<std::endl;
    std::cout<<"   Tolerance : "<<node_distance_tolerance<<std::endl;
    std::cout<<"----------------------------------------"<<std::endl;

    viz_objects_permanent.push_back(drawConfiguration(env_, world_.getStart(), Green, 10));
    viz_objects_permanent.push_back(drawConfiguration(env_, world_.getGoal(), Green, 10));

    planner_.plan(max_iterations);
    path_ = planner_.getPath();

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
        else if(input == "goalbias")
        {
            sinput>>planner_.goal_bias_;
            sinput>>temp;
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