#ifndef PLANNER_MODULE_H
#define PLANNER_MODULE_H

#include <boost/bind.hpp>
#include "Planner.h"

using namespace std;
using namespace OpenRAVE;

struct PlannerParams
{
    float goalbias = 0.5;
    float stepLength = 1;
    float stepLength_fine = 0.07;
    float goal_resolution = 0.14; // stepLength_fine*2;
};

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
        ModuleBase(penv), env_(penv), world_(env_), planner_(&world_)
    {
        RegisterCommand("PlannerCommand",boost::bind(&PlannerModule::runCommand,this,_1,_2),
                        "algo 1/2; goal x,y,th ; goalbias 1; done");
    }

    virtual ~PlannerModule() {}

    bool runCommand(std::ostream& sout, std::istream& sinput);

    void parseInput(std::istream& sinput);

    void visualizePath();

    void moveRobot();

private:
    EnvironmentBasePtr env_;
    EuclideanWorldWithPotential world_;
    RRTPlanner planner_;
    PlannerParams params_;
    std::vector<double> start_;
    std::vector<double> goal_;
    int algo_;
    std::vector<std::vector<double> > path_;
};

bool PlannerModule::runCommand(std::ostream& sout, std::istream& sinput)
{
    int max_iterations = 1000;
    parseInput(sinput);
    env_ = GetEnv();
    std::vector<double> lower_limit;
    std::vector<double> upper_limit;
    env_->GetRobot("PR2")->GetActiveDOFValues(start_);
    env_->GetRobot("PR2")->GetActiveDOFLimits(lower_limit, upper_limit); 

    world_.setStart(start_.data());
    world_.setGoal(goal_.data());
    world_.setLowerLimits(lower_limit.data());
    world_.setUpperLimits(upper_limit.data());

    std::cout<<"Calling plan()..."<<std::endl;

    planner_.plan(max_iterations);
    return true;
    path_ = planner_.getPath();

    // visualizePath("red", sout);
    // moveRobot();
    
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
            sinput>>params_.goalbias;
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