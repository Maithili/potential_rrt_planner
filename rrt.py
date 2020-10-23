#!/usr/bin/env python

import time
import openravepy
import sys
import scipy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996])
        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)
    env.Reset()     
    env.Load('scenes/2D.env.xml')
    time.sleep(0.1)
    robot = env.GetRobots()[0]
    tuckarms(env,robot)

    RaveInitialize()
    RaveLoadPlugin('planner_plugin/build/planner_plugin')
  
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
    goalconfig = [4,4,pi/4]
    robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    try:
        with env:
            mod = RaveCreateModule(env,'PlannerModule')
            start = time.clock()
            seed = 1
            if (len(sys.argv)>1):
                algo = sys.argv[1]
            else:
                algo = '1'
            if (algo == '2'):
                print 'potential'
                goalbias = 80
            else:
                print 'RRT'
                goalbias = 30
            print mod.SendCommand("PlannerCommand algo %s ; seed %f ; goal %f , %f , %f ; goalbias %f ; done" %(algo,seed,goalconfig[0],goalconfig[1],goalconfig[2],float(goalbias)/100))
            end = time.clock()
            print end - start
        waitrobot(robot)
    except Exception as e:
        print(e)
        print 'Error on line {}'.format(sys.exc_info()[-1].tb_lineno)
    finally:
        raw_input("Press enter to exit...")