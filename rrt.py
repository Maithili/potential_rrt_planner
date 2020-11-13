#!/usr/bin/env python

import time
import openravepy
import sys
import scipy
import random

PROBLEM = "Planar"
# PROBLEM = "Arm"

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    print("wait robot")
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996])
        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

def setarmenv(e):
    # table
    table = e.ReadKinBodyXMLFile('models/furniture/rolly-table.iv')
    e.Add(table)
    table.SetTransform([0.70711,0.70711,0,0,0,0,0])

    # bottle (and its grasp)
    mug = e.ReadKinBodyXMLFile('models/objects/mug3.iv')
    e.Add(mug)
    mug.SetTransform([1,0,0,0,0,0,0.7])
    T_mug_palm = numpy.array(
    [[ 0, -1,  0, 0.000 ],
        [ 0,  0, -1, 0.075 ],
        [ 1,  0,  0, 0.100 ],
        [ 0,  0,  0, 1     ]])

    # robot
    r = e.ReadRobotXMLFile('robots/barrettwam_withspheres.robot.xml')
    e.Add(r)
    r.SetTransform([0.70711,0,0.70711,0,-1.0,0,1.0])

    # set up active manip, active dofs
    r.SetActiveManipulator('arm')
    m = r.GetActiveManipulator()
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(r,
    iktype=openravepy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    r.SetActiveDOFs(m.GetArmIndices())
    T_palm_ee = numpy.array(
    [[ 1., 0., 0., 0. ],
        [ 0., 1., 0., 0.  ],
        [ 0., 0., 1., 0.125 ],
        [ 0., 0., 0., 1. ]])

    # get IK solution for bottle
    T_ee = reduce(numpy.dot, [
    mug.GetTransform(),
    numpy.linalg.inv(T_mug_palm),
    T_palm_ee])
    q_goal = m.FindIKSolution(T_ee, 0)
    q_goal = [i for i in q_goal]
    print('q_goal:', q_goal)
    return str(q_goal)[1:-1]

def setarm(r):
    # set starting arm configuration
    # r.SetActiveDOFValues([2.75,-3.8,0.0,2.0,0.0,0.2,0.0])
    # r.SetActiveDOFValues([ 1.60645868, -0.99563845,  0.000 ,  2.50930208, -3.14159265, 1.51366363, -0.03566235])
    r.SetActiveDOFValues([1.60645868, -0.99563845, 0.00, 1.7930208, -3.14159265, 1.71366363, -0.23566235])
    r.GetController().SetDesired(r.GetDOFValues())
    # waitrobot(r)

def set2denv(e):
    e.Load('scenes/2D.env.xml')
    time.sleep(0.1)
    r = env.GetRobots()[0]
    tuckarms(env,r)
    r.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
    q_goal = [4,4,pi/4]
    r.GetController().SetDesired(r.GetDOFValues())
    # waitrobot(r)
    return str(q_goal)[1:-1]

def set2d(r):
    r.SetActiveDOFValues([-4, -4, 0])
    r.GetController().SetDesired(r.GetDOFValues())
    time.sleep(0.1)
    # waitrobot(r)

def setenv(e):
    if(PROBLEM == "Planar"): return set2denv(e)
    if(PROBLEM == "Arm"): return setarmenv(e)
    print("Updating published")
    e.UpdatePublishedBodies()
    time.sleep(0.1)

def setrobot(r):
    if(PROBLEM == "Planar"): set2d(r)
    if(PROBLEM == "Arm"): setarm(r)
    print("Updating published")
    time.sleep(0.1)


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)
    env.Reset()   
    goalstring = setenv(env)  
    robot = env.GetRobots()[0]
    RaveInitialize()
    RaveLoadPlugin('planner_plugin/build/planner_plugin')

    try:
        with env:
            mod = RaveCreateModule(env,'PlannerModule')
            print(mod)
            seed = random.random()*10000
            goalbias = 50 # in percentage

            if(PROBLEM == "Planar") : algo = 2
            if(PROBLEM == "Arm") : algo = 1
            setrobot(robot)
            env.UpdatePublishedBodies()
            print("PlannerCommand algo %f ; seed %f ; goal %s ; goalbias %f ; done" %(algo,seed,goalstring,float(goalbias)/100))
            raw_input("Press enter to start...")
            start = time.clock()
            traj_pot = mod.SendCommand("PlannerCommand algo %f ; seed %f ; goal %s ; goalbias %f ; done" %(algo,seed,goalstring,float(goalbias)/100))
            end = time.clock()
            print 'Total time for RRT with potential : ', end - start

            if(PROBLEM == "Planar") : algo = 2
            else: raise
            setrobot(robot)
            print("PlannerCommand algo %f ; seed %f ; goal %s ; goalbias %f ; done" %(algo,seed,goalstring,float(goalbias)/100))
            raw_input("Press enter to start...")
            start = time.clock()
            traj_rrt = mod.SendCommand("PlannerCommand algo %f ; seed %f ; goal %s ; goalbias %f ; done" %(algo,seed,goalstring,float(goalbias)/100))
            end = time.clock()
            print 'Total time for RRT : ', end - start
    except Exception as e:
        print(e)
        print 'Error on line {}'.format(sys.exc_info()[-1].tb_lineno)
    finally:
        raw_input("Press enter to exit...")