#!/usr/bin/env python
import matplotlib.pyplot as plt

import time
import openravepy
import sys
import scipy
import random
from math import sqrt

PROBLEM = "Planar"
# PROBLEM = "Arm"

interactive = True

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

def getcollisionfraction(env,robot):
    collisions = 0
    non_collisions = 0
    for x in range(-50,50):
        for y in range(-50,50):
            robot.SetActiveDOFValues([float(x)/10, float(y)/10, random.random()*3.14])
            if (env.CheckCollision(robot)) : collisions = collisions + 1
            else : non_collisions = non_collisions + 1
    return (float(collisions)/float(collisions+non_collisions))

def dist(a,b):
    return numpy.linalg.norm(numpy.array([a[0]-b[0],a[1]-b[1]]))

def area(a,b,c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

def threepointcurvature(a,b,c):
    return (4*area(a,b,c))/(dist(a,b)*dist(b,c)*dist(c,a))

def curvatureRMS(traj):
    n = traj.GetNumWaypoints()
    n = n-2
    curvatures = []
    for i in range(n):
        p1 = traj.GetWaypoint(i)
        p2 = traj.GetWaypoint(i+1)
        p3 = traj.GetWaypoint(i+2)
        curvatures.append(threepointcurvature(p1,p2,p3))
    return numpy.linalg.norm(numpy.array(curvatures))/sqrt(n)

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
    r = e.GetRobots()[0]
    tuckarms(e,r)
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

def runplanners():
    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)
    env.Reset()   
    goalstring = setenv(env)  
    robot = env.GetRobots()[0]
    RaveInitialize()
    RaveLoadPlugin('planner_plugin/build/planner_plugin')
    results = {}
    results['CurvatureRMS'] = {}
    results['Time'] = {}
    results['PathDuration'] = {}

    try:
        with env:
            mod = RaveCreateModule(env,'PlannerModule')
            print(mod)
            seed = random.random()*10000

            if(PROBLEM == "Planar") : algo = 2
            if(PROBLEM == "Arm") : algo = 1
            setrobot(robot)
            env.UpdatePublishedBodies()
            print("PlannerCommand algo %f ; seed %f ; goal %s ; done" %(algo,seed,goalstring))
            if(interactive) : raw_input("Press enter to start...")
            start = time.clock()
            traj_pot = mod.SendCommand("PlannerCommand algo %f ; seed %f ; goal %s ; done" %(algo,seed,goalstring))
            end = time.clock()
            if traj_pot is None: raise ValueError
            results['Time']['potential'] = end - start
            print 'Total time for RRT with potential : ', results['Time']['potential']
            t = openravepy.RaveCreateTrajectory(env,'').deserialize(traj_pot)
            results['PathDuration']['potential'] = t.GetDuration()
            print 'Trajectory will take ',results['PathDuration']['potential'],'s to execute'
            results['CurvatureRMS']['potential'] = curvatureRMS(t)

            if(PROBLEM == "Planar") : algo = 3
            else: raise ValueError
            setrobot(robot)
            print("PlannerCommand algo %f ; seed %f ; goal %s ; done" %(algo,seed,goalstring))
            if(interactive) : raw_input("Press enter to start...")
            start = time.clock()
            traj_rrt = mod.SendCommand("PlannerCommand algo %f ; seed %f ; goal %s ; done" %(algo,seed,goalstring))
            end = time.clock()
            results['Time']['vanilla'] = end - start
            print 'Total time for RRT : ', results['Time']['vanilla']
            t = openravepy.RaveCreateTrajectory(env,'').deserialize(traj_rrt)
            results['PathDuration']['vanilla'] = t.GetDuration()
            print 'Trajectory will take ',results['PathDuration']['vanilla'],'s to execute'
            results['CurvatureRMS']['vanilla'] = curvatureRMS(t)
            
            results['CollisionFraction'] = getcollisionfraction(env, robot)
 
    except Exception as e:
        print(e)
        print 'Error on line {}'.format(sys.exc_info()[-1].tb_lineno)
    finally:
        if(interactive) : raw_input("Press enter to exit...")
        print("Destroying...")

    return results

def runfewtimes():
    time_potential=[]
    time_vanilla=[]
    pathduration_potential=[]
    pathduration_vanilla=[]
    for i in range(2):
        res = None
        try:
            res = runplanners()
        except:
            if res is None:
                continue
        print(res)
        time_potential.append(res['Time']['potential'])
        time_vanilla.append(res['Time']['vanilla'])
        pathduration_potential.append(res['PathDuration']['potential'])
        pathduration_vanilla.append(res['PathDuration']['vanilla'])
    
    plt.subplot(1,2,1)
    plot(time_potential,label="Potential")
    plot(time_vanilla,label="Vanilla")
    plt.legend()

    plt.subplot(1,2,2)
    plot(pathduration_potential,label="Potential")
    plot(pathduration_vanilla,label="Vanilla")
    plt.legend()

    plt.show()


if __name__ == "__main__":
    res = runplanners()
    print(res)
    if res and not interactive:
        f = open("results_11_15.txt", "a")
        f.write('\n'+str(res['Time']['potential'])+','+str(res['Time']['vanilla'])+','+str(res['CurvatureRMS']['potential'])+','+str(res['CurvatureRMS']['vanilla'])+','+str(res['CollisionFraction'])+';')
        f.close()
    RaveDestroy()
