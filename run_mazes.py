#!/usr/bin/env python
import matplotlib.pyplot as plt

import time
import openravepy
import sys
import scipy
import random
from math import sqrt

# WORLD = "maze1"
# START = [-20, 20, 0]
# GOAL = [12.5, -20, 0]

WORLD = "maze2"
START = [-40, -40, 0]
GOAL = [40, 40, 0]

# WORLD = "2d1"
# START = [-10, -10, 0]
# GOAL = [20, -10, 0]

# WORLD = "2d2"
# START = [-20, -5, 0]
# GOAL = [30, -5, 0]

interactive = False

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996])
        robot.GetController().SetDesired(robot.GetDOFValues())
    print("wait robot")
    while not robot.GetController().IsDone():
        time.sleep(0.01)

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

def setenv(e):
    e.Load('scenes/Maze.env.xml')
    time.sleep(0.1)
    obstacles = e.ReadKinBodyXMLFile('scenes/'+WORLD+'_obstacles.xml')
    e.Add(obstacles)
    time.sleep(0.1)
    r = e.GetRobots()[0]
    tuckarms(e,r)
    r.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
    q_goal = GOAL
    r.GetController().SetDesired(r.GetDOFValues())
    e.UpdatePublishedBodies()
    return str(q_goal)[1:-1]

def setrobot(r):
    r.SetActiveDOFValues(START)
    r.GetController().SetDesired(r.GetDOFValues())
    time.sleep(0.1)

def runplanner(module, algo, world, seed, goalstring, env):
    robot = env.GetRobots()[0]
    run_result = {}
    setrobot(robot)
    env.UpdatePublishedBodies()
    print("PlannerCommand algo %f ; seed %f ; goal %s ; done" %(algo,seed,goalstring))
    if(interactive) : raw_input("Press enter to start...")
    start = time.clock()
    traj = module.SendCommand("PlannerCommand algo %f ; world %f; seed %f ; goal %s ; done" %(algo,world,seed,goalstring))
    end = time.clock()
    if not traj: return None
    t = openravepy.RaveCreateTrajectory(env,'').deserialize(traj)
    run_result['Time'] = end - start
    run_result['PathDuration'] = t.GetDuration()
    run_result['CurvatureRMS'] = curvatureRMS(t)
    print 'Total time : ', run_result['Time']
    return run_result

def run():
    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)
    env.Reset()   
    goalstring = setenv(env)  
    
    RaveInitialize()
    RaveLoadPlugin('planner_plugin/build/planner_plugin')
    results = {}
    results['RRT'] = {}
    results['RRTconnect'] = {}
    results['RRTstar'] = {}

    try:
        with env:
            mod = RaveCreateModule(env,'PlannerModule')
            print(mod)
            seed = random.random()*10000
            world = 2
            results['RRT']['Potential'] = runplanner(mod, 1, world, seed, goalstring, env)
            if results['RRT']['Potential'] is None: return {}
            results['RRTconnect']['Potential'] = runplanner(mod, 2, world, seed, goalstring, env)
            if results['RRTconnect']['Potential'] is None: return {}
            results['RRTstar']['Potential'] = runplanner(mod, 3, world, seed, goalstring, env)
            if results['RRTstar']['Potential'] is None: return {}

            world = 1
            results['RRT']['Vanilla'] = runplanner(mod, 1, world, seed, goalstring, env)
            if results['RRT']['Vanilla'] is None: return {}
            results['RRTconnect']['Vanilla'] = runplanner(mod, 2, world, seed, goalstring, env)
            if results['RRTconnect']['Vanilla'] is None: return {}
            results['RRTstar']['Vanilla'] = runplanner(mod, 3, world, seed, goalstring, env)
            if results['RRTstar']['Vanilla'] is None: return {}
 
    except Exception as e:
        print(e)
        print 'Error on line {}'.format(sys.exc_info()[-1].tb_lineno)
    finally:
        if(interactive) : raw_input("Press enter to exit...")
        print("Destroying...")

    return results

def serialize(results):
    s = ''

    s = s + str(res['RRT']['Potential']['Time']) + ','
    s = s + str(res['RRTconnect']['Potential']['Time']) + ','
    s = s + str(res['RRTstar']['Potential']['Time']) + ','
    s = s + str(res['RRT']['Vanilla']['Time']) + ','
    s = s + str(res['RRTconnect']['Vanilla']['Time']) + ','
    s = s + str(res['RRTstar']['Vanilla']['Time']) + ','

    s = s + str(res['RRT']['Potential']['CurvatureRMS']) + ','
    s = s + str(res['RRTconnect']['Potential']['CurvatureRMS']) + ','
    s = s + str(res['RRTstar']['Potential']['CurvatureRMS']) + ','
    s = s + str(res['RRT']['Vanilla']['CurvatureRMS']) + ','
    s = s + str(res['RRTconnect']['Vanilla']['CurvatureRMS']) + ','
    s = s + str(res['RRTstar']['Vanilla']['CurvatureRMS'])

    return s

if __name__ == "__main__":
    res = run()
    if res and not interactive:
        f = open("results_"+WORLD+".txt", "a")
        f.write('\n'+serialize(res)+';')
        f.close()
    RaveDestroy()
