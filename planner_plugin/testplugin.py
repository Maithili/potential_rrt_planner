#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/planner_plugin')
RaveLoadPlugin('build/chomp/chomp')
try:
    env=Environment()
    env.Load('../scenes/myscene.env.xml')
    # Planner = RaveCreateModule(env,'PlannerModule')
    Planner = RaveCreateModule(env,'orcdchomp')
    print(Planner)
    print('Hahahaa!')
    # print Planner.SendCommand('PlannerCommand testing')
    print Planner.SendCommand('viewspheres PR2')
    print('Hohoho!')
finally:
    RaveDestroy()