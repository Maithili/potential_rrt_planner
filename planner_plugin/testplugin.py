#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/planner_plugin')
try:
    env=Environment()
    env.Load('../scenes/myscene.env.xml')
    Planner = RaveCreateModule(env,'PlannerModule')
    print(Planner)
    print('Merry...')
    print Planner.SendCommand('TestCommand testing')
    print('Christmas!')
finally:
    RaveDestroy()