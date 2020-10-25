#!/usr/bin/env python

import time
import openravepy
import sys
import scipy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)
    env.Reset()     
    env.Load('2D.env.xml')
    time.sleep(0.1)

    try:
        time.sleep(0.1)
    except Exception as e:
        print(e)
        print 'Error on line {}'.format(sys.exc_info()[-1].tb_lineno)
    finally:
        raw_input("Press enter to exit...")