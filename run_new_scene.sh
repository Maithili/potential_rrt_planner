#! /bin/bash
python scenes/make_random_obstacles.py > scenes/random_obstacles.xml
./make_planner.sh && ./rrt.py