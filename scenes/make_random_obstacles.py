import random
import math
import numpy as np
from matplotlib import pyplot

x_lim = [-4,4]
y_lim = [-4,4]
start = [-4,-4]
goal = [4,4]
cnt = 0

def make_square(x,y,w,h,th):
    center = np.array((x,y))
    rot = np.matrix(((math.cos(th),-math.sin(th)),(math.sin(th),math.cos(th))))
    p1 = center + np.matmul(rot, np.array(( w,  h)))
    p2 = center + np.matmul(rot, np.array((-w,  h)))
    p3 = center + np.matmul(rot, np.array(( w, -h)))
    p4 = center + np.matmul(rot, np.array((-w, -h)))
    return(p1,p2,p3,p4)

def rand_x():
    return random.random()*(x_lim[1]-x_lim[0])+x_lim[0]

def rand_y():
    return random.random()*(y_lim[1]-y_lim[0])+y_lim[0]

def rand_ext():
    return random.random() * 1.0

def rand_th():
    return random.random() * 180

def print_header():
    print"<KinBody name=\"obstacles\">"
    print"    <RotationAxis>0 0 0 0</RotationAxis>"
    print"    <Translation>0 0 0.2</Translation>"

def print_footer():
    print"</KinBody>"

def print_box(x,y,w,h,t):
    if(np.linalg.norm(np.array((x,y))-np.array(start)) < min(w,h)):
        return
    if(np.linalg.norm(np.array((x,y))-np.array(goal)) < min(w,h)):
        return
    global cnt
    print"    <Body name=\"",cnt,"\" type=\"static\">"
    print"        <Geom type=\"box\">"
    print"            <Extents>",w,h," 0.1</Extents>"
    print"            <RotationAxis> 0 0 1 ",t,"</RotationAxis>"
    print"            <Translation>",x,y," 0</Translation>"
    print"            <diffuseColor>1 1 1</diffuseColor>"
    print"        </Geom>"
    print"    </Body>"
    print""
    cnt = cnt+1

def print_circle(x,y,r):
    if(np.linalg.norm(np.array((x,y))-np.array(start)) < r):
        return
    if(np.linalg.norm(np.array((x,y))-np.array(goal)) < r):
        return
    global cnt
    print"    <Body name=\"",cnt,"\" type=\"static\">"
    print"        <Geom type=\"cylinder\">"
    print"            <radius>",r,"</radius>"
    print"            <height>0.2</height>"
    print"            <Translation>",x,y," 0</Translation>"
    print"            <RotationAxis>1 0 0 90</RotationAxis>"
    print"            <diffuseColor>1 1 1</diffuseColor>"
    print"        </Geom>"
    print"    </Body>"
    cnt = cnt+1

def main():
    print_header()
    for i in range(5):
        print_box(rand_x(),rand_y(),rand_ext(),rand_ext(),rand_th())
    for i in range(10):
        print_circle(rand_x(),rand_y(),rand_ext())
    print_footer()
    


if __name__ == "__main__":
    main()