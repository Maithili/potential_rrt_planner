import random
import math
import numpy as np
from matplotlib import pyplot

x_lim = [-4,4]
y_lim = [-4,4]
start = [-4,-4]
goal = [4,4]
cnt = 0

rectangles = []
circles = []

class Rectangle:
    def __init__(self,x,y,w,h,th):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.th = th

class Circle:
    def __init__(self,x,y,r):
        self.x = x
        self.y = y
        self.r = r

def intersect_cc(c1, c2):
    dist = np.linalg.norm(np.array(((c1.x-c2.x), (c1.y-c2.y))))
    return (dist<(c1.r+c2.r))

def intersect_rc(rect, circ):
    rel_x = (circ.x - rect.x) * math.cos(rect.th) - (circ.y - rect.y) * math.sin(rect.th)
    rel_y = (circ.x - rect.x) * math.sin(rect.th) + (circ.y - rect.y) * math.cos(rect.th)
    return ((abs(rel_x) < rect.w+circ.r) and (abs(rel_y) < rect.h+circ.r))

def intersect_rr(r1, r2):
    radius1 = np.linalg.norm(np.array((r1.w, r1.h)))
    circ1 = Circle(r1.x, r1.y, radius1)
    radius2 = np.linalg.norm(np.array((r2.w, r2.h)))
    circ2 = Circle(r2.x, r2.y, radius2)
    return (intersect_rc(r1, circ2) or intersect_rc(r2, circ1))

def circle_intersects_anything(circ):
    global rectangles 
    global circles
    for r in rectangles:
        if(intersect_rc(r, circ)):
            return True
    for c in circles:
        if(intersect_cc(c, circ)):
            return True
    return False

def rectangle_intersects_anything(rect):
    global rectangles 
    global circles
    for r in rectangles:
        if(intersect_rr(rect, r)):
            return True
    for c in circles:
        if(intersect_rc(rect, c)):
            return True
    return False

def make_square(rect):
    center = np.array((rect.x,rect.y))
    rot = np.matrix(((math.cos(rect.th),-math.sin(rect.th)),(math.sin(rect.th),math.cos(rect.th))))
    p1 = center + np.matmul(rot, np.array(( rect.w,  rect.h)))
    p2 = center + np.matmul(rot, np.array((-rect.w,  rect.h)))
    p3 = center + np.matmul(rot, np.array(( rect.w, -rect.h)))
    p4 = center + np.matmul(rot, np.array((-rect.w, -rect.h)))
    return(p1,p2,p3,p4)

def rand_x():
    return random.random()*(x_lim[1]-x_lim[0])+x_lim[0]

def rand_y():
    return random.random()*(y_lim[1]-y_lim[0])+y_lim[0]

def rand_ext():
    return random.random() * 1.0 + 0.1

def rand_th():
    return random.random() * 180

def print_header():
    print"<KinBody name=\"obstacles\">"
    print"    <RotationAxis>0 0 0 0</RotationAxis>"
    print"    <Translation>0 0 0.2</Translation>"

def print_footer():
    print"</KinBody>"

def print_box(rect):
    global rectangles
    rectangles.append(rect)
    if(np.linalg.norm(np.array((rect.x,rect.y))-np.array(start)) < min(rect.w,rect.h)):
        return
    if(np.linalg.norm(np.array((rect.x,rect.y))-np.array(goal)) < min(rect.w,rect.h)):
        return
    global cnt
    print"    <Body name=\"",cnt,"\" type=\"static\">"
    print"        <Geom type=\"box\">"
    print"            <Extents>",rect.w,rect.h," 0.1</Extents>"
    print"            <RotationAxis> 0 0 1 ",rect.th,"</RotationAxis>"
    print"            <Translation>",rect.x,rect.y," 0</Translation>"
    print"            <diffuseColor>1 1 1</diffuseColor>"
    print"        </Geom>"
    print"    </Body>"
    print""
    cnt = cnt+1

def print_circle(circ):
    global circles
    circles.append(circ)
    if(np.linalg.norm(np.array((circ.x,circ.y))-np.array(start)) < circ.r):
        return
    if(np.linalg.norm(np.array((circ.x,circ.y))-np.array(goal)) < circ.r):
        return
    global cnt
    print"    <Body name=\"",cnt,"\" type=\"static\">"
    print"        <Geom type=\"cylinder\">"
    print"            <radius>",circ.r,"</radius>"
    print"            <height>0.2</height>"
    print"            <Translation>",circ.x,circ.y," 0</Translation>"
    print"            <RotationAxis>1 0 0 90</RotationAxis>"
    print"            <diffuseColor>1 1 1</diffuseColor>"
    print"        </Geom>"
    print"    </Body>"
    cnt = cnt+1

def main():
    area_left = 25.0*0.4
    print_header()
    while(area_left > 0):
        rect = Rectangle(rand_x(),rand_y(),rand_ext(),rand_ext(),rand_th())
        while(rectangle_intersects_anything(rect)):
            rect = Rectangle(rand_x(),rand_y(),rand_ext(),rand_ext(),rand_th())
        print_box(rect)
        area_left -= rect.w * rect.h
        circ = Circle(rand_x(),rand_y(),rand_ext())
        while (circle_intersects_anything(circ)):
            circ = Circle(rand_x(),rand_y(),rand_ext())
        print_circle(circ)
        area_left -= math.pi * circ.r * circ.r
    print_footer()

if __name__ == "__main__":
    main()