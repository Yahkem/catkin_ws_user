#!/usr/bin/env python

import rospy
import numpy as np
import sys
import math
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16

def calculate_angles(a,b):
    alfa = 0.418879017234 #angle of car IP 111
    angle_diff = alfa/2
    
    PI = math.pi # np.deg2rad(180)
    theta_multiplier = 1

    print "a=%s; b=%s" % (a,b)
    if b > a:
        b,a=a,b
        theta_multiplier = -1

    c = math.sqrt(a*a + b*b - 2*a*b*np.cos(alfa)) # 3rd side of triangle a,b,c - Law of cosine
    phi = math.asin((b*math.sin(alfa)) / c) # angle between a,c - Law of sine
    beta = PI - phi - alfa # angle between b,c
    omega = PI - phi - angle_diff # angle between x,c' in triangle a,x,c'
    
    sin_phi = math.sin(phi)
    x = (a*sin_phi) / math.sin(omega) # 'ray' from 'middle wheel'
    k = sin_phi*a #if a>b else math.sin(beta)*b # perpendicular from car to the table
    theta = math.acos(k/x) * theta_multiplier # angle between 'middle wheel' and the perpendicular

    print("c=%s\nphi=%s\nbeta=%s\nx=%s\nk=%s\ntheta=%s\n\n") % (c,np.rad2deg(phi),np.rad2deg(beta),x,k,np.rad2deg(theta))

def main(args):
    rospy.init_node("test_theta")


    # print("alfa=%s" % np.rad2deg(alfa))
    # a = input("a=") # 0.33599999547
    # b = input("b=") # 0.412000000477

    rg = range(3, 8)
    a = 3
    b = 5

    print ("b=5-------------------------------------------")

    for a in rg:
        calculate_angles(a,b)
        time.sleep(0.3)

    a = 5
    b = 3

    print ("a=5-------------------------------------------")

    for b in rg:
        calculate_angles(a,b)
        time.sleep(0.3)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
