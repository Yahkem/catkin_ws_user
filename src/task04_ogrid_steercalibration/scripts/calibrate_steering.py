#!/usr/bin/env python

import rospy
import numpy as np
import sys
# import shapely
import math
import time
# from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16

class speed_controller(object):
    
    def __init__(self):
        self.pub_speed = rospy.Publisher("manual_control/speed", Int16, queue_size=10)
        self.drive_duration = 4

    def start(self):
        rospy.loginfo("starting...")
        self.pub_speed.publish(100) #TODO prijit na spravnou rychlost
    
    def stop(self):
        rospy.loginfo("stopping...")
        self.pub_speed.publish(0)


class steering_controller(object):

    def __init__(self):
        self.pub_steer = rospy.Publisher("manual_control/steering", Int16, queue_size=10)

    def steer(self, arg):
        self.pub_steer.publish(arg)

class scan_receiver(object):
    
    def __init__(self):
        self.rad180deg = np.deg2rad(180)
        self.index_offset = 12 #offset of scan_msg.ranges[] list - angles used for measuring
        self.should_measure = False
        self.measure2 = False
        self.msrmt = None
        rospy.Subscriber("scan", LaserScan, self.scan_recieved, queue_size=10) #comment before test, measure Theta
        # rospy.Subscriber("scan", LaserScan, self.listen_theta, queue_size=10) #uncomment before test, measure Theta


    def set_measure1(self):
        #because of lambda
        self.should_measure = True
        self.measure2 = False

    def set_measure2(self):
        self.should_measure = True
        self.measure2 = True

    # cb for initial theta setting - closest to 0deg
    def listen_theta(self, scan_msg):
        angle_min = scan_msg.angle_min
        angle_cur = angle_min
        angle_max = scan_msg.angle_max
        angle_inc = scan_msg.angle_increment
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max
        
        # print "MIN=%s;MAX=%s;INC=%s" % (angle_min, angle_max, angle_inc)
        a = scan_msg.ranges[self.index_offset]
        b = scan_msg.ranges[-self.index_offset - 1] # -1, because it ends at PI, but starts at -PI+angle_inc
        #print "a=%s; b=%s" % (self.msrmt.a1, self.msrmt.b1)

        angle_diff = angle_inc*self.index_offset
        alfa = angle_diff*2
        #self.msrmt.alfa = alfa # alfa1
        #print "Alfa=%s"%alfa

        #comment
        # angle1 = angle_min+angle_diff
        # angle2 = angle_max-angle_diff
        # print "A1=%s;A2=%s;DIFF=%s" % (np.rad2deg(angle1), np.rad2deg(angle2), np.rad2deg(angle_diff))
        #comment

        #print "PROSTREDEK=%s;%s;%s\n" % (scan_msg.ranges[-1],scan_msg.ranges[0], scan_msg.ranges[1])

        #vzorce
        # alfa = angle_diff*2
        
        c = math.sqrt(a*a + b*b - 2*a*b*np.cos(alfa))
        phi = math.asin((b*math.sin(alfa)) / c)
        beta = self.rad180deg - phi - angle_diff

        sin_phi = math.sin(phi)
        x = (a*sin_phi) / math.sin(beta)
        k = sin_phi * a
        theta = math.acos(k/x)

        if not math.isnan(theta):
            print("Theta=%s" % np.rad2deg(theta))


    def scan_recieved(self, scan_msg):
        if not self.should_measure: return

        angle_min = scan_msg.angle_min
        angle_cur = angle_min
        angle_max = scan_msg.angle_max
        angle_inc = scan_msg.angle_increment
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max
        
        # print "MIN=%s;MAX=%s;INC=%s" % (angle_min, angle_max, angle_inc)
        a = scan_msg.ranges[self.index_offset]
        b = scan_msg.ranges[-self.index_offset - 1] # -1, because it ends at PI, but starts at -PI+angle_inc

        # we don't want to measure invalid values
        if math.isnan(a) or math.isnan(b) or math.isinf(a) or math.isinf(b):
            return

        print "a=%s; b=%s" % (a, b)

        angle_diff = angle_inc*self.index_offset
        alfa = angle_diff*2
        self.msrmt.alfa = alfa # alfa1

        #comment
        #angle1 = angle_min+angle_diff
        #angle2 = angle_max-angle_diff
        #print "A1=%s;A2=%s;DIFF=%s" % (np.rad2deg(angle1), np.rad2deg(angle2), np.rad2deg(angle_diff))
        #comment

        #print "PROSTREDEK=%s;%s;%s\n" % (scan_msg.ranges[-1],scan_msg.ranges[0], scan_msg.ranges[1])

        #vzorce
        # alfa = angle_diff*2
        
        c = math.sqrt(a*a + b*b - 2*a*b*np.cos(alfa))
        phi = math.asin((b*math.sin(alfa)) / c)
        beta = self.rad180deg - phi - angle_diff

        sin_phi = math.sin(phi)
        x = (a*sin_phi) / math.sin(beta)
        k = sin_phi * a
        theta = math.acos(k/x)

        print("c=%s\nphi=%s\nbeta=%s\nx=%s\nk=%s\ntheta=%s\n\n") % (c,np.rad2deg(phi),np.rad2deg(beta),x,k,np.rad2deg(theta))
        # TODO save results somewhere -> to angles_deg

        if self.measure2:
            self.msrmt.a2 = a
            self.msrmt.b2 = b
            self.msrmt.c2 = c
            self.msrmt.phi2 = phi
            self.msrmt.beta2 = beta
            self.msrmt.theta2 = theta
            self.msrmt.x2 = x
            self.msrmt.k2 = k
            #TODO asign vals to msrt, final calc,print etc..
            self.msrmt.calculate_after_measuring()
        else:
            self.msrmt.a1 = a
            self.msrmt.b1 = b
            self.msrmt.c1 = c
            self.msrmt.beta1 = beta
            self.msrmt.phi1 = phi
            self.msrmt.x1 = x
            self.msrmt.k1 = k
            self.msrmt.theta1 = theta

        self.measure2 = False
        self.should_measure = False
    # def calculate_opposite_side(self, a, b, alfa):
    #     # cosine law
    #     return math.sqrt(a*a + b*b - 2*a*b*np.cos(alfa))

    # def calculate_phi(self, b, c, alfa):
    #     return math.asin((b*math.sin(alfa)) / c)

WHEEL_DISTANCE = 0.26

class measurement(object):
    def __init__(self, angle_arg):
        self.angle_arg = angle_arg
        self.a1 = 0
        self.b1 = 0
        self.c1 = 0
        self.alfa = 0
        self.phi1 = 0
        self.beta1 = 0
        self.theta1 = 0
        self.x1 = 0
        self.k1 = 0 # perpendicular
        self.a2 = 0
        self.b2 = 0
        self.c2 = 0
        self.phi2 = 0
        self.beta2 = 0
        self.theta2 = 0
        self.x2 = 0
        self.k2 = 0 # perpendicular
        # self.angle_result = 0
        # TODO R, L-const, 
        #results
        self.d = 0
        self.r = 0
        self.gamma = 0 # angle_result

    def calculate_after_measuring(self):
        self.d = self.k2 - self.k1
        self.r = self.d/ math.sin(self.theta2)
        self.gamma = math.asin(WHEEL_DISTANCE/self.r)

        print "d=%s\nr=%s\ngamma=%s" % (self.d, self.r, np.rad2deg(self.gamma))


angles_deg = [
    [0, None],
    [30, None],
    [60, None],
    [90, 2.97257469303],
    [120, None],
    [150, None],
    [179, None]
]

def print_table(angles_deg):
    print "\n|--Argument--|--Real angle--|"

    for p in angles_deg:
        print "|" + "{0}".format(p[0]).rjust(7).ljust(12) + "|" + "{0}".format(p[1]).rjust(8).ljust(14) + "|"
    
    print "|------------|--------------|"
    

def perform_test(spd_ctrl, str_ctrl, scan_rcv, msrmt):
    scan_rcv.set_measure1()
    scan_rcv.msrmt = msrmt
    #str_ctrl.steer(90) #straighten
    spd_ctrl.stop() # sicher
    time.sleep(0.5)
    # TODO measure1
    str_ctrl.steer(msrmt.angle_arg)
    time.sleep(1) # wait for measurement
    spd_ctrl.start()
    rospy.Timer(rospy.Duration(spd_ctrl.drive_duration), lambda _: spd_ctrl.stop(), oneshot=True)
    
    # measures after this
    rospy.Timer(rospy.Duration(spd_ctrl.drive_duration+0.1), lambda _: scan_rcv.set_measure2(), oneshot=True)

    time.sleep(0.5)
    #str_ctrl.steer(90) #straighten
    

def main(args):
    # print np.rad2deg(0.0518812156548); return
    rospy.init_node("calibrate_steering")

    speed_control = speed_controller()
    steer_control = steering_controller()
    scan_rcv = scan_receiver()

    # measure k
    #1.steer
    #2.start
    #3.after x sec (2?) stop 
    # measure k, theta, Gamma!
    m = measurement(0)
    perform_test(speed_control, steer_control, scan_rcv, m) #comment before test, measure Theta

    #print_table(angles_deg)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
