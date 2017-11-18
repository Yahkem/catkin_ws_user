#!/usr/bin/env python

import rospy
import numpy as np
import sys
import math
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16

class speed_controller(object):
    
    def __init__(self):
        self.pub_speed = rospy.Publisher("manual_control/speed", Int16, queue_size=1)
        self.drive_duration = 3

    def start(self):
        rospy.loginfo("starting...")
        self.pub_speed.publish(100) #100=real speed
    
    def stop(self):
        rospy.loginfo("stopping...")
        self.pub_speed.publish(0)


class steering_controller(object):

    def __init__(self):
        self.pub_steer = rospy.Publisher("manual_control/steering", Int16, queue_size=1)

    def steer(self, arg):
        self.pub_steer.publish(arg)

class scan_receiver(object):
    
    def __init__(self, setup_before_measuring):
        self.index_offset = 12 #offset of scan_msg.ranges[] list - angles used for measuring
        self.should_measure = False
        self.measure2 = False
        self.msrmt = None

        if setup_before_measuring:
            rospy.Subscriber("scan", LaserScan, self.listen_theta, queue_size=1) #closest to 0
        else:
            rospy.Subscriber("scan", LaserScan, self.scan_recieved, queue_size=1) # measuring of gamma itself


    def set_measure1(self):
        #because of lambda
        self.should_measure = True
        self.measure2 = False

    def set_measure2(self):
        self.should_measure = True
        self.measure2 = True

    # cb for initial theta setting - closest to 0deg
    def listen_theta(self, scan_msg):
        angle_inc = scan_msg.angle_increment
        
        a = scan_msg.ranges[self.index_offset]
        b = scan_msg.ranges[-self.index_offset - 1] # -1, because it ends at PI, but starts at -PI+angle_inc
        print "a=%s; b=%s" % (a,b)

        angle_diff = angle_inc*self.index_offset
        alfa = angle_diff*2
        
        PI = math.pi # np.deg2rad(180)
        theta_multiplier = 1

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
        theta = math.acos(k/x) # angle between 'middle wheel' and the perpendicular

        if not math.isnan(theta):
            print "Theta=%s" % np.rad2deg(theta * theta_multiplier)


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

        angle_diff = angle_inc*self.index_offset # alfa from sketch
        alfa = angle_diff*2 # alfa*2 from sketch
        self.msrmt.alfa = alfa # both measurements have same alfa
        print "Alfa=%s" % alfa

        PI = math.pi

        if b > a: # to have uniform calculation, we just know that result gamma will be negative
            b,a=a,b
            self.msrmt.negative_gamma = True

        c = math.sqrt(a*a + b*b - 2*a*b*np.cos(alfa)) # 3rd side of triangle a,b,c - Law of cosine
        phi = math.asin((b*math.sin(alfa)) / c) # angle between a,c - Law of sine
        beta = PI - phi - alfa # angle between b,c
        omega = PI - phi - angle_diff # angle between x,c' in triangle a,x,c'
        
        sin_phi = math.sin(phi)
        x = (a*sin_phi) / math.sin(omega) # 'ray' from 'middle wheel'
        k = sin_phi*a #if a>b else math.sin(beta)*b # perpendicular from car to the table
        theta = math.acos(k/x) # angle between 'middle wheel' and the perpendicular

        # print "c=%s\nphi=%s\nbeta=%s\nomega=%s\nx=%s\nk=%s\ntheta=%s\n\n" % \
        #     (c,np.rad2deg(phi),np.rad2deg(beta),np.rad2deg(omega),x,k,np.rad2deg(theta))

        if self.measure2:
            # 2nd measuring
            self.msrmt.a2 = a
            self.msrmt.b2 = b
            self.msrmt.c2 = c
            self.msrmt.phi2 = phi
            self.msrmt.beta2 = beta
            self.msrmt.theta2 = theta
            self.msrmt.omega2 = omega
            self.msrmt.x2 = x
            self.msrmt.k2 = k

            print "c=%s\nphi=%s\nbeta=%s\nomega=%s\nx=%s\nk=%s\ntheta=%s\n\n" % \
                (c,np.rad2deg(phi),np.rad2deg(beta),np.rad2deg(omega),x,k,np.rad2deg(theta))
            
            # calculate result (gamma)
            self.msrmt.calculate_after_measuring()
        else:
            # 1st measuring
            THETA_TOLERANCE = 0.5
            theta_deg = np.rad2deg(theta)
            # theta closest to 0.0, so measurement is precise
            if theta_deg > THETA_TOLERANCE:
                return

            print "c=%s\nphi=%s\nbeta=%s\nomega=%s\nx=%s\nk=%s\ntheta=%s\n\n" % \
                (c,np.rad2deg(phi),np.rad2deg(beta),np.rad2deg(omega),x,k,theta_deg)

            self.msrmt.a1 = a
            self.msrmt.b1 = b
            self.msrmt.c1 = c
            self.msrmt.phi1 = phi
            self.msrmt.beta1 = beta
            self.msrmt.theta1 = theta
            self.msrmt.omega1 = omega
            self.msrmt.x1 = x
            self.msrmt.k1 = k

        self.measure2 = False
        self.should_measure = False

''' Manually measured distance between front and back wheels - L in sketch '''
WHEEL_DISTANCE = 0.26

class measurement(object):
    def __init__(self, angle_arg):
        self.angle_arg = angle_arg
        self.a1 = 0 # left beam
        self.b1 = 0 # right beam
        self.c1 = 0 # side of a1,b1,c1 triangle
        self.alfa = 0 # angle between a1,b1
        self.phi1 = 0 # angle between a1,c1
        self.beta1 = 0 # angle between b1,c1
        self.theta1 = 0 # angle between x1 and perpendicular (k1)
        self.omega1 = 0 # angle between x1 and c1 - inside "triangle" a1,c1',x1
        self.x1 = 0 # 1st line from 'middle wheel'
        self.k1 = 0 # perpendicular from car to table/wall
        self.a2 = 0
        self.b2 = 0
        self.c2 = 0
        self.phi2 = 0
        self.beta2 = 0
        self.theta2 = 0
        self.omega2 = 0
        self.x2 = 0 # 2nd line from 'middle wheel'
        self.k2 = 0 # perpendicular
        
        #results
        self.d = 0 # diff between k2,k1
        self.r = 0 # radius of steering
        self.gamma = 0 # result steering angle
        self.negative_gamma = False

    def calculate_after_measuring(self):
        self.d = self.k2 - self.k1

        if self.theta2 == 0:
            self.gamma = 0
        else:
            self.r = self.d / math.sin(self.theta2)
            self.gamma = math.asin(WHEEL_DISTANCE/self.r)
            if self.negative_gamma:
                self.gamma *= -1

        gamma_deg = np.rad2deg(self.gamma)

        # copied from console to angles_deg
        print "d=%s\nr=%s\nGAMMA=%s" % (self.d, self.r, gamma_deg)


''' Results of measuring - in degrees '''
angles_deg = [
    [0, None], # 5.78232772914 7.01673140476 8.37394859144 7.23885239021 23.6679004034
    [30, None],#1.01571180243 0.998853724323 3.4167661193 3.16325378947 16.4576664611
    [60, None], #7.02317928443 4.28223442844 3.07800787528 10.7319619901
    [90, 2.97257469303], #4.73999588815 0.143465331583 # 2.11019396 #2.76924590998
    [120, None], # 4.26428962395
    [150, None], #10.0330308795
    [179, None] #14.913139882
]

def print_table(angles_deg):
    print "\n|--Argument--|--Real angle--|"

    for p in angles_deg:
        print "|" + "{0}".format(p[0]).rjust(7).ljust(12) + "|" + "{0}".format(p[1]).rjust(8).ljust(14) + "|"
    
    print "|------------|--------------|"
    

def perform_test(spd_ctrl, str_ctrl, scan_rcv, msrmt, setup_only):
    spd_ctrl.stop() # sicher

    if setup_only: return # only listen to theta, don't drive

    scan_rcv.set_measure1()
    scan_rcv.msrmt = msrmt
    time.sleep(2) # wait for measurement
    
    str_ctrl.steer(msrmt.angle_arg)
    time.sleep(1) # wait for setting up steering
    spd_ctrl.start()

    # stop after a given duration
    rospy.Timer(rospy.Duration(spd_ctrl.drive_duration), lambda _: spd_ctrl.stop(), oneshot=True)
    
    # wait and measure 2nd time
    rospy.Timer(rospy.Duration(spd_ctrl.drive_duration+1.5), lambda _: scan_rcv.set_measure2(), oneshot=True)


def get_array_column(arr, idx):
    return [row[idx] for row in arr]    

def main(args):
    # print np.rad2deg(0.0518812156548); 
    
    # print "test interpol."
    # uhly = get_array_column(angles_deg,0) # [0,30,60,90,120,150,179]
    # tst = get_array_column(angles_deg,1)
    # tst = [-32, -21, -14, 2.97, 17, 24, 36]
    # print uhly
    # print np.interp(95, uhly, tst) # arg->real
    # print np.interp(0, tst, uhly) # real->arg
    # return

    rospy.init_node("calibrate_steering")

    # True==only listening to Theta,False=driving,measuring gamma
    SETUP_ONLY = False
    scan_rcv = scan_receiver(setup_before_measuring=SETUP_ONLY)
    speed_control = speed_controller()
    steer_control = steering_controller()

    m = measurement(60) # arg==steering data sent to the topic

    # comment before measuring+add False to receiver, measure Theta - should be close to 0
    perform_test(speed_control, steer_control, scan_rcv, m, SETUP_ONLY)

    #print_table(angles_deg)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
