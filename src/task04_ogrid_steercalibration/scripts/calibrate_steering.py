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
        self.drive_duration = 3

    def start(self):
        rospy.loginfo("starting...")
        self.pub_speed.publish(30) #TODO prijit na spravnou rychlost
    
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
        self.index_offset = 12 #offset of scan_msg.ranges[] list - angles used for measuring
        should_measure = False
        rospy.Subscriber("scan", LaserScan, self.scan_recieved, queue_size=10)

    def set_should_measure(self, should_measure):
        #because of lambda
        self.should_measure = should_measure

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
        b = scan_msg.ranges[-self.index_offset] # -1?
        print "a=%s; b=%s" % (a, b)

        angle_diff = angle_inc*self.index_offset # alfa1

        angle1 = angle_min+angle_diff
        angle2 = angle_max-angle_diff
        print "A1=%s;A2=%s;DIFF=%s" % (np.rad2deg(angle1), np.rad2deg(angle2), np.rad2deg(angle_diff))
        print "PROSTREDEK=%s;%s;%s\n" % (scan_msg.ranges[-1],scan_msg.ranges[0], scan_msg.ranges[1])

        #vzorce
        alfa = angle_diff*2
        c = math.sqrt(a*a + b*b - 2*a*b*np.cos(alfa))
        phi = math.asin((b*math.sin(alfa)) / c)
        beta = np.deg2rad(180) - phi - angle_diff

        sin_phi = math.sin(phi)
        x = (a*sin_phi) / math.sin(beta)
        k = sin_phi * a
        theta1 = math.acos(k/x)

        print("c=%s\nphi=%s\nbeta=%s\nx=%s\nk=%s\ntheta1=%s\n\n") % (c,np.rad2deg(phi),np.rad2deg(beta),x,k,np.rad2deg(theta1))
        # TODO save results somewhere -> to angles_deg

        self.should_measure = False
    # def calculate_opposite_side(self, a, b, alfa):
    #     # cosine law
    #     return math.sqrt(a*a + b*b - 2*a*b*np.cos(alfa))

    # def calculate_phi(self, b, c, alfa):
    #     return math.asin((b*math.sin(alfa)) / c)

angles_deg = {
    0: None,
    30: None,
    60: None,
    90: None,
    120: None,
    150: None,
    179: None
}

def perform_test(spd_ctrl, str_ctrl, scan_rcv, steer_arg):
    scan_rcv.should_measure = True
    spd_ctrl.stop() # sicher
    # TODO measure1
    time.sleep(1) # wait for measurement
    str_ctrl.steer(steer_arg)
    spd_ctrl.start()
    rospy.Timer(rospy.Duration(spd_ctrl.drive_duration), lambda _: spd_ctrl.stop(), oneshot=True)
    
    # measures after this
    rospy.Timer(rospy.Duration(spd_ctrl.drive_duration+0.1), lambda _: scan_rcv.set_should_measure(True), oneshot=True)
    

def main(args):
    rospy.init_node("calibrate_steering")

    speed_control = speed_controller()
    steer_control = steering_controller()
    scan_rcv = scan_receiver()

    # measure k
    #1.steer
    #2.start
    #3.after x sec (2?) stop 
    # measure k, theta, Gamma!
    perform_test(speed_control, steer_control, scan_rcv, 70)

    # print angles_deg[30]

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
