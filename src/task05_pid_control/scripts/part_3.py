#!/usr/bin/env python

import sys
import time
import math
import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan

# trigger by LaserScan
# no yaw or odometry

# scanning to the side

# take into account L

#0.5 for lookahead point

LOOKAHEAD_DISTANCE = 0.5 # m

''' Wanted distance from the wall '''
P = 0.4 # m

''' 's' in sketch, L from previous assignment '''
WHEEL_DISTANCE = 0.25

# HOW TO DO ENUM
# def enum(**enums):
#     return type('Enum', (), enums)

# SUBSCRIBE_TYPES = enum(LASER=0, THETA=1, DRIVE=2)


class ScanReciever(object):
    
    def __init__(self, is_only_listening):
        # self.index_offset = 12 #offset of scan_msg.ranges[] list - angles used for measuring
        self.wanted_alfa = 60 # deg
        self.wanted_bside_angle = 240 # this+60 -> a-side angle
        self.alfa = 0.0 # we'll get it later

        if is_only_listening:
            # measuring of theta & everything...
            rospy.Subscriber("scan", LaserScan, self.listen_laser, queue_size=1)
        else:
            # actual drive
            rospy.Subscriber("scan", LaserScan, self.listen_theta, queue_size=1) #closest to 0
            pass

    def listen_laser(self, scan_msg):
        rospy.loginfo("LEN=%s\n%s\n--------" % (len(scan_msg.ranges), scan_msg))

        len_ranges = len(scan_msg.ranges)
        idx_ratio = len_ranges/360.0

        flt_first_idx = idx_ratio * self.wanted_bside_angle
        b_idx = int(round(flt_first_idx))
        a_idx = int(round(flt_first_idx + self.wanted_alfa*idx_ratio))

        rospy.loginfo(">>>idx_ratio=%s; b_idx=%s; a_idx=%s" % (idx_ratio, b_idx, a_idx))
        
        real_angle_to_b = b_idx * scan_msg.angle_increment
        real_angle_to_a = a_idx * scan_msg.angle_increment

        self.alfa = real_angle_to_a - real_angle_to_b

        ratb_deg = np.rad2deg(real_angle_to_b)
        rata_deg = np.rad2deg(real_angle_to_a)

        rospy.loginfo('>>>ALFA(deg)=%s; Angle to b=%s; Angle to a=%s' % (np.rad2deg(self.alfa), ratb_deg, rata_deg))

        a = scan_msg.ranges[a_idx]
        b = scan_msg.ranges[b_idx]

        d,theta,theta_multiplier = self.get_d_and_theta(a,b)


        rospy.loginfo('>>> a=%s; b=%s; d=%s; THETA=%s; THETA(deg)=%s' % (a,b,d,theta, np.rad2deg(theta)))

        c_y = d + math.sin(theta) * WHEEL_DISTANCE
        theta_star = math.atan((P-c_y)/LOOKAHEAD_DISTANCE) * theta_multiplier

        rospy.loginfo('>>> c_y=%s; THETA*=%s; THETA*(deg)=%s' % (c_y, theta_star, np.rad2deg(theta_star)))


    def drive_beside_wall(self, scan_msg):
        pass

    def get_d_and_theta(self, a, b):
        alfa = self.alfa
        PI = math.pi # np.deg2rad(180)
        theta_multiplier = 1 # TODO maybe reverse? -> prob. not

        if b > a:
            b,a=a,b
            theta_multiplier = -1

        c = math.sqrt(a*a + b*b - 2*a*b*np.cos(alfa)) # 3rd side of triangle a,b,c - Law of cosine
        phi = math.asin((b*math.sin(alfa)) / c) # angle between a,c - Law of sine
        beta = PI - phi - alfa # angle between b,c
        omega = PI - phi - alfa/2 # angle between x,c' in triangle a,x,c'
        
        sin_phi = math.sin(phi)
        x = (a*sin_phi) / math.sin(omega) # 'ray' from the 'side'
        d = sin_phi*a #if a>b else math.sin(beta)*b # perpendicular from car to the table
        theta = math.acos(d/x) * theta_multiplier # angle between 'middle wheel' and the perpendicular

        return (d, theta, theta_multiplier)


def steering_angle_mapping(degrees):
    topic_args = [0, 30, 60, 90, 120, 150, 179]
    measured = [-23.4823835367, -16.2088298025, -4.10722402982,
        6.64525217941, 16.4196652944, 27.7491348616, 30.8972609386]
    
    deg_min = measured[0]
    deg_max = measured[-1]

    # culling high/low degrees
    if degrees < deg_min:
        degrees = deg_min
    elif degrees > deg_max:
        degrees = deg_max
    
    deg_interpolated = np.interp(degrees, measured, topic_args)

    return int(round(deg_interpolated))


def main(args):
    rospy.init_node('closing_the_loop')

    is_only_listening = True
    scan_receiver = ScanReciever(is_only_listening)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
