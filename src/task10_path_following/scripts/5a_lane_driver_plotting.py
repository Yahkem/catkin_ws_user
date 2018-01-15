#!/usr/bin/env python

from __future__ import print_function
import sys
import os
import time
import tf
import json
import rospy
import cv2
import math
import numpy as np
from scipy import stats
from std_msgs.msg import String, Float32, Int16
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion

start_pos_log = open("start_position.txt", "a")
pos_log = open("position.txt", "a")

# General Settings
GRID_RESOLUTION = 10
HYSTERESIS_TOP_CAP = 0
HYSTERESIS_LOW_CAP = 0


class CircleDriver():
    def __init__(self):
        self.NPY_FILENAME_1 = 'matrixDynamic_lane1.npy'
        # self.NPY_FILENAME_2 = 'matrixDynamic_lane2.npy'

        self.potential_field = np.load(self.NPY_FILENAME_1)
        self.map_size_x=600 #cm
        self.map_size_y=400 #cm
        self.resolution = GRID_RESOLUTION

        # Subcribers, publishers
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16,
                                         queue_size=100)
        self.pub_steer = rospy.Publisher("/manual_control/steering", Int16,
                                         queue_size=100)
        time.sleep(1)  # for initializing publishers - sometimes there's delay
        self.sub_odom = rospy.Subscriber("/assignment6/odom",
                                           Odometry,
                                           callback=self.odom_listener,
                                           queue_size=1)

        self.test_1()  # did we load the potential field?
        self.initialized = False

    def test_1(self):
        print("\n--Test potential field loaded--")
        print(self.potential_field[20, 10])  # [0.349999... -0.739000...2]
        print(self.potential_field[20, 11, :])  # [0.34999... -0.838999...]
        print("--End Test1--")

    def odom_listener(self, odom_msg):
        global pos_log, start_pos_log
        # this is modified code of Sarah
        x_car = odom_msg.pose.pose.position.x
        y_car = odom_msg.pose.pose.position.y
        oritentation_q = odom_msg.pose.pose.orientation
        oritentation_list = [oritentation_q.x, oritentation_q.y, oritentation_q.z, oritentation_q.w]
        _, _, yaw_car = euler_from_quaternion(oritentation_list)
        pos_log.write("{},{}\n".format(x_car, y_car))
        if not self.initialized:
            start_pos_log.write("{},{},{}\n".format(x_car, y_car, yaw_car))
        x_index = np.int(x_car*self.resolution)
        y_index = np.int(y_car*self.resolution)

        if (x_index < 0):
            x_index = 0
        if (x_index > ((self.map_size_x/self.resolution) - 1)):
            x_index = (self.map_size_x/self.resolution) - 1

        if (y_index < 0):
            y_index = 0
        if (y_index > ((self.map_size_y/self.resolution) - 1)):
            y_index = (self.map_size_y/self.resolution) - 1

        x3, y3 = self.potential_field[x_index, y_index, :]
        print(x3, y3)
        f_x = np.cos(yaw_car)*x3 + np.sin(yaw_car)*y3
        f_y = -np.sin(yaw_car)*x3 + np.cos(yaw_car)*y3
        Kp = 500.0
        steering = Kp*np.arctan(f_y/(2.5*f_x))

        if not self.initialized:
            if (f_x > HYSTERESIS_TOP_CAP):
                print("driving forward: angle: " + str(steering))
                print("fx: ", f_x)
                print("fy: ", f_y)
                speed = -150
            elif (f_x < HYSTERESIS_LOW_CAP):
                print("driving backwards: angle: " + str(steering))
                print("fx: ", f_x)
                print("fy: ", f_y)
                speed = 150
                if (f_y > 0):
                    steering = -np.pi/2
                if (f_y < 0):
                    steering = np.pi/2

            if (steering > (np.pi)/2):
                steering = (np.pi)/2

            if (steering < -(np.pi)/2):
                steering = -(np.pi)/2
            steering = 90 + steering * (180/np.pi)
            # this line inverts the steering in case the current car has inverse
            # setup: 0 = left , 180 = right
            steering = 180 - steering

            self.pub_speed.publish(Int16(speed))
            self.pub_steer.publish(Int16(steering))
            self.initialized = True
        print("marker")

    def process_odom_msg(self, odom_msg):
        ''' Processes the odometry message and returns tuple (pos_x, pos_y, pos_theta) '''
        pose_odom = odom_msg.pose.pose
        position_odom = pose_odom.position
        orientation_odom = pose_odom.orientation

        pos_x = position_odom.x
        pos_y = position_odom.y
        _, _, pos_theta = tf.transformations.euler_from_quaternion([
            orientation_odom.x,
            orientation_odom.y,
            orientation_odom.z,
            orientation_odom.w
        ])

        return (pos_x, pos_y, pos_theta)


def main(args):
    rospy.init_node('steering_circles', anonymous=True)

    # asd
    # kalman = KalmanFilter()
    circle_driver = CircleDriver()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
