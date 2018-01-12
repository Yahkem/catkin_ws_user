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


class SteeringController(object):
    ''' Takes care of steering mapping '''
    def __init__(self):
        self.MAPPING_GAMMA = [-0.449, -0.327, -0.140, -0.057, -0.009, 0.062, 0.113, 0.175, 0.234, 0.348, 0.494]
        self.MAPPING_STEER = [179, 150, 120, 110, 100, 90, 80, 70, 60, 30, 0]
        self.pi_by_180 = math.pi/180.0
    
    def mapping(self, gamma_deg):
        gamma_rad = gamma_deg * self.pi_by_180
        index = next((i for i, v in enumerate(self.MAPPING_GAMMA) if v > gamma_rad), -1)

        if index == -1 or index == 0:
            return self.MAPPING_STEER[index]
        else:
            prev_idx = index-1
            steer_cur = self.MAPPING_STEER[index]
            gamma_cur = self.MAPPING_GAMMA[index]
            steer_prev = self.MAPPING_STEER[prev_idx]
            gamma_prev = self.MAPPING_GAMMA[prev_idx]

            return steer_prev + (steer_cur-steer_prev) * (gamma_rad-gamma_prev) / (gamma_cur-gamma_prev)


class CircleDriver(object):
    def __init__(self, steer_control):
        self.NPY_FILENAME_1 = 'matrixDynamic_lane1.npy'
        # self.NPY_FILENAME_2 = 'matrixDynamic_lane2.npy'

        self.steer_control = steer_control
        self.potential_field = np.load(self.NPY_FILENAME_1)
        
        self.kalman_recieved = False
        
        # Subcribers, publishers
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)
        self.pub_steer = rospy.Publisher("/manual_control/steering", Int16, queue_size=100)
        time.sleep(1) # for initializing publishers - sometimes there's delay
        self.sub_kalman = rospy.Subscriber("/global_position/filtered", Odometry, callback=self.kalman_listener, queue_size=1)

        self.test_1() # did we load the potential field?

    def test_1(self):
        print("\n--Test potential field loaded--")
        print(self.potential_field[20,10]) #[0.349999... -0.739000...2]
        print(self.potential_field[20,11, :]) # [0.34999... -0.838999...]
        print("--End Test1--")

    def kalman_listener(self, odom_msg):
        print("kalman_listener fired!")
        # TODO plot positions after initial sensory data
        # TODO int counter because 1st measurement might be crap?
        if self.kalman_recieved: # fire only once 
            return

        # (x,y) in [m]; yaw in [rad]
        (x_car, y_car, yaw_car) = self.process_odom_msg(odom_msg)
        print("x=%s; y=%s; yaw=%s;" % (x_car, y_car, yaw_car))

        # meters -> field coords 10x10cm
        x_idx = int(round(x_car * 10))
        y_idx = int(round(y_car * 10))

        # equations from the PDF
        (x_map, y_map) = self.potential_field[x_idx, y_idx, :]
        print("Force vec = (%s, %s)" % (x_map, y_map))
        x_force = math.cos(yaw_car)*x_map + math.sin(yaw_car)*y_map
        y_force = -math.sin(yaw_car)*x_map + math.cos(yaw_car)*y_map
        Kp = 4 # TODO maybe 1 or other value, because large values with 4?
        steering_rad = Kp * math.atan(y_force / (2.5*x_force)) # TODO 4 instead of 2.5?
        steering_deg = np.rad2deg(steering_rad)
        steering_arg = self.steer_control.mapping(steering_deg) # what we supply to the topic

        print("Angle %s=%s -> %s arg\n-------\n" % (steering_rad, steering_deg, steering_arg))

        backwards = False
        # TODO change backwards?
        if steering_deg > 90 or steering_deg < -90:
            # backwards = True
            print("!!!!!!!!!!!!!!!!!!!!")
            time.sleep(2)

        self.drive(steering_arg, backwards=backwards)
        '''
        1. get position from kalman
            recalculate pos.
        2. get force vector based on this position
        3. calculate angle
        4. drive [infinitely/for amount of time]
        TODO
            adjust steering forwards/backwards?
        dont forget to uncomment kalman_recieved!
        '''

        # self.kalman_recieved = True
    
    def drive(self, steering_arg, backwards=False):
        STOP_AFTER_SECS = 5
        SPEED_ARG = -100 # TODO change appropriately

        if backwards: SPEED_ARG *= -1

        self.pub_steer.publish(steering_arg)
        time.sleep(0.3)
        self.pub_speed.publish(SPEED_ARG)
        rospy.Timer(rospy.Duration(STOP_AFTER_SECS), lambda _: self.pub_speed.publish(0), oneshot=True)

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
    steer_control = SteeringController()
    circle_driver = CircleDriver(steer_control)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
