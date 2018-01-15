#!/usr/bin/env python

from __future__ import print_function
import sys
import os
import time
import tf
import json
import rospy
import cv2
import numpy as np
from numpy import pi, sin, cos
from scipy import stats
from std_msgs.msg import String, Float32, Int16
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError


# TODO fix the x,y shift between coordinate systems?

class LanePlanning(object):
    def __init__(self, path):
        rospy.loginfo("Initializing LanePlanning instance...")

        # vars
        if path is None:
            raise Exception("No path found!")
        self.file_path = path
        self.grid = np.load(self.file_path)
        self.grid_shape = self.grid.shape[0:2]

        # Odometry subscribers, publishers
        self.pub_planning_speed = rospy.Publisher("/planning/speed", Int16, queue_size=1)
        self.pub_planning_yaw = rospy.Publisher("/planning/yaw", Int16, queue_size=1)

        self.sub_position_filtered = rospy.Subscriber("/global_position/filtered", Odometry, self.update, queue_size=1)

        rospy.loginfo("LanePlanning instance initialized!")

    def update(self, odom):
        pose_odom = odom.pose.pose
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
        #print(pos_x, pos_y, pos_theta)
        x = max(0, min(int(pos_x * 10), self.grid_shape[0]))
        y = max(0, min(int(pos_y * 10), self.grid_shape[1]))
        desired_x = self.grid[x, y, 0]
        desired_y = self.grid[x, y, 1]

        desired_yaw = np.arctan2(desired_y, desired_x)
        desired_speed = np.hypot(desired_y, desired_x)
        planning_yaw = desired_yaw-pos_theta
        if planning_yaw>pi:
            planning_yaw = 2*pi-planning_yaw
        elif planning_yaw<-pi:
            planning_yaw = planning_yaw+2*pi

        # backwards check
        if planning_yaw > pi/2:
            planning_yaw = -pi
            planning_speed = 100
        elif planning_yaw < -pi/2:
            planning_yaw = pi
            planning_speed = 100
        else:
            planning_speed = -100-cos(abs(planning_yaw))*100
        print("pos", pos_x, pos_y, pos_theta)
        print("plan", desired_yaw, planning_yaw, planning_speed)

        # publish
        self.pub_planning_speed.publish(planning_speed)
        self.pub_planning_yaw.publish(planning_yaw)


def main(args):
    rospy.init_node('lane_planning', anonymous=True)

    # Create LanePlanning object and listen to Odometry messages
    planning = None
    if len(args) > 1 and args[1] == "2":
        planning = LanePlanning("files/matrixDynamic_lane2.npy")
    else:
        planning = LanePlanning("files/matrixDynamic_lane1.npy")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
