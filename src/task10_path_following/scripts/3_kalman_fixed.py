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
from scipy import stats
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError

# TODO fix the x,y shift between coordinate systems?

class KalmanFilter(object):
    
    def __init__(self):
        rospy.loginfo("Initializing KalmanFilter instance...")
        
        # predition of /odom in real-world-coordinates
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_time = None
        # previous values in car-coordinates
        self.prev_odom_x = 0.0
        self.prev_odom_y = 0.0
        self.prev_odom_theta = 0.0
        # predition of /global_position/odom in real-world-coordinates
        self.global_x = 0.0
        self.global_y = 0.0
        self.global_theta = 0.0
        self.global_time = None
        
        # self.k = 0.5 # const? i think so...
        # TODO different for x,y and one for theta
        self.k_xy = 0.05
        self.k_theta = 0.5

        # Odometry subscribers, publishers
        self.pub_filtered = rospy.Publisher("/global_position/filtered", Odometry, queue_size=1)
        ### rostopic echo /global_position/filtered
        self.sub_predict_odom = rospy.Subscriber("/odom", Odometry, self.predict, queue_size=1)
        self.sub_update_gps_odom = rospy.Subscriber("/global_position/odom", Odometry, self.update, queue_size=1)
        self.sub_update_certainity_pos = rospy.Subscriber("/global_position/certainty/pos", Float32, self.certainty_pos, queue_size=1)
        self.sub_update_certainity_yaw = rospy.Subscriber("/global_position/certainty/yaw", Float32, self.certainty_yaw, queue_size=1)
        
        rospy.loginfo("KalmanFilter instance initialized!")

    def certainty_pos(self, data):
        if data is not None:
            self.k_xy = data.data / 4.0

    def certainty_yaw(self, data):
        if data is not None:
            self.k_theta = data.data / 4.0

    def process_odom_msg(self, odom_msg, is_predicting=True):
        ''' Processes the odometry message and returns (position, orientation, pos_x, pos_y, pos_theta) '''
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
        #pos_theta = np.arccos(orientation_odom.w) * 2 * np.sign(orientation_odom.z) # acos(q.w)*2)*sign(q.z) <- wtf parentheses in PDF

        # rospy.loginfo("Position = %s" % position_odom)
        # rospy.loginfo("Orientation = %s" % orientation_odom)
        #pred_or_upd_str = "prediction" if is_predicting else "update"
        #rospy.loginfo("Before %s: X=%s; Y=%s; Theta=%srad" % (pred_or_upd_str, pos_x, pos_y, pos_theta))

        return (position_odom, orientation_odom, pos_x, pos_y, pos_theta)

    def predict(self, odom_msg):
        ''' prediction step -  from /odom topic '''

        _,_,x,y,theta = self.process_odom_msg(odom_msg, is_predicting=True)
	#print(x,y,theta)
	theta *= -1

        # if not initilized
        if self.odom_time is None:
            self.prev_odom_x = x
            self.prev_odom_y = y
            self.prev_odom_theta = theta
            self.odom_time = rospy.Time.now()
            return
        if self.global_time is None:
            return

        distance_travelled = self.pythagoras(self.prev_odom_x, self.prev_odom_y, x, y)

        # predict delta for /odom to real-world-coordinates
        x_delta = np.cos(self.odom_theta) * distance_travelled
        y_delta = np.sin(self.odom_theta) * distance_travelled
        theta_delta = self.prev_odom_theta - theta  # negated theta because internal theta is negated
        if theta_delta > np.pi:
            theta_delta -= 2*np.pi
        elif theta_delta < -np.pi:
            theta_delta += 2*np.pi

        # update predictions
        self.odom_x += x_delta
        self.odom_y += y_delta
        self.odom_theta += theta_delta
        if self.odom_theta > np.pi:
            self.odom_theta -= 2*np.pi
        elif self.odom_theta < -np.pi:
            self.odom_theta += 2*np.pi
        #print("print delta", x_delta, y_delta, theta_delta)

        # update previous "raw" values
        self.prev_odom_x = x
        self.prev_odom_y = y
        self.prev_odom_theta = theta

        # publish estimation after prediction
        self.publish(self.odom_x, self.odom_y, self.odom_theta, is_predicting=True)


    def update(self, odom_msg):
        ''' update step - from /global_position/odom topic (GPS) '''

        # measured GPS positions
        _,_,x,y,theta = self.process_odom_msg(odom_msg, is_predicting=False)

        # if not initilized
        if self.global_time is None:
            self.global_x = x
            self.global_y = y
            self.global_theta = theta
            self.publish(self.global_x, self.global_y, self.global_theta, is_predicting=False)
            self.global_time = rospy.Time.now()
            self.odom_x = self.global_x
            self.odom_y = self.global_y
            self.odom_theta = self.global_theta
            return
            
        # calculate updated values
        k_xy_compl = 1 - self.k_xy
        self.global_x = self.k_xy*x + k_xy_compl*self.odom_x
        self.global_y = self.k_xy*y + k_xy_compl*self.odom_y
        sin_mean = self.k_theta*np.sin(theta) + (1-self.k_theta)*np.sin(self.odom_theta)
        cos_mean = self.k_theta*np.cos(theta) + (1-self.k_theta)*np.cos(self.odom_theta)
        self.global_theta = np.arctan2(sin_mean, cos_mean)
        if self.global_theta > np.pi:
            self.global_theta -= 2*np.pi
        elif self.global_theta < -np.pi:
            self.global_theta += 2*np.pi
        # set new "start point" of estimation
        self.odom_x = self.global_x
        self.odom_y = self.global_y
        self.odom_theta = self.global_theta

        # calc filtered odom topic and publish
        self.publish(self.global_x, self.global_y, self.global_theta, is_predicting=False)

    def publish(self, x, y, theta, is_predicting=True):
        '''publish latest position'''
        pred_or_upd_str = "prediction" if is_predicting else "update"
        #if is_predicting:
        #    print("PUB", pred_or_upd_str, x, y, theta)
        #else:
        #    print("-------")
        time = rospy.Time.now()
        odom_broadcaster = tf.TransformBroadcaster()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta) # TODO is theta yaw? it looks like it...
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            time,
            "base_link",
            "filtered_position"
        )
        odom = Odometry()
        odom.header.stamp = time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        #odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        self.pub_filtered.publish(odom)
    
    def pythagoras(self, x0, y0, x1, y1):
        return np.sqrt((x1-x0)**2+(y1-y0)**2)


def main(args):
    rospy.init_node('kalman_filter_simple', anonymous=True)

    # Create KalmanFilter object and listen to Odometry messages
    kalman = KalmanFilter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
