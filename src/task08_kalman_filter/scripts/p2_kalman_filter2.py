#!/usr/bin/env python

import sys
import os
import time
import tf
import json
import rospy
import cv2
import numpy as np
from scipy import stats
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError

class KalmanFilter(object):
    
    def __init__(self):
        rospy.loginfo("Initializing KalmanFilter instance...")
        
        self.x_prior = 0.0 # predicted x
        self.x_posterior = 0.0 # x from update t-1
        self.x_odom_prev = 0.0 # previous unprocessed x from /odom
        # self.x_delta = 0.0
        self.y_prior = 0.0
        self.y_posterior = 0.0
        self.y_odom_prev = 0.0
        # self.y_delta = 0.0
        self.theta_prior = 0.0
        self.theta_posterior = 0.0
        self.theta_odom_prev = 0.0
        # self.theta_delta = 0.0

        # 2) assume that values of vectors are in meters for x,y, radian for Theta
        # and the values of the matrices are in m^2 and rad^2
        # TODO 'make some assumptions' about variance and mean
        # "one could assume that the process noise (stddev) for component y and for delta time of 0.01 is 0.0001m, so in that case, Q would be (0.0001m)^2"
        self.Q = np.identity(3) # process noise (covariance) matrix 
        # TODO "make assumptions for R (x,y,Theta) as well"
        self.R = np.identity(3) # observation noise (cov.) matrix
        self.H = np.identity(3) # sensory data transform matrix "Assume that H is the identity matrix"
        self.P_apriori = np.identity(3) # TODO "in prediction step P_apriori = P_old + Q"
        
        # TODO "initial value for P-matrix for x and y could be e.g. (3m)^2, theta (pi/2)^2" <- ???
        self.P = np.array([
            [9., 0., 0.],
            [0., 9., 0.],
            [0., 0., (np.pi/2.)**2]])

        #self.k = 0.5 # const? i think so...
        # TODO matrix or 3 different gains?
        self.K = np.identity(3) # Kalman gain - TODO 
        self.k_x = 0.5
        self.k_y = 0.5
        self.k_theta = 0.5

        # Odometry subscribers, publishers
        self.sub_predict_odom = rospy.Subscriber("/odom", Odometry, self.predict, queue_size=1)
        self.sub_update_gps_odom = rospy.Subscriber("/global_position/odom", Odometry, self.update, queue_size=1)
        # rostopic echo /global_position/filtered
        self.pub_filtered = rospy.Publisher("/global_position/filtered", Odometry, queue_size=1)

        # for delta t - do we actually need that? v_x*delta t in pdf is basically just sqrt(dx^2 + dy^2)
        # and what the hell is 'v'?
        # self.time_init = time.time()
        # self.time_prev = self.time_init
        # self.time_curr = self.time_init
        
        rospy.loginfo("KalmanFilter instance initialized!")

    def process_odom_msg(self, odom_msg, is_predicting=True):
        ''' Processes the odometry message and returns (position, orientation, pos_x, pos_y, pos_theta) '''
        pose_odom = odom_msg.pose.pose
        position_odom = pose_odom.position
        orientation_odom = pose_odom.orientation

        pos_x = position_odom.x
        pos_y = position_odom.y
        # _, _, pos_yaw = tf.transformations.euler_from_quaternion([
        #     orientation_odom.x,
        #     orientation_odom.y,
        #     orientation_odom.z,
        #     orientation_odom.w
        # ])
        pos_theta = np.arccos(orientation_odom.w) * 2 * np.sign(orientation_odom.z) # acos(q.w)*2)*sign(q.z) <- wtf parentheses in PDF

        # rospy.loginfo("Position = %s" % position_odom)
        # rospy.loginfo("Orientation = %s" % orientation_odom)
        pred_or_upd_str = "prediction" if is_predicting else "update"
        rospy.loginfo("Before %s: X=%s; Y=%s; Theta=%srad" % (pred_or_upd_str, pos_x, pos_y, pos_theta))

        return (position_odom, orientation_odom, pos_x, pos_y, pos_theta)

    def predict(self, odom_msg):
        ''' prediction step -  from /odom topic '''
        
        _, _, x,y,theta = self.process_odom_msg(odom_msg)

        # TODO initial just set values?

        # TODO do we need (delta) time?
        distance_travelled = self.pythagoras(self.x_odom_prev, self.y_odom_prev, x, y)

        # calc from PDF
        x_delta = np.cos(theta) * distance_travelled
        y_delta = np.sin(theta) * distance_travelled
        theta_delta = theta - self.theta_odom_prev

        self.x_prior = self.x_posterior + x_delta
        self.y_prior = self.y_posterior + y_delta
        self.theta_prior = self.theta_posterior + theta_delta
        rospy.loginfo("PRIOR[x=%s; y=%s; Th=%s]" % (self.x_prior, self.y_prior, self.theta_prior))

        # update previous "raw" values
        self.x_odom_prev = x
        self.y_odom_prev = y
        self.theta_odom_prev = theta

        # TODO publish after prediction?


    def update(self, odom_msg):
        ''' update step - from /global_position/odom topic (GPS) '''
        # measured GPS positions
        _,_,x,y,theta = self.process_odom_msg(odom_msg)

        # calculate updated values
        k_xy_compl = 1 - self.k_xy
        self.x_posterior = self.k_xy*x + k_xy_compl*self.x_prior
        self.y_posterior = self.k_xy*y + k_xy_compl*self.y_prior
        self.theta_posterior = self.k_theta*theta + (1-self.k_theta)*self.theta_prior

        # calc filtered odom topic and publish
        time = rospy.Time.now()
        odom_broadcaster = tf.TransformBroadcaster()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta_posterior) # TODO is theta yaw? it looks like it...
        odom_broadcaster.sendTransform(
            (self.x_posterior, self.y_posterior, 0.),
            odom_quat,
            time,
            "base_link",
            "global_position"
        )
        odom_filtered = Odometry()
        odom_filtered.header.stamp = time
        odom_filtered.header.frame_id = "filtered"
        odom_filtered.pose.pose = Pose(Point(self.x_posterior, self.y_prior, 0.), Quaternion(*odom_quat))
        odom_filtered.child_frame_id = "base_link"
        odom_filtered.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        self.pub_filtered.publish(odom_filtered)

    
    def pythagoras(self, x0, y0, x1, y1):
        dx = x1-x0; dy=y1-y0

        # TODO abs?
        return np.sqrt(dx*dx + dy*dy)

    # def get_velocity(self):
    #     # TODO from last 2 odom positions and delta t??


def main(args):
    rospy.init_node('kalman_filter2', anonymous=True)

    # Create KalmanFilter object and listen to Odometry messages
    kalman = KalmanFilter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)