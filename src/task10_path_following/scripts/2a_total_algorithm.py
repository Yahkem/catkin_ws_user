#!/usr/bin/env python

from __future__ import print_function

import json

import cv2
import numpy as np
from scipy import stats
from numpy import pi, sin, cos
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import Int16, String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from ColorBulb import ColorBulb




# SETUP
rospy.init_node('total_triangulation', anonymous=True)
position_pub_odom = rospy.Publisher("/global_position/odom", Odometry, queue_size=1)
# used for kalman filter proportions
position_pub_certainty_pos = rospy.Publisher("/global_position/certainty/pos", Float32, queue_size=1)
position_pub_certainty_yaw = rospy.Publisher("/global_position/certainty/yaw", Float32, queue_size=1)



def ros_callback(data):
    my_dict = json.loads(data.data)
    # position of car in the image
    pos_x = my_dict['width']/2
    pos_y = my_dict['height']/2
    bulbs = [ColorBulb(bulb) for bulb in my_dict['bulbs']]

    # yaw split in x and y to do "medians of angles"
    # count how many combinations are computed
    yaw_x = 0
    yaw_y = 0
    count = 0

    # position measurement
    best_error = float("inf")
    x = 0
    y = 0
    
    valid_bulbs = []

    for a in bulbs:
        if a.x_img==-1:
            continue
        for b, _ in valid_bulbs:
            # yaw
            alpha = -np.arctan2(b.x_img-a.x_img, a.y_img-b.y_img)
            alpha += np.arctan2(b.y_real-a.y_real, b.x_real-a.x_real)
            yaw_x += sin(alpha)
            yaw_y += cos(alpha)
            count += 1

        a_angle = np.arctan2(a.y_img-pos_y, a.x_img-pos_x)
        for idx, (b, b_angle) in enumerate(valid_bulbs, 1):
            for c, c_angle in valid_bulbs[idx:]:
            	# moified beacon coords
                x1d = (a.x_real-b.x_real)
                y1d = (a.y_real-b.y_real)
                x3d = (c.x_real-b.x_real)
                y3d = (c.y_real-b.y_real)
                # cot
                t12 = 1/np.tan(b_angle-a_angle)
                t23 = 1/np.tan(c_angle-b_angle)
                t31 = (1-t12*t23)/(t12+t23)
                #print("t", t12, t23, t31)
                # modified circle center coords
                x12d = x1d+t12*y1d
                y12d = y1d-t12*x1d
                x23d = x3d-t23*y3d
                y23d = y3d+t23*x3d
                x31d = (x3d+x1d)+t31*(y3d-y1d)
                y31d = (y3d+y1d)-t31*(x3d-x1d)
                # k31d
                k31d = x1d*x3d+y1d*y3d+t31*(x1d*y3d-x3d*y1d)
                # d (if 0, return error/lined up with 2 bulbs/error=infinity)
                d = (x12d-x23d)*(y23d-y31d)-(y12d-y23d)*(x23d-x31d)
                error = 1-np.abs(d)/(np.abs(d)+1000000)
                # robot position
                if error < best_error:
                	best_error = error
                	x = b.x_real+k31d*(y12d-y23d)/d
                	y = b.y_real+k31d*(x23d-x12d)/d
        valid_bulbs.append((a, a_angle))
    if len(valid_bulbs) < 3:  # only 2 or less valid points - result not usable
        print("Number of bulbs found:", "< 3")
        return

    # yaw
    ## mean of angles
    yaw = -np.arctan2(yaw_y/count, yaw_x/count)+np.pi/2
    if yaw > np.pi:
        yaw -= 2*np.pi
                
    # position
    x = x/100.0
    y = y/100.0
        
    #print("pos", y, x, yaw, d)

    # calc odom topic
    time = rospy.Time.now()
    odom_broadcaster = tf.TransformBroadcaster()
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        time,
        "base_link",
        "global_position"
    )
    odom = Odometry()
    odom.header.stamp = time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    #odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))


    # publish
    position_pub_certainty_pos.publish(1-best_error)
    position_pub_certainty_yaw.publish(0.7)  # TODO dynamic with std.-dev.
    position_pub_odom.publish(odom)




rospy.Subscriber("/bulb_coords", String, ros_callback, queue_size=1)
try:
    rospy.spin()
except KeyboardInterrupt:
    #f_debug.close()
    print("Shutting down")
cv2.destroyAllWindows()
