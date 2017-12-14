#!/usr/bin/env python

from __future__ import print_function

import json

import cv2
import numpy as np
from numpy import pi, sin, cos
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import Int16, String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from ColorBulb import ColorBulb

# SETUP
MAT_WIDTH = 600  # corresponds to x in cm
MAT_HEIGHT = 400  # corresponds to y in cm
RESOLUTION = 10  # cm per measurement in grid
K = 20 *(10/RESOLUTION)**2  # use k nearest points
HISTORY_LENGTH = 5  # median of last positions



rospy.init_node('find_lines', anonymous=True)
position_pub_x = rospy.Publisher("/global_position/x", Int16, queue_size=1)
position_pub_y = rospy.Publisher("/global_position/y", Int16, queue_size=1)
position_pub_yaw = rospy.Publisher("/global_position/yaw", Float32, queue_size=1)
position_pub_combined = rospy.Publisher("/global_position/combined", String, queue_size=1)
position_pub_odom = rospy.Publisher("/global_position/odom", Odometry, queue_size=1)

angles = None
angles_origin = None
name_order = None
history_x = []
history_y = []


def pythagoras(x, y, x2, y2):
    return np.sqrt((x - x2) ** 2 + (y - y2) ** 2)


def calc_angles(bulbs):
    global angles, name_order, angles_origin
    angles = {}
    angles_origin = {}
    name_order = []

    dist = {}
    for bulb in bulbs:
        dist[bulb.name] = [[pythagoras((x+0.5)*RESOLUTION, (y+0.5)*RESOLUTION, bulb.x_real, bulb.y_real) for x in range(int(MAT_WIDTH//RESOLUTION))] for y in range(int(MAT_HEIGHT//RESOLUTION))]
        dist[bulb.name] = np.array(dist[bulb.name])
        angles_origin[bulb.name] = [[np.arctan2((y+0.5)*RESOLUTION-bulb.y_real, bulb.x_real-(x+0.5)*RESOLUTION) for x in range(int(MAT_WIDTH//RESOLUTION))] for y in range(int(MAT_HEIGHT//RESOLUTION))]
        angles_origin[bulb.name] = np.array(angles_origin[bulb.name])
        #print(angles_origin)
        name_order.append(bulb.name)
    for i in range(len(bulbs)-1):
        for j in range(i+1, len(bulbs)):
            a = bulbs[i]  # bulbs to calc distances
            b = bulbs[j]  # bulbs to calc distances
            key = a.name+";"+b.name
            dist_a_b = pythagoras(a.x_real, a.y_real, b.x_real, b.y_real)
            angles[key] = np.arccos((np.square(dist[a.name])+np.square(dist[b.name])-dist_a_b**2)/2/dist[a.name]/dist[b.name])
            #print(a, b, dist_a_b)
            #print(angles)


def get_x_y_offset(direction_vec):
    '''
    Returns x and y offset of coordinates based on normalized direction vector

    direction_vec - normalized direction vector - NumPy array - e.g. np.array([1,0])
    returns np.array([x,y]), where x and y are offsets of coordinates (these should be substracted from original X,Y)
    '''
    PI = np.pi
    PI_HALF = PI/2.0
    angles = [-PI, -PI_HALF, 0.0, PI_HALF, PI]
    # measured values from different positions
    x_offsets = [
        [-13, -12, 17, 20, -13],
        [-6, -20, 22, 8, -6]
    ]
    y_offsets = [
        [-18, -23, -12, 4, -18],
        [6, -24, -15, -2, 6]
    ]
    x = direction_vec[0]*100
    y = direction_vec[1]*100
    input_angle = np.arctan2(y, x) # RAD

    x_list = [np.interp(input_angle, angles, offset) for offset in x_offsets]
    y_list = [np.interp(input_angle, angles, offset) for offset in y_offsets]

    x_r = np.mean(x_list)
    y_r = np.mean(y_list)
    return np.array([x_r, y_r])


def ros_callback(data):
    global angles, name_order, history_x, history_y, angles_origin

    my_dict = json.loads(data.data)
    pos_x = my_dict['width']/2
    pos_y = my_dict['height']/2
    bulbs = [ColorBulb(bulb) for bulb in my_dict['bulbs']]
    if angles is None:
        calc_angles(bulbs)

    dist = {}
    diff = np.zeros((int(MAT_HEIGHT//RESOLUTION), int(MAT_WIDTH//RESOLUTION)))
    diff_count = 0
    for bulb in bulbs:
        dist[bulb.name] = pythagoras(pos_x, pos_y, bulb.x_img, bulb.y_img)

    for i in range(len(name_order)-1):
        for j in range(i+1, len(name_order)):
            a = (x for x in bulbs if x.name == name_order[i]).next()  # bulbs to calc distances
            b = (x for x in bulbs if x.name == name_order[j]).next()  # bulbs to calc distances
            if a.x_img==-1 or b.x_img==-1:
                continue
            key = a.name + ";" + b.name
            dist_a_b = pythagoras(a.x_img, a.y_img, b.x_img, b.y_img)
            angle = np.arccos((np.square(dist[a.name])+np.square(dist[b.name])-dist_a_b**2)/2.0/dist[a.name]/dist[b.name])
            diff = diff + np.abs(angles[key] - angle)
            diff_count += 1

    if diff_count < 2:  # only 2 or less valid points - result not usable
        print("Number of bulbs found:", "< 3")
        return
    flat_diff = diff.flatten()
    ind = np.argpartition(flat_diff, K)[:K]
    x, y = 0, 0
    points = np.array(np.where(diff <= max(diff.flatten()[ind])))
    points = (points+0.5)*RESOLUTION/100
    means = np.mean(points, axis=1)
    centered = points-np.tile(means, (points.shape[1], 1)).T
    stds = np.sqrt(np.sum(np.square(centered), axis=1)/points.shape[1])
    y, x = means
    std_y, std_x = stds
    #print("-----------------------")

    # mean of history - removed to follow advise of tutor
    #history_x.append(x)
    #history_x = history_x[-HISTORY_LENGTH:]
    #median_x = sorted(history_x)[len(history_x)//2]
    #history_y.append(y)
    #history_y = history_y[-HISTORY_LENGTH:]
    #median_y = sorted(history_y)[len(history_y)//2]
    median_x = x
    median_y = y

    # get rotation (yaw)
    yaw_x = 0
    yaw_y = 0
    yaw_count = 0
    for a in bulbs:
        if a.x_img==-1:
            continue
        alpha = np.arctan2(pos_y-a.y_img, a.x_img-pos_x) - angles_origin[a.name][int(median_y/RESOLUTION-0.5), int(median_x/RESOLUTION-0.5)]
        yaw_x += sin(alpha)
        yaw_y += cos(alpha)
        yaw_count += 1
    yaw = -np.arctan2(yaw_y/yaw_count, yaw_x/yaw_count)
    print("Number of bulbs found:", yaw_count)

    # correct x, y
    #corr_x, corr_y = get_x_y_offset((sin(yaw), cos(yaw)))
    #x -= corr_x
    #y -= corr_y

    # calc odom topic
    time = rospy.Time.now()
    odom_broadcaster = tf.TransformBroadcaster()
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    odom_broadcaster.sendTransform(
        (median_x, median_y, 0.),
        odom_quat,
        time,
        "base_link",
        "global_position"
    )
    odom = Odometry()
    odom.header.stamp = time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(median_x, median_y, 0.), Quaternion(*odom_quat))
    #odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))


    # publish
    #print(x, y, std_x, std_y, median_x, median_y)
    #print("yaw", yaw)
    position_pub_x.publish(x)
    position_pub_y.publish(y)
    position_pub_yaw.publish(yaw)
    position_pub_combined.publish(json.dumps({'x': x, 'y': y, 'std_x': x, 'std_y': std_y, 'median_x': median_x, 'median_y': median_y}))
    position_pub_odom.publish(odom)




rospy.Subscriber("/bulb_coords", String, ros_callback, queue_size=1)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
cv2.destroyAllWindows()
