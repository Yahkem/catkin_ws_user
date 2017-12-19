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
RESOLUTION = 5  # cm per measurement in grid
K = 1 *(10/RESOLUTION)**2  # use k nearest points



rospy.init_node('find_lines', anonymous=True)
position_pub_x = rospy.Publisher("/global_position/x", Int16, queue_size=1)
position_pub_y = rospy.Publisher("/global_position/y", Int16, queue_size=1)
position_pub_yaw = rospy.Publisher("/global_position/yaw", Float32, queue_size=1)
position_pub_combined = rospy.Publisher("/global_position/combined", String, queue_size=1)
position_pub_odom = rospy.Publisher("/global_position/odom", Odometry, queue_size=1)

angles = None
angles_origin = None
name_order = None

def pythagoras(x, y, x2, y2):
    return np.sqrt((x - x2) ** 2 + (y - y2) ** 2)


def calc_angles(bulbs):
    ''' Will be executed once to pre-calculate static values '''
    global angles, name_order, angles_origin
    angles = {}
    angles_origin = {}
    name_order = []

    dist = {} # distance from each point in the grid (np.array) to each bulb
    for bulb in bulbs:
        dist[bulb.name] = [[pythagoras((x+0.5)*RESOLUTION, (y+0.5)*RESOLUTION, bulb.x_real, bulb.y_real) for x in range(int(MAT_WIDTH//RESOLUTION))] for y in range(int(MAT_HEIGHT//RESOLUTION))]
        dist[bulb.name] = np.array(dist[bulb.name])
        
        # array to use the bulb every time in the same order
        name_order.append(bulb.name)
    for i in range(len(bulbs)-1):
        for j in range(i+1, len(bulbs)):
            a = bulbs[i]  # bulbs to calc distances
            b = bulbs[j]  # bulbs to calc distances
            key = a.name+";"+b.name
            # distance between two bulbs
            dist_a_b = pythagoras(a.x_real, a.y_real, b.x_real, b.y_real)
            # angle for each point in the grid to each pair of lamps (component-wise operations)
            angles[key] = np.arccos((np.square(dist[a.name])+np.square(dist[b.name])-dist_a_b**2)/2/dist[a.name]/dist[b.name])
            # angle for each bulb to origin
            angles_origin[key] = np.arctan2(b.y_real-a.y_real, b.x_real-a.x_real)


def ros_callback(data):
    global angles, name_order, angles_origin

    my_dict = json.loads(data.data)
    # position of car in the image
    pos_x = my_dict['width']/2
    pos_y = my_dict['height']/2
    bulbs = [ColorBulb(bulb) for bulb in my_dict['bulbs']]
    if angles is None:
        # first loop only - precalculate static variables
        calc_angles(bulbs)


    # distance for each bulb to car pos. in image (image center)
    dist = {}
    # sum of angle differences for each cell in the grid
    diff = np.zeros((int(MAT_HEIGHT//RESOLUTION), int(MAT_WIDTH//RESOLUTION)))
    # count how many combinations are computed
    count = 0
    # yaw split in x and y to do "medians of angles"
    yaw_x = 0
    yaw_y = 0
    for bulb in bulbs:
        dist[bulb.name] = pythagoras(pos_x, pos_y, bulb.x_img, bulb.y_img)

    for i in range(len(name_order)-1):
        for j in range(i+1, len(name_order)):
            a = (x for x in bulbs if x.name == name_order[i]).next()  # bulbs to calc distances
            b = (x for x in bulbs if x.name == name_order[j]).next()  # bulbs to calc distances
            if a.x_img==-1 or b.x_img==-1:
                continue
            key = a.name + ";" + b.name
            count += 1

            # position
            dist_a_b = pythagoras(a.x_img, a.y_img, b.x_img, b.y_img)
            ## angle from car to each bulb pair
            angle = np.arccos((np.square(dist[a.name])+np.square(dist[b.name])-dist_a_b**2)/2.0/dist[a.name]/dist[b.name])
            diff = diff + np.abs(angles[key] - angle)

            # yaw
            alpha = -np.arctan2(b.x_img-a.x_img, a.y_img-b.y_img)
            alpha += angles_origin[key]
            yaw_x += sin(alpha)
            yaw_y += cos(alpha)

    if count < 2:  # only 2 or less valid points - result not usable
        print("Number of bulbs found:", "< 3")
        return

    # position
    ## get postion from car where angle difference is the lowest (mean of K=1 points)
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

    # yaw
    ## mean of angles
    yaw = -np.arctan2(yaw_y/count, yaw_x/count)+np.pi/2
    if yaw > np.pi:
        yaw -= 2*np.pi

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
    position_pub_x.publish(x)
    position_pub_y.publish(y)
    position_pub_yaw.publish(yaw)
    position_pub_combined.publish(json.dumps({'x': x, 'y': y, 'yaw': yaw, 'std_x': x, 'std_y': std_y}))
    position_pub_odom.publish(odom)




rospy.Subscriber("/bulb_coords", String, ros_callback, queue_size=1)
try:
    rospy.spin()
except KeyboardInterrupt:
    #f_debug.close()
    print("Shutting down")
cv2.destroyAllWindows()
