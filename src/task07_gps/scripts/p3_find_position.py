from __future__ import print_function

import json

import cv2
import numpy as np
from numpy import pi, sin, cos
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import Int16, String
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
position_pub_combined = rospy.Publisher("/global_position/combined", String, queue_size=1)

angles = None
name_order = None
history_x = []
history_y = []


def pythagoras(x, y, x2, y2):
    return np.sqrt((x - x2) ** 2 + (y - y2) ** 2)


def calc_distances(bulbs):
    global angles, name_order
    angles = {}
    name_order = []

    dist = {}
    for bulb in bulbs:
        dist[bulb.name] = [[pythagoras((x+0.5)*RESOLUTION, (y+0.5)*RESOLUTION, bulb.x_real, bulb.y_real) for x in range(int(MAT_WIDTH//RESOLUTION))] for y in range(int(MAT_HEIGHT//RESOLUTION))]
        dist[bulb.name] = np.array(dist[bulb.name])
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


def ros_callback(data):
    global angles, name_order, history_x, history_y

    my_dict = json.loads(data.data)
    pos_x = my_dict['width']/2
    pos_y = my_dict['height']/2
    bulbs = [ColorBulb(bulb) for bulb in my_dict['bulbs']]
    if angles is None:
        calc_distances(bulbs)

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

    if diff_count < 2:  # only 2 valid points - result not usable
        return
    flat_diff = diff.flatten()
    ind = np.argpartition(flat_diff, K)[:K]
    x, y = 0, 0
    points = np.array(np.where(diff <= max(diff.flatten()[ind])))
    points = (points+0.5)*RESOLUTION
    means = np.mean(points, axis=1)
    centered = points-np.tile(means, (points.shape[1], 1)).T
    stds = np.sqrt(np.sum(np.square(centered), axis=1)/points.shape[1])
    y, x = means
    std_y, std_x = stds
    #print("-----------------------")

    # mean of history
    history_x.append(x)
    history_x = history_x[-HISTORY_LENGTH:]
    median_x = sorted(history_x)[len(history_x)//2]
    history_y.append(y)
    history_y = history_y[-HISTORY_LENGTH:]
    median_y = sorted(history_y)[len(history_y)//2]

    # publish
    print(x, y, std_x, std_y, median_x, median_y)
    position_pub_x.publish(x)
    position_pub_y.publish(y)
    position_pub_combined.publish(json.dumps({'x': x, 'y': y, 'std_x': x, 'std_y': std_y, 'median_x': median_x, 'mean_y': median_y}))



rospy.Subscriber("/bulb_coords", String, ros_callback, queue_size=1)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
cv2.destroyAllWindows()
