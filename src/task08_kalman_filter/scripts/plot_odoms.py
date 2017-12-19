#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
import time
import tf
import matplotlib.pyplot as plt
from scipy import stats
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

# rospy.init_node('asdfasdfdasfa_asd', anonymous=True)
# rospy.spin()

TIME_INIT = time.time()

xs_odom = []
ys_odom = []
thetas_odom = []
xs_gps = []
ys_gps = []
thetas_gps = []
xs_filtered = []
ys_filtered = []
thetas_filtered = []
times_odom = []
times_gps = []
times_filtered = []

def process_odom_msg(odom_msg):
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

def odom_cb(odom_msg):
    _, _, x, y, theta = process_odom_msg(odom_msg)

    # rospy.loginfo("%s; %s; %s" % (x,y,theta))

    xs_odom.append(x)
    ys_odom.append(-y) # minus, because y from odom is reverted
    thetas_odom.append(-theta)
    times_odom.append(time.time() - TIME_INIT)

def gps_cb(odom_msg):
    _, _, x, y, theta = process_odom_msg(odom_msg)

    xs_gps.append(x)
    ys_gps.append(y)
    thetas_gps.append(theta)
    times_gps.append(time.time() - TIME_INIT)

def filtered_cb(odom_msg):
    _, _, x, y, theta = process_odom_msg(odom_msg)

    xs_filtered.append(x)
    ys_filtered.append(y)
    thetas_filtered.append(theta)
    times_filtered.append(time.time() - TIME_INIT)

def plot_odom(abc_arg):
    print "!!!!!"
    time_label = 'Time [s]'

    plt.figure(1)
    plt.subplot(311)
    plt.title("/odom topic - X coordinate")
    plt.xlabel(time_label, labelpad=20)
    plt.ylabel('X coord')
    plt.grid()
    # plt.show()
    plt.plot(times_odom, xs_odom, 'og')

    # plt.figure(2)
    plt.subplot(312)
    plt.title('/odom topic - Y coordinate')
    plt.xlabel(time_label, labelpad=20)
    plt.ylabel('Y coord')
    plt.grid()
    # plt.show()
    plt.plot(times_odom, ys_odom, 'oy')

    # plt.figure(3)
    plt.subplot(313)
    plt.title('/odom topic - Theta')
    plt.xlabel(time_label, labelpad=20)
    plt.ylabel('Theta')
    plt.grid()
    plt.plot(times_odom, thetas_odom, 'or')
    plt.show()

def plot_gps(abc_arg):
    print "gps!!!!!"
    time_label = 'Time [s]'

    plt.figure(1)
    plt.subplot(311)
    plt.title("/global_position/odom topic (GPS) - X coordinate")
    plt.xlabel(time_label, labelpad=20)
    plt.ylabel('X coord')
    plt.grid()
    # plt.show()
    plt.plot(times_gps, xs_gps, 'og')

    # plt.figure(2)
    plt.subplot(312)
    plt.title('/global_position/odom topic (GPS) - Y coordinate')
    plt.xlabel(time_label, labelpad=20)
    plt.ylabel('Y coord')
    plt.grid()
    # plt.show()
    plt.plot(times_gps, ys_gps, 'oy')

    # plt.figure(3)
    plt.subplot(313)
    plt.title('/global_position/odom topic (GPS) - Theta')
    plt.xlabel(time_label, labelpad=20)
    plt.ylabel('Theta')
    plt.grid()
    plt.plot(times_gps, thetas_gps, 'or')
    plt.show()

def plot_filtered(abc_arg):
    print "filtered!!!!!"
    time_label = 'Time [s]'

    plt.figure(1)
    plt.subplot(311)
    plt.title("/global_position/filtered topic (Kalman) - X coordinate")
    plt.xlabel(time_label, labelpad=20)
    plt.ylabel('X coord')
    plt.grid()
    # plt.show()
    plt.plot(times_filtered, xs_filtered, 'og')

    # plt.figure(2)
    plt.subplot(312)
    plt.title('/global_position/filtered topic (Kalman) - Y coordinate')
    plt.xlabel(time_label, labelpad=20)
    plt.ylabel('Y coord')
    plt.grid()
    # plt.show()
    plt.plot(times_filtered, ys_filtered, 'oy')

    # plt.figure(3)
    plt.subplot(313)
    plt.title('/global_position/filtered topic (Kalman) - Theta')
    plt.xlabel(time_label, labelpad=20)
    plt.ylabel('Theta')
    plt.grid()
    plt.plot(times_filtered, thetas_filtered, 'or')
    plt.show()

def main(args):
    rospy.init_node('plotplot_plot', anonymous=True)

    # Create KalmanFilter object and listen to Odometry messages
    # kalman = KalmanFilter()
    # rospy.Timer(rospy.Duration(23), plot_odom, oneshot=True)
    # rospy.Timer(rospy.Duration(23), plot_gps, oneshot=True)
    rospy.Timer(rospy.Duration(23), plot_filtered, oneshot=True)


    sub_odom = rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=1)
    sub_gps = rospy.Subscriber("/global_position/odom", Odometry, gps_cb, queue_size=1)
    sub_filtered = rospy.Subscriber("/global_position/filtered", Odometry, filtered_cb, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

# plt.xlabel(labels[1])
# plt.ylabel(labels[2])
# plt.plot(data[1], data[2], "oy",)
# plt.show()
# plt.xlabel(labels[4])
# plt.ylabel(labels[5])
# plt.plot(data[4], data[5], "ob",)
# plt.show()

# # plot x
# plt.xlabel(labels[0])
# plt.ylabel(labels[1]+";"+labels[4])
# plt.plot(data[0], data[1], "oy",)
# plt.plot(data[0], data[4], "ob",)
# plt.show()

# # plot y
# plt.xlabel(labels[0])
# plt.ylabel(labels[2]+";"+labels[5])
# plt.plot(data[0], data[2], "oy",)
# plt.plot(data[0], data[5], "ob",)
# plt.show()

# # plot yaw
# plt.xlabel(labels[0])
# plt.ylabel(labels[3]+";"+labels[6])
# plt.plot(data[0], data[3], "oy",)
# plt.plot(data[0], data[6], "ob",)
# plt.show()

