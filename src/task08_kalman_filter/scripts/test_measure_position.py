from __future__ import print_function
import cv2
import numpy as np
from numpy import pi, sin, cos
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf

# SETUP
rospy.init_node('measure_position', anonymous=True)
global_x = None
global_y = None
global_yaw = None
relative_x = None
relative_y = None
relative_yaw = None
start_time = rospy.Time.now()
file = open('measure.csv', 'w')
file.write('time;global_x;global_y;global_yaw;relative_x;relative_y;relative_yaw\n')

def ros_callback_global_odom(data):
    global global_x, global_y, global_yaw
    # publish
    global_x = data.pose.pose.position.x
    global_y = data.pose.pose.position.y
    _, _, global_yaw = tf.transformations.euler_from_quaternion([
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    ])
    time = rospy.Time.now()-start_time
    file.write(str(time)+";"+str(global_x)+";"+str(global_y)+";"+str(global_yaw)+";"+str(relative_x)+";"+str(relative_y)+";"+str(relative_yaw)+'\n')
    print("global  ", time, global_x, global_y, global_yaw)


def ros_callback_relative_odom(data):
    global relative_x, relative_y, relative_yaw
    # publish
    relative_x = data.pose.pose.position.x
    relative_y = data.pose.pose.position.y
    _, _, relative_yaw = tf.transformations.euler_from_quaternion([
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    ])
    time = rospy.Time.now()-start_time
    file.write(str(time)+";"+str(global_x)+";"+str(global_y)+";"+str(global_yaw)+";"+str(relative_x)+";"+str(relative_y)+";"+str(relative_yaw)+'\n')
    print("relative", time, relative_x, relative_y, relative_yaw)



rospy.Subscriber("/global_position/odom", Odometry, ros_callback_global_odom, queue_size=1)
rospy.Subscriber("/odom", Odometry, ros_callback_relative_odom, queue_size=1)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
cv2.destroyAllWindows()
