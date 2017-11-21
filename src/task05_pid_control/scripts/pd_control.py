#!/usr/bin/env python

import rospy
import sys
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry # estimation of position without gps or anything

# rostopic type /odom
# nav_msgs/Odometry

# "/odom/pose/pose/position/y"

# (0.2 - odom_y) = e(t) = (desired-actual) in time t
# difference_change_of_error = [e(t)-e(t-1)]/time (divided by /time for normalization - or *frequency)

# Kd = 0 - just with Pcontroller, then start playing with Kd(damping)
# u(t) = Kp(0.2 - odom_y) + Kd(difference_change_of_error)

#start at 0,0; should follow y=0.2

# subscribe to yaw, always when subscribe - somehow measure freq


# rostopic type /odom/pose/pose/position/y
# nav_msgs/Odometry pose/pose/position/y float64

def odom_cb(odom_arg):
    print "%s\n=====\n" % odom_arg


def main(args):
    rospy.init_node("pd_control")

    # K_P = 1.3 # TODO change
    # SPEED_ARG = -150
    # DRIVE_DURATION = 16
    
    # speed_ctrl = SpeedController(SPEED_ARG, DRIVE_DURATION)
    # steer_ctrl = SteeringController()
    # p_controller = PController(K_P, speed_ctrl, steer_ctrl)

    rospy.Subscriber("odom/pose/pose/position/y", Float64, odom_cb, queue_size=10)

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)