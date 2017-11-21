#!/usr/bin/env python

import sys
import time
import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Float32
# from nav_msgs.msg import Odometry = estimation of position without gps or anything

# rostopic type /model_car/yaw
# std_msgs/Float32

#(whatWeWant - theta)^2 - 

# start +-10deg

#K_P = 1 #'????' # 0-1.1; 1 - tune so it doesnt oscillate

class SpeedController(object):
    
    def __init__(self, speed_arg, duration):
        self.pub_speed = rospy.Publisher("manual_control/speed", Int16, queue_size=10)
        self.speed_arg = speed_arg
        self.drive_duration = duration
        self.is_driving = False

    def start(self):
        rospy.loginfo("starting...")
        # 100..200=real speed backwards, 100 is too little for car No. 110
        self.pub_speed.publish(self.speed_arg)
        self.is_driving = True

    def stop(self):
        rospy.loginfo("stopping...")
        self.pub_speed.publish(0)
        self.is_driving = False

    def drive_journey(self):
        self.start()
        rospy.Timer(rospy.Duration(self.drive_duration), lambda _: self.stop(), oneshot=True)

class SteeringController(object):

    def __init__(self):
        self.pub_steer = rospy.Publisher("manual_control/steering", Int16, queue_size=10)

    def steer(self, degree):
        ''' degrees==0 -> straight '''

        topic_arg = self.steering_angle_mapping(degree)
        self.pub_steer.publish(topic_arg)

    def steering_angle_mapping(self, degrees):
        ''' degrees==0 -> straight '''

        topic_args = [0, 30, 60, 90, 120, 150, 179]
        measured = [-23.4823835367, -16.2088298025, -4.10722402982,
            6.64525217941, 16.4196652944, 27.7491348616, 30.8972609386]
        
        deg_min = measured[0]
        deg_max = measured[-1]

        # culling high/low degrees
        if degrees < deg_min:
            degrees = deg_min
        elif degrees > deg_max:
            degrees = deg_max
        
        deg_interpolated = np.interp(degrees, measured, topic_args)

        return int(round(deg_interpolated))


class PController(object):

    def __init__(self, Kp, speed_ctrl, steer_ctrl):
        self.Kp = Kp
        self.speed_ctrl = speed_ctrl
        self.steer_ctrl = steer_ctrl
        
        # set_point == what we want=initial yaw-10deg
        self.set_point = 0.0
 
        self.output = 0.0

        ''' first memasured yaw==desired-10 '''
        self.first_measure = True

        # we will be measuring only every Xth callback
        self.yaw_cb_inc = -1
        self.xth_cb = 30

        # set to time.time() in the beginning for plotting
        self.initial_time = 0.0
        # self.current_time = 0.0

        # for plotting
        self.time_arr = []
        self.squared_diffs = []
        self.is_chart_plotted = False

    def add_time(self):
        cur_time = time.time()
        self.time_arr.append(cur_time-self.initial_time)

    # process_variable = what we measure
    def update(self, process_variable):
        error = self.set_point - process_variable
        self.squared_diffs.append(error**2)
        self.output = self.Kp * error
        print "Updated, output(steer degree)=%s" % self.output

    def plot_squared_diffs_over_time(self):
        plt.plot(self.time_arr, self.squared_diffs)
        plt.xlabel('Time [s]')
        plt.ylabel('Squared yaw difference for Kp=%s' % self.Kp)
        plt.show()        

    def update_and_steer(self, yaw_angle):
        self.update(yaw_angle)
        self.steer_ctrl.steer(self.output)

    def yaw_cb(self, yaw):
        # 1) (heading_angle) -> steer_command
        self.yaw_cb_inc += 1

        if self.yaw_cb_inc % self.xth_cb != 0: return

        yaw_angle = yaw.data

        if self.first_measure:
            self.first_measure = False
            self.is_chart_plotted = False
            self.set_point = yaw_angle - 15
            
            print "INITIAL YAW=%s" % yaw_angle
            print "SET POINT=%s" % self.set_point
            
            self.steer_ctrl.steer(self.set_point)
            self.initial_time = time.time()
            self.time_arr.append(0)
            self.speed_ctrl.drive_journey()
            
            self.update_and_steer(yaw_angle)
            #rospy.Timer(rospy.Duration(self.speed_ctrl.drive_duration+0.1), lambda _: self.plot_squared_diffs_over_time(), oneshot=True)
            return

        # self.speed_ctrl.start()
        self.add_time()
        print "Yaw: %s" % yaw_angle
        
        self.update_and_steer(yaw_angle)

        if not self.speed_ctrl.is_driving and not self.is_chart_plotted:
            self.is_chart_plotted = True
            self.plot_squared_diffs_over_time()


def main(args):
    rospy.init_node("p_control")

    K_P = 1.3 # TODO change
    SPEED_ARG = -150
    DRIVE_DURATION = 16
    
    speed_ctrl = SpeedController(SPEED_ARG, DRIVE_DURATION)
    steer_ctrl = SteeringController()
    p_controller = PController(K_P, speed_ctrl, steer_ctrl)

    rospy.Subscriber("model_car/yaw", Float32, p_controller.yaw_cb, queue_size=10)

    # pb = rospy.Publisher("manual_control/speed", Int16, queue_size=1)
    # while True:
    #     pb.publish(0)
    # speed_ctrl.drive_journey()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)