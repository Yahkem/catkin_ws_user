#!/usr/bin/env python

import sys
import time
import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int16
# from std_msgs.msg import Float32
# from std_msgs.msg import Float64
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

class SpeedController(object):
    
    def __init__(self, speed_arg, duration):
        Q_SIZE = 10000 # must be big, otherwise problems??? - car doesn't respond :(
        self.pub_speed = rospy.Publisher("manual_control/speed", Int16, queue_size=Q_SIZE)
        self.speed_arg = speed_arg
        self.drive_duration = duration
        self.is_driving = False

    def start(self):
        # rospy.loginfo("starting...")
        # forward < 0 > backwards
        self.is_driving = True
        # for i in range(1, 10000): # STUPID CAR
        self.pub_speed.publish(self.speed_arg)

    def stop(self):
        # rospy.loginfo("stopping...")
        self.pub_speed.publish(0)
        self.is_driving = False

    def drive_journey(self):
        self.start()
        # while self.is_driving: #FU, ASSHOLE
        # DRIIIIIIIIIIIIIVEVEEVEVEEVEEEEEEEEEEEEEEEE
        rospy.Timer(rospy.Duration(0.1), lambda _: self.start(), oneshot=True)
        # rospy.Timer(rospy.Duration(0.1), lambda _: self.start(), oneshot=True)
        rospy.Timer(rospy.Duration(self.drive_duration), lambda _: self.stop(), oneshot=True)

class SteeringController(object):

    def __init__(self):
        self.pub_steer = rospy.Publisher("manual_control/steering", Int16, queue_size=100)

    def steer(self, degrees):
        ''' degrees==0 -> straight '''

        topic_arg = self.steering_angle_mapping(degrees)
        self.pub_steer.publish(topic_arg)

    def steering_angle_mapping(self, degrees):
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


class PDController(object):

    def __init__(self, Kp, Kd, speed_ctrl, steer_ctrl):
        self.Kp = Kp
        self.Kd = Kd
        self.speed_ctrl = speed_ctrl
        self.steer_ctrl = steer_ctrl
        
        # set_point == what we want -> 0.2 y
        self.set_point = 0.2

        # result from update()
        self.output = 0.0

        ''' first memasured coords - y==0 '''
        self.first_measure = True

        # we will be measuring only every Xth callback
        self.cb_inc = -1
        self.xth_cb = 20

        # set to time.time() in the beginning for plotting
        self.initial_time = 0.0
        self.initial_y_pos = 0.0
        self.final_y_pos = 0.0

        # previous values for the Derivative part
        self.last_time = 0.0
        self.last_error = 0.0

        # for plotting
        self.time_arr = []
        self.y_coordinates = []
        self.is_chart_plotted = False

    # def add_time(self):
    #     cur_time = time.time()
    #     self.time_arr.append(cur_time-self.initial_time)

    def update(self, process_variable, add_time):
        ''' process_variable = what we measure '''

        self.y_coordinates.append(process_variable)
        
        error = self.set_point - process_variable
        delta_error = error - self.last_error

        cur_time = time.time()
        delta_time = cur_time - self.last_time
        if add_time:
            self.time_arr.append(cur_time - self.initial_time)

        # remember for next
        self.last_time = cur_time
        self.last_error = error
        
        dterm = (delta_error/delta_time) if delta_time > 0 else 0.0

        self.output = (self.Kp * error) + (self.Kd * dterm) # Proportional + Derivative
        # self.output = -self.output # minus, because -=left, +=right
        # TODO output->steering angle?
        print "Updated, output=%s" % self.output

    def update_and_steer(self, y_pos, add_time):
        self.update(y_pos, add_time)
        # if add_time: self.add_time()
        self.steer_ctrl.steer(self.output)

    def plot_squared_diffs_over_time(self):
        plt.figure(1)
        plt.subplot(111)
        plt.title('Initial Y-Position=%s | Desired Y-Pos=%s | Final Y-Pos=%s' % (self.initial_y_pos, self.set_point, self.final_y_pos))
        plt.plot(self.time_arr, self.y_coordinates)
        plt.xlabel('Time [s]')
        plt.ylabel('Y-coordinates')
        plt.show()        

    def odom_cb(self, odom_arg):
        # 1) (heading_angle) -> steer_command
        
        self.cb_inc += 1
        if self.cb_inc % self.xth_cb != 0: 
            return

        print "ARG=%s" %odom_arg
        y_pos = odom_arg.pose.pose.position.y

        if self.first_measure:
            self.first_measure = False
            self.is_chart_plotted = False

            self.set_point = y_pos + 0.2 # always y=0.2, assuming we start on 0 
            
            self.initial_y_pos = y_pos
            print "INITIAL Y_POS=%s" % self.initial_y_pos
            # print "SET POINT=%s" % self.set_point
            
            self.steer_ctrl.steer(0) # at first go straight# TODO y_error -> steering
            self.time_arr.append(0)
            self.initial_time = time.time()
            self.last_time = self.initial_time

            self.speed_ctrl.drive_journey() # Go!
            
            self.update_and_steer(y_pos, False)
            #rospy.Timer(rospy.Duration(self.speed_ctrl.drive_duration+0.1), lambda _: self.plot_squared_diffs_over_time(), oneshot=True)
            return

        # self.speed_ctrl.start()
        # self.add_time()
        print "Y_pos: %s" % y_pos
        
        self.update_and_steer(y_pos, True)

        if not self.speed_ctrl.is_driving and not self.is_chart_plotted:
            self.final_y_pos = y_pos
            self.plot_squared_diffs_over_time()
            self.is_chart_plotted = True
            rospy.signal_shutdown("Drive ended")

def odom_cb(odom_arg):
    print "Y=%s\nTime=%s\n=====" % (odom_arg.pose.pose.position.y, time.time())


def main(args):
    rospy.init_node("pd_control")

    K_P = 0
    K_D = 0 # TODO change
    SPEED_ARG = -120
    DRIVE_DURATION = 4
    
    speed_ctrl = SpeedController(SPEED_ARG, DRIVE_DURATION)
    steer_ctrl = SteeringController()
    pd_controller = PDController(K_P, K_D, speed_ctrl, steer_ctrl)

    # "odom" topic
    rospy.Subscriber("odom", Odometry, pd_controller.odom_cb, queue_size=10)

    # pbspd = rospy.Publisher("manual_control/speed", Int16, queue_size=100)
    # steer_ctrl.steer(0)
    # pbspd.publish(SPEED_ARG)
    # pbspd.publish(SPEED_ARG)
    # pbspd.publish(SPEED_ARG)
    # pbspd.publish(SPEED_ARG)
    # pbspd.publish(SPEED_ARG)
    # pbspd.publish(SPEED_ARG)

    # rospy.Timer(rospy.Duration(0.1), lambda _: pbspd.publish(SPEED_ARG), oneshot=True)
    # rospy.Subscriber("odom", Odometry, odom_cb, queue_size=10)
    # rospy.Timer(rospy.Duration(5), lambda _: pbspd.publish(0), oneshot=True)
    # r = rospy.Rate(1)
    try:
        rospy.spin()
        # while not rospy.is_shutdown():
        #     r.sleep()
    except KeyboardInterrupt:
        speed_ctrl.pub_speed.publish(0)
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)