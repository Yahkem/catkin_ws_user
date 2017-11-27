#!/usr/bin/env python

import sys
import time
import math
import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int16
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
        Q_SIZE = 1000 # must be big, otherwise some problems...
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=Q_SIZE)
        time.sleep(1) # init publisher
        self.speed_arg = speed_arg
        self.drive_duration = duration
        self.is_driving = False

    def start(self):
        rospy.loginfo("starting...")
        # forward < 0 > backwards
        self.is_driving = True
        self.pub_speed.publish(self.speed_arg)

    def stop(self):
        # rospy.loginfo("stopping...")
        self.pub_speed.publish(0)
        self.is_driving = False

    def drive_journey(self):
        self.start()
        rospy.Timer(rospy.Duration(self.drive_duration), lambda _: self.stop(), oneshot=True)

class SteeringController(object):

    def __init__(self):
        self.pub_steer = rospy.Publisher("manual_control/steering", Int16, queue_size=100)
        time.sleep(1) # init publisher

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
        steering_arg = int(round(deg_interpolated)) #+9 for sim
        rospy.loginfo("SteeringArg=%s" % steering_arg)

        return steering_arg


class PDController(object):

    def __init__(self, kp, kd, set_point=0.0, sample_time=0.0):
        self.kp = kp
        self.kd = kd
        self.sample_time = sample_time

        self.cur_time=time.time()
        self.prev_time = self.cur_time
        self.prev_error = 0.0
        self.set_point = set_point

        self.p = 0.0
        self.d = 0.0

        self.output = 0.0

    def update(self, feedback_value):
        error = self.set_point - feedback_value

        self.cur_time = time.time()
        delta_time = self.cur_time - self.prev_time
        print "dtime=%s" % delta_time

        if delta_time >= self.sample_time:
            self.p = self.kp * error

            self.d = 0.0

            if delta_time > 0.0:
                delta_error = error - self.prev_error
                self.d = delta_error / delta_time # delta_error*freq

        self.prev_time = self.cur_time
        self.prev_error = error

        self.output = self.p + (self.kd * self.d)

        return self.output

class OdomReceiver(object):

    def __init__(self, speed_ctrl, steer_ctrl):
        self.speed_ctrl = speed_ctrl
        self.steer_ctrl = steer_ctrl
        
        #get later
        self.init_time = 0.0

        # we want Y=0.2
        self.wanted_y = 0.2 # TOUPDATE, beginning = [0,0]

        # we will be measuring only every Xth callback
        self.yaw_cb_inc = -1
        self.xth_cb = 10

        K_P = 1.0
        K_D = 0.000 #0.0001
        self.pdctrl_distance = PDController(K_P, K_D, self.wanted_y)
        self.pdctrl_yaw = PDController(K_P, K_D, 0.0)

        # for recording and plotting
        self.rec_time = []
        self.rec_ycoords = []

        self.has_corrected_yaw = False
        self.has_started = False
    
    def odom_cb(self, odom_msg):
        # every xth cb
        self.yaw_cb_inc += 1
        if self.yaw_cb_inc % self.xth_cb != 0: 
            return

        pose = odom_msg.pose.pose
        position = pose.position
        orientation = pose.orientation

        y_pos = position.y
        yaw = self.quaternion_to_yaw(orientation)
        yaw_deg = np.rad2deg(yaw)

        if not self.has_started:
            # self.ctrl_angle.update(INITIAL_ANGLE)
            # self.pdctrl_distance.update(y_pos)
            self.wanted_y = y_pos + 0.2
            self.pdctrl_distance.set_point = self.wanted_y
            print "Wanted Y=%s" % self.wanted_y
            self.speed_ctrl.drive_journey()
            self.init_time = time.time()

            self.has_started = True

        if not self.speed_ctrl.is_driving and self.has_started:
            self.plot_values()
            rospy.signal_shutdown("Plotted chart")

        pd_output = self.pdctrl_distance.update(y_pos)
        pd_yaw_out = self.pdctrl_yaw.update(yaw_deg)

        # append values to plot
        self.rec_time.append(time.time() - self.init_time)
        self.rec_ycoords.append(y_pos)

        wanted_steer_deg = 0.0

        if pd_yaw_out == 0.0 and not self.has_corrected_yaw:
            # initial yaw to 0.0, now we correct dist
            print "\n------------YAW CORRECTED!!!\n------------\n"
            self.has_corrected_yaw = True
        elif self.has_corrected_yaw:
            wanted_steer_deg = pd_output #self.map_output(pd_output) #-pd_output*15
        else:
            wanted_steer_deg = pd_yaw_out

        # wanted_steer_deg = pd_output if self.has_corrected_yaw else -30
        # wanted_steer_deg = 0#-pd_output*30
        
        info_tuple = (y_pos, pd_yaw_out, pd_output, wanted_steer_deg)
        rospy.loginfo("Y=%s; PDYawOut=%sdeg; PDOutput=%s; Steer=%sdeg;" % info_tuple)

        # SIM(gazebo) = -steer -> right, +steer->left
        # real car = +steer -> right, -steer ->left
        self.steer_ctrl.steer(wanted_steer_deg)

    def map_output(self, pd_output):
        #     inputs = [-1, 0, 1]
        #     outputs = [30 , -5, 30]
        #     return np.interp(pd_output, inputs, outputs)
        pd_outputs = [-0.2, -0.05, 0.0, 0.05, 0.2]
        steering_degrees = [30.0, 5.0, 0.0, -5.0, -30.0]
        return np.interp(pd_output, pd_outputs, steering_degrees)

    def quaternion_to_yaw(self, q):
        ''' yaw=(z-axis rotation) '''

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)  
        yaw = math.atan2(siny, cosy)

        return yaw

    def plot_values(self):
        # plt.figure(1)
        # plt.subplot(111)
        rospy.loginfo("-------\n-------")
        plt.title('Y coords over time, wanted Y=%s' % self.wanted_y)
        plt.xlabel('Time[s]')
        plt.ylabel('Y coordinates')
        plt.grid()
        plt.plot(self.rec_time, self.rec_ycoords)
        plt.show()
        rospy.signal_shutdown("Plotted chart")


def main(args):
    rospy.init_node("pd_control")

    SPEED_ARG = -180
    DRIVE_DURATION = 14
    
    speed_ctrl = SpeedController(SPEED_ARG, DRIVE_DURATION)
    steer_ctrl = SteeringController()
    odom_reciever = OdomReceiver(speed_ctrl, steer_ctrl)

    # "odom" topic
    rospy.Subscriber("odom", Odometry, odom_reciever.odom_cb, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        speed_ctrl.pub_speed.publish(0)
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
