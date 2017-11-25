#!/usr/bin/env python

import sys
import time
import math
import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
# no yaw or odometry

#0.5 for lookahead point
LOOKAHEAD_DISTANCE = 0.5 # [m]

''' Wanted distance from the wall '''
P = 0.4 # [m]

# take into account L
''' 's' in sketch, L from previous assignment '''
WHEEL_DISTANCE = 0.255

class SpeedController(object):
    
    def __init__(self, speed_arg, duration):
        Q_SIZE = 1000 # must be big, otherwise problems??? - car doesn't respond :(
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=Q_SIZE)
        time.sleep(1) # for initiating publisher
        # rospy.sleep(1)
        self.speed_arg = speed_arg
        self.drive_duration = duration
        self.is_driving = False

    def start(self):
        # rospy.loginfo("starting...")
        # forward < 0 > backwards
        self.is_driving = True
        # for i in range(1, 10000): # STUPID DUMB CAR
        self.pub_speed.publish(self.speed_arg)

    def stop(self):
        # rospy.loginfo("stopping...")
        self.pub_speed.publish(0)
        self.is_driving = False

    def drive_journey(self):
        self.stop()
        # while True:
        self.start()
        # while self.is_driving: #FUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU
        # DRIIIIIIIIIIIIIVEVEEVEVEEVEEEEEEEEEEEEEEEE
        rospy.Timer(rospy.Duration(0.1), lambda _: self.start(), oneshot=True)
        rospy.Timer(rospy.Duration(0.2), lambda _: self.start(), oneshot=True)
        rospy.Timer(rospy.Duration(self.drive_duration), lambda _: self.stop(), oneshot=True)

class SteeringController(object):

    def __init__(self):
        self.pub_steer = rospy.Publisher("/manual_control/steering", Int16, queue_size=100)
        time.sleep(1) # for initiating publisher
        # rospy.sleep(1)

    def steer(self, degrees):
        ''' degrees==0 -> straight '''

        rospy.loginfo('Steering %sdegrees...' % degrees)
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


class ScanReciever(object):
    
    def __init__(self, speed_ctrl, steer_ctrl, is_only_listening):
        self.speed_ctrl = speed_ctrl
        self.steer_ctrl = steer_ctrl

        self.wanted_alfa = 40 # deg
        self.wanted_bside_angle = 250 # this+wanted_alfa == a-side angle
        
        # we'll get these after initial receiving of the topic
        self.alfa = 0.0
        self.a_idx = -1
        self.b_idx = -1
        self.init_time = 0.0
        self.has_recieved_init_values = False

        K_P = 1.0
        K_D = 0.0001
        self.ctrl_angle = PDController(K_P, K_D, 0.0)

        # for recording and plotting
        self.rec_time = []
        self.rec_dist = []
        self.rec_theta = []
        self.rec_steer = []

        if is_only_listening:
            # measuring of theta & everything...
            rospy.Subscriber("scan", LaserScan, self.listen_laser, queue_size=None)
        else:
            # actual driving
            rospy.Subscriber("scan", LaserScan, self.drive_beside_wall, queue_size=None) #closest to 0
        # time.sleep(1)

    def listen_laser(self, scan_msg):
        # rospy.loginfo("LEN=%s\n%s\n--------" % (len(scan_msg.ranges), scan_msg))

        len_ranges = len(scan_msg.ranges)
        idx_ratio = len_ranges/360.0

        flt_first_idx = idx_ratio * self.wanted_bside_angle
        self.b_idx = int(round(flt_first_idx))
        self.a_idx = int(round(flt_first_idx + self.wanted_alfa*idx_ratio))

        # rospy.loginfo(">>>idx_ratio=%s; b_idx=%s; a_idx=%s" % (idx_ratio, self.b_idx, self.a_idx))
        
        real_angle_to_b = self.b_idx * scan_msg.angle_increment
        real_angle_to_a = self.a_idx * scan_msg.angle_increment

        self.alfa = real_angle_to_a - real_angle_to_b

        ratb_deg = np.rad2deg(real_angle_to_b)
        rata_deg = np.rad2deg(real_angle_to_a)

        # rospy.loginfo('>>>ALFA(deg)=%s; Angle to b=%sdeg; Angle to a=%sdeg' % (np.rad2deg(self.alfa), ratb_deg, rata_deg))

        a = scan_msg.ranges[self.a_idx]
        b = scan_msg.ranges[self.b_idx]

        if math.isinf(a) or math.isinf(b):
            return

        d,theta,theta_multiplier = self.get_d_and_theta(a,b)

        # rospy.loginfo('>>> a=%s; b=%s; d=%s; THETA=%s; THETA(deg)=%s' % (a,b,d,theta, np.rad2deg(theta)))

        c_y = d + math.sin(theta) * WHEEL_DISTANCE
        theta_star = math.atan((P-c_y)/LOOKAHEAD_DISTANCE) #* theta_multiplier
        if (c_y < P and theta_star > 0.0) or (c_y > P and theta_star < 0.0):
            theta_star *= -1

        rospy.loginfo('>>> a=%s; b=%s, THETA=%sdeg; c_y=%s; THETA*=%s; THETA*(deg)=%s' % (a,b,np.rad2deg(theta), c_y, theta_star, np.rad2deg(theta_star)))


    def drive_beside_wall(self, scan_msg):
        if not self.has_recieved_init_values: # 1st recieve - setup values
            INITIAL_ANGLE = 10
            self.listen_laser(scan_msg)
            self.steer_ctrl.steer(INITIAL_ANGLE) # so it doesn't go straight in the beginning
            self.ctrl_angle.update(INITIAL_ANGLE)
            self.speed_ctrl.drive_journey()
            self.init_time = time.time()

            # plot after drive
            plot_time = rospy.Duration(self.speed_ctrl.drive_duration+0.5)
            rospy.Timer(plot_time, lambda _: self.plot_values(), oneshot=True)
            self.has_recieved_init_values = True

        a = scan_msg.ranges[self.a_idx]
        b = scan_msg.ranges[self.b_idx]

        if math.isinf(a) or math.isinf(b): # math.isnan(a) or math.isnan(b) or 
            return

        d,theta,theta_multiplier = self.get_d_and_theta(a,b)
        # rospy.loginfo('>>> a=%s; b=%s; d=%s; THETA=%s; THETA(deg)=%s' % (a,b,d,theta, np.rad2deg(theta)))

        c_y = d + math.sin(theta) * WHEEL_DISTANCE
        theta_star = math.atan((P-c_y)/LOOKAHEAD_DISTANCE) #* theta_multiplier
        if (c_y < P and theta_star > 0.0) or (c_y > P and theta_star < 0.0):
            theta_star *= -1
        theta_star_deg = np.rad2deg(theta_star)
        rospy.loginfo('>>> c_y=%s; THETA*=%srad; THETA*=%sdeg' % (c_y, theta_star, theta_star_deg))

        # TODO record
        self.rec_dist.append(c_y) # d?
        self.rec_theta.append(np.rad2deg(theta)) #theta_star??
        self.rec_time.append(time.time() - self.init_time)

        pd_angle_output = self.ctrl_angle.update(theta_star_deg)

        steer_output = -pd_angle_output #0.0
        
        self.steer_ctrl.steer(steer_output)
        self.rec_steer.append(steer_output)

        # rospy.loginfo('Steer output=%s', steer_output)


    def get_d_and_theta(self, a, b):
        alfa = self.alfa
        PI = math.pi # np.deg2rad(180)
        theta_multiplier = 1 # TODO maybe reverse? -> prob. not

        if b > a:
            b,a=a,b
            theta_multiplier = -1

        c = math.sqrt(a*a + b*b - 2.0*a*b*np.cos(alfa)) # 3rd side of triangle a,b,c - Law of cosine
        phi = math.asin((b*math.sin(alfa)) / c) # angle between a,c - Law of sine
        beta = PI - phi - alfa # angle between b,c
        omega = PI - phi - alfa/2 # angle between x,c' in triangle a,x,c'
        
        sin_phi = math.sin(phi)
        x = (a*sin_phi) / math.sin(omega) # 'ray' from the 'side'
        d = sin_phi*a #if a>b else math.sin(beta)*b # perpendicular from car to the table
        theta = math.acos(d/x) * theta_multiplier # angle between the 'ray' from the side and the perpendicular 'd'

        return (d, theta, theta_multiplier)

    def plot_values(self):
        time_label = 'Time [s]'

        plt.figure(1)
        plt.subplot(311)
        plt.ylabel('Distance from the wall in meters')
        plt.xlabel(time_label)
        plt.ylabel('Distance [m]')
        plt.grid()
        plt.plot(self.rec_time, self.rec_dist)

        # plt.figure(2)
        plt.subplot(312)
        plt.title('Theta in degrees')
        plt.xlabel(time_label)
        plt.ylabel('Theta [deg]')
        plt.grid()
        plt.plot(self.rec_time, self.rec_theta)

        # plt.figure(3)
        plt.subplot(313)
        plt.title('Steering output in degrees (0 is straight, negative left, positive right)')
        plt.xlabel(time_label)
        plt.ylabel('Steering angle [deg]')
        plt.grid()
        plt.plot(self.rec_time, self.rec_steer)
        plt.show()


def main(args):
    rospy.init_node('closing_the_loop')

    SPEED_ARG = -150
    DRIVE_DURATION = 15

    speed_ctrl = SpeedController(SPEED_ARG, DRIVE_DURATION)
    steer_ctrl = SteeringController()

    is_only_listening = False
    scan_receiver = ScanReciever(speed_ctrl, steer_ctrl, is_only_listening)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
