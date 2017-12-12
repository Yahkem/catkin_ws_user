#!/usr/bin/env python

import sys
import rospy
import cv2
import json
import os
import tf
import numpy as np
from scipy import stats
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError


class KalmanFilter(object):
    
    def __init__(self):
        rospy.loginfo("Initializing KalmanFilter instance...")

        # rospy.Publisher("/global_position/odom", Odometry, queue_size=1)
        self.sub_distorted = rospy.Subscriber("/global_position/odom", Odometry, self.process_odom, queue_size=1)
        
        rospy.loginfo("KalmanFilter instance initialized!")

    def process_odom(self, odom_msg):
        pose_odom = odom_msg.pose.pose
        position_odom = pose_odom.position
        orientation_odom = pose_odom.orientation

        global_x = position_odom.x
        global_y = position_odom.y
        _, _, global_yaw = tf.transformations.euler_from_quaternion([
            orientation_odom.x,
            orientation_odom.y,
            orientation_odom.z,
            orientation_odom.w
        ])

        rospy.loginfo("Position = %s" % position_odom)
        rospy.loginfo("Orientation = %s" % orientation_odom)
        rospy.loginfo("Yaw = %s" % global_yaw)

        self.position_odom=position_odom
        self.orientation_odom=orientation_odom
        self.global_x=global_x
        self.global_y=global_y
        self.global_yaw=global_yaw
        
    def predict(self):
        # TODO
        kh = np.array([self.global_x, self.global_y, self.global_yaw]) # position of the car, [cos(yaw)*v+tx; sin(yaw)*v*ty, yaw]
        kV = [] # TODO, we obtain it - process noise
        kH = [] # TODO [dx, dy, dyaw]
        kP = [] # kP += kV

    def update(self):
        # TODO
        z = [] # measurements and calculation
        kP = [] 
        kW = []


def main(args):
    rospy.init_node('kalman_filter', anonymous=True)

    # Create KalmanFilter object and listen to Odometry messages
    kalman = KalmanFilter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)


# K=0 -> I care about prediction, K=1 -> caring about sensor data -  std squared
#x,y,theta space
# kH = predicted position
# kP = predicted uncertainty
# kV = Q in slides (3x3) - he gives it
# dX, dY - diff. position, in world frames 

# pred -    [xt'] = [xt-1] + [v] * [delta t] * [cosTheta]
# update -  [xt] = ([k]-1)* [xt'] + [z] * [k]
# k-calculated, updated (should converge if Q,R constant, which is our case)
# Q=error/process noise mtrx,R=calculated from positions from bulbs - are fixed
# yaw = world coords, Vx=car velocity

#subscribe to both odom and predicted

#pred,pred,...,pred->update, dont have to switch
# pred from odom, update from ours

# How Kalman works http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
# State x=(p,v)  # position,velocity
# both vars, pos and vel, are random and Gaussian distributed - each has mean phi and variance sigma -> in 2d Gaussian blob ("oval")
# Covariance matrix SIGMA(S)=symmetric (correlation between i-th and j-th state variable)
# at time k: estimate=x_k=[p,v]; covarMat=P_k=[[S_pp S_pv] [S_vp S_vv]]
# currentState=(k-1)
# v_k = v_k-1                   # velocity same
# p_k = p_k-1 + deltaT*v_k-1    # position new
# in other words x_k = [[1 deltaT] [0 1]] * x_k-1 = F_k*x_k-1; F_k==predictionMatrix
# if we multiply each point in distribution by mat. A:
#   Cov(x) = S; Cov(Ax) = A*S*A_transpos
#   -> x_k = F_k * x_k-1
#   -> P_k = F_k * P_k-1 * F_k_transpos
# External influence: controlMat B_k, controlVec u_k
#   -> x_k = F_k*x_k-1 + B_k*u_k
# every state in our original extimate could have moved to a RANGE of states because of external uncertainty (slipping, drift)
#   -> each of the predicted points is in Gaussian blob with covariance Q_k
#   -> this produces a new Gauss. blob, with different covariance and same mean
# x_k = F_k*x_k-1 + B_k*u_k
# newBestEstimate = Prediction * prevEstimate + Correction*knownExternalInfluences
# 
# P_k = F_k*P_k-1*F_k_trans + Q_k
# newUncertainty = Prediction*oldUncertainty(*PredTransposed) + AdditionalEnvUncertainty
# P_k - covarMat
#
# all of the above is without getting sensory data
# units and scale of the reading might not be the same as the units and scale of the state we're keeping track of
# H_k = matrix modeling the sensors; from above:
# (mean) phi_expected = H_k * x_k
# (covar) S_expected = H_k * P_k * H_k_transpos
#
# sensors might be unreliable, every state in our original estimate might be a RANGE of sensor readings
# since there is some uncertainty, some states are more likely than others
# covarOfSensorUncertainty = R_k
# the distribution has a mean - z_k
# now there are 2 Gaussian blobs - 
#   one surrounding our transformed prediction
#   one surrounding the sensor reading
# we have 2 probabilities:
#   the probability that z_k from sensor is (mis-)measurement of (z1, z2) // components of vector z_k
#   the probability that our previous estimate thinks (z1, z2) is the reading we SHOULD see
# probability that BOTH our true -> multiply -> Overlap (another Gauss blob)
# in 1d -> multiplying 2 gaussian curves
# N(x,phi0,sig0) * N(x,phi1,sig1) = N(x,phi',sig')
#
# newMean=mean0 + k(mean1 - mean0)
# phi' = phi0 + k(phi1 - phi0)
#
# newVariance**2 = var0**2 - k*(var**2)
# sigma^2 = sigma_0^2 - k*sigma_0^2
# k = sigma_0^2 / (sigma_0^2 + sigma_1^2)
#
# matrix version:
# K = S0 * (S0+S1)_trans
# phi_vec' = phi_vec0 + K(phi_vec1 - phi_vec0)
# S' = S0 - KS0
# K == kalman gain - determines how much the innovation (covarMat S) will be considered for the new state estimate
