from __future__ import print_function

import json

import cv2
import numpy as np
from random import uniform
from scipy import stats
from numpy import pi, sin, cos
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import Int16, String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseArray
import tf
from ColorBulb import ColorBulb




# SETUP
PARTICLE_X_MIN = 0
PARTICLE_X_MAX = 600
PARTICLE_Y_MIN = 0
PARTICLE_Y_MAX = 400
PARTICLES_COUNT = 100

particles = [
        np.random.rand(PARTICLES_COUNT)*(PARTICLE_X_MAX-PARTICLE_X_MIN)+PARTICLE_X_MIN,
        np.random.rand(PARTICLES_COUNT)*(PARTICLE_Y_MAX-PARTICLE_Y_MIN)+PARTICLE_Y_MIN,
        np.random.rand(PARTICLES_COUNT)*2*pi-pi
]
particles = np.array(particles).T

last_x = None
last_y = None
last_yaw = None

rospy.init_node('particle_triangulation', anonymous=True)
position_pub_odom = rospy.Publisher("/global_position/odom", Odometry, queue_size=1)
position_pub_odom2 = rospy.Publisher("/mcpf_gps", Odometry, queue_size=1)
# used for kalman filter proportions
position_pub_certainty_pos = rospy.Publisher("/global_position/certainty/pos", Float32, queue_size=1)
position_pub_certainty_yaw = rospy.Publisher("/global_position/certainty/yaw", Float32, queue_size=1)
pub = rospy.Publisher("/mcposearray", PoseArray, queue_size=1)

def ros_odom_callback(data):
    ''' Move all particles with car movement and add noise '''
    global particles, last_x, last_y, last_yaw
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    _, _, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    # calculate diffference to last position
    if last_x is not None:
        diff_x = last_x-x
        diff_y = y-last_y
        diff_yaw = yaw-last_yaw
        if diff_yaw < -pi:
            diff_yaw += 2 * pi
        if diff_yaw > pi:
            diff_yaw -= 2 * pi
        
        # move each particle by difference
        def move_particle(a):
            theta = a[2]-diff_yaw + uniform(-0.1, 0.1)
            if theta < -pi:
                theta += 2 * pi
            if theta > pi:
                theta -= 2 * pi

            rw = uniform(-2, 2)
            rh = uniform(-2, 2)
            x = a[0]+diff_x*100 + cos(theta) * rw + sin(theta) * rh
            y = a[1]+diff_y*100 + sin(theta) * rw + cos(theta) * rh
            return np.array([x, y, theta])
        particles = np.apply_along_axis(move_particle, 1, particles)

    # set current position as last position
    last_x = x
    last_y = y
    last_yaw = yaw


def ros_callback(data):
    ''' calculate particle error und create a new paricle generation '''
    global particles, PARTICLES_COUNT
    my_dict = json.loads(data.data)
    # position of car in the image
    pos_x = my_dict['width']/2
    pos_y = my_dict['height']/2
    bulbs = [ColorBulb(bulb) for bulb in my_dict['bulbs']]

    # calculate angles in the image
    # return tuple with image angle and global bulb position (for later calculation)
    angles = []
    for a in bulbs:
        if a.x_img==-1:
            continue
        angle = -np.arctan2(a.y_img-pos_y, a.x_img-pos_x)-pi/2
        if angle < -pi:
            angle += 2*pi
        angles.append((-angle, a.x_real, a.y_real))

    # calculate weight for particle a = (x, y, yaw)
    def my_func(a):
        """Average first and last element of a 1-D array"""
        STDDEV_SQUARED = 0.1
        error = 1
        for yaw, x, y in angles:
            e = np.arctan2(y-a[1], x-a[0])-a[2]-yaw
            while e < -pi:
                e+=2*pi
            while e > pi:
                e-=2*pi
            error *= np.exp(-e**2/STDDEV_SQUARED)
        return 1-error

    # calculte weights
    weights = 1-np.apply_along_axis(my_func, 1, particles)
    # sort particles and weights
    argorder = np.argsort(weights)[::-1]
    weights = weights[argorder]
    particles = particles[argorder, :]
    # loop through particles qith equal step size
    step_size = np.sum(weights)/float(PARTICLES_COUNT)
    i = step_size/2.0
    pos = 0
    s = weights[0]
    new_particles = []
    for _ in range(PARTICLES_COUNT):
        while s < i:
            s += weights[pos]
            pos += 1
        new_particles.append(particles[pos])
        i += step_size
    particles = np.array(new_particles)

    # publish pose array
    poseArray = PoseArray()
    poseArray.header.stamp = rospy.Time.now()
    poseArray.header.frame_id = "/odom"
    for x, y, yaw in particles:
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        somePose = Pose()
        somePose.position.x = x/100
        somePose.position.y = y/100
        somePose.position.z = 0.0

        somePose.orientation.x = quaternion[0]
        somePose.orientation.y = quaternion[1]
        somePose.orientation.z = quaternion[2]
        somePose.orientation.w = quaternion[3]

        poseArray.poses.append(somePose)

    pub.publish(poseArray)

    # lists of all x, y, yaw values
    xs = particles[:, 0]/100 # x
    ys = particles[:, 1]/100 # y
    ts = particles[:, 2] # theta = yaw

    # means
    mx = sum(xs)/PARTICLES_COUNT
    my = sum(ys)/PARTICLES_COUNT
    sinsum = sum(sin(ts))
    cossum = sum(cos(ts))
    mt = np.arctan2(sinsum, cossum)
    #print("mean", mx, my, mt)

    # std.dev.
    std_x = np.sqrt(sum(np.square(xs-mx)))
    std_y = np.sqrt(sum(np.square(ys-my)))
    sin_avg = (sinsum/PARTICLES_COUNT)
    cos_avg = (cossum/PARTICLES_COUNT)
    if sin_avg+cos_avg > 1:
        std_t  = 0
    else:
        std_t = np.sqrt(-np.log(sin_avg**2+cos_avg**2))
    #print("std ", std_x, std_y, std_t)

    # certainty for kalman filter
    certainty_pos = np.exp(-std_x)*np.exp(-std_y)
    certainty_yaw = np.exp(-std_t/0.5)
    print("cert", certainty_pos, certainty_yaw)


    # calc odom topic
    time = rospy.Time.now()
    odom_broadcaster = tf.TransformBroadcaster()
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, mt)
    odom_broadcaster.sendTransform(
        (mx, my, 0.),
        odom_quat,
        time,
        "base_link",
        "global_position"
    )
    odom = Odometry()
    odom.header.stamp = time
    odom.header.frame_id = "odom"
    odom.pose.pose = Pose(Point(mx, my, 0.), Quaternion(*odom_quat))
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))


    # publish
    position_pub_certainty_pos.publish(certainty_pos)
    position_pub_certainty_yaw.publish(certainty_yaw)
    position_pub_odom.publish(odom)
    position_pub_odom2.publish(odom)




rospy.Subscriber("/bulb_coords", String, ros_callback, queue_size=1)
rospy.Subscriber("/odom", Odometry, ros_odom_callback, queue_size=1)
try:
    rospy.spin()
except KeyboardInterrupt:
    #f_debug.close()
    print("Shutting down")
cv2.destroyAllWindows()
