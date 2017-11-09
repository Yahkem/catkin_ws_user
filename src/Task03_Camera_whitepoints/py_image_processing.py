#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
from operator import itemgetter

# from __future__ import print_function

class image_converter:

  def __init__(self):
    rospy.loginfo("Initializing image_converter...")
    self.bridge = CvBridge()
    
    #rosrun image_view image_view image:=/image_processing/greyscale_img
    #rosbag record /image_processing/greyscale_img
    self.pub_greyscale = rospy.Publisher("/image_processing/greyscale_img", Image, queue_size=1)
    #rosbag record /image_processing/bw_img
    #rosrun image_view image_view image:/image_processing/bw_img
    self.pub_bw = rospy.Publisher("/image_processing/bw_img", Image, queue_size=1)

    # /app/camera/rgb/image_raw;/camera/rgb/image_raw
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw", Image, self.callback, queue_size=1)

    self.processed = False
    rospy.loginfo("image_converter initialized!")

  def callback(self,data):
    #if self.processed: return
    self.processed=True

    rospy.loginfo("Callback!")

    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 200
    th, bw_img = cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    height, width = bw_img.shape

    # points=[]

    # #print bw_img.shape # (height,width)
    # for x in range(0,width):
    #     for y in range(0, height):
    #         if bw_img[y,x] == 255:
    #             points.append([x,y])
    
    #print bw_img.shape

    # https://docs.opencv.org/3.1.0/d4/d73/tutorial_py_contours_begin.html
    _, contours, hierarchy = cv2.findContours(bw_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    centers = []

    for cnt in contours: #cnt = contours[0]
    #contoured = cv2.drawContours(img3, contours, -1, (100,100,100), 3)
        area = cv2.contourArea(cnt)
        #print "area: %s" % area 

        if area < 1000 and area > 1: #areas byvaji 5-100
            # https://stackoverflow.com/questions/9074202/opencv-2-centroid
            moment = cv2.moments(cnt)
            #print "moment: %s" % moment

            cx=0;cy=0

            m00 = moment['m00']

            if m00 != 0:
                cx = int(moment['m10'] / m00)
                cy = int(moment['m01'] / m00)
            
                centers.append([cx,cy]) # TODO posunout za if?
        else: # delete big white areas
            bw_img = cv2.drawContours(bw_img, [cnt], 0, (0,0,0), cv2.FILLED)
            #print "Center: [%s,%s]" % (cx, cy)

    # sorted() is stable, so 1st by Y-reversed (image coords are +y==down)
    centers = sorted(centers, key=itemgetter(1)) 
    # and then by X
    centers = sorted(centers, key=itemgetter(0))
    
    # for i,p in enumerate(centers):
    #     centers[i] = [p[0], height-p[1]]

    #print centers

    try:
        self.pub_greyscale.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))#gray
        rospy.loginfo("Greyscale image published")
    except CvBridgeError as e:
        print(e)

    try:
        self.pub_bw.publish(self.bridge.cv2_to_imgmsg(bw_img, "mono8"))
        rospy.loginfo("Black and white image published")
    except CvBridgeError as e:
        print(e)

    # lets say, that the bottom left corner of the bottom left square is [0,0,0]
    # z|, y/, x_
    object_points = get_3d_coords()
    print "\n-----------------"
    print "3D/object points - measured:\n%s" % object_points
    print "-----------------"
    print "\n-----------------"
    print "2D/image points (centers of white squares):\n%s" % centers
    print "-----------------"
    
    image_points_np = np.array(centers, dtype=np.float64)
    process_2d_3d_points(image_points_np, object_points)


def process_2d_3d_points(image_points, object_points):
    print "Image points size: %s; Object points size: %s" % (len(image_points), len(object_points))

    # intrinsic parameters
    fx=614.1699; fy=614.9002 # camera focal lengths
    cx=329.9491; cy=237.2788 # optical centers in pixels
    camera_matrix = np.matrix([
        [fx,0,  cx],
        [0, fy, cy],
        [0, 0,  1]])

    print "Intrinsic parameter (camera) matrix:\n{0}".format(camera_matrix)

    # distortion parameters
    k1 = 0.1115
    k2 = -0.1089
    t1 = 0
    t2 = 0
    dist_coeffs = np.array([k1, k2, t1, t2])
    
    print "Distortion parameter matrix:\n{0}".format(dist_coeffs)
    
    #image_points=np.array([[2,2],[3,3],[4,4],[5,5]], dtype=np.float64) # !!!! float64, else PnP->ERROR
    #object_points=np.array([[2,3,1],[3,3,1],[4,4,1],[5,5,1]], dtype=np.float64)
    
    #rvec=output rotation vector, tvec=output translation vector
    retval_pnp, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    
    print "rvec: %s" % rvec
    print "tvec: %s" % tvec

    mat_rotation, _ = cv2.Rodrigues(rvec) #supply rvec
    print "Rotation matrix:\n{0}".format(mat_rotation)

    inverse_rmat = np.transpose(mat_rotation)
    inverse_tvec = np.multiply(np.negative(inverse_rmat), tvec)

    print "Inverse rotation matrix:\n{0}".format(inverse_rmat)
    print "Inverse translation vector:\n{0}".format(inverse_tvec)

    rm21 = inverse_rmat[2,1]
    rm22 = inverse_rmat[2,2]
    
    yaw = np.arctan2(inverse_rmat[1,0], inverse_rmat[0,0])
    pitch = np.arctan2(-inverse_rmat[2,0], math.sqrt(rm21**2 + rm22**2))
    roll = np.arctan2(rm21, rm22)
    
    # print "yaw: %s" % yaw # np.rad2deg?
    # print "pitch: %s" % pitch
    # print "roll: %s" % roll
    print "yaw: %s" % np.rad2deg(yaw)
    print "pitch: %s" % np.rad2deg(pitch)
    print "roll: %s" % np.rad2deg(roll)

    #print image_points
    #print object_points
    # print rotationMatrixToEulerAngles(inverse_rmat)

# Checks if a matrix is a valid rotation matrix.
# def isRotationMatrix(R) :
#     Rt = np.transpose(R)
#     shouldBeIdentity = np.dot(Rt, R)
#     I = np.identity(3, dtype = R.dtype)
#     n = np.linalg.norm(I - shouldBeIdentity)
#     return n < 1e-6
 
# # Calculates rotation matrix to euler angles
# # The result is the same as MATLAB except the order
# # of the euler angles ( x and z are swapped ).
# def rotationMatrixToEulerAngles(R):
#     assert(isRotationMatrix(R))
        
#     sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        
#     singular = sy < 1e-6

#     if  not singular :
#         x = math.atan2(R[2,1] , R[2,2])
#         y = math.atan2(-R[2,0], sy)
#         z = math.atan2(R[1,0], R[0,0])
#     else :
#         x = math.atan2(-R[1,2], R[1,1])
#         y = math.atan2(-R[2,0], sy)
#         z = 0

#     x = np.rad2deg(x)
#     y = np.rad2deg(y)
#     z = np.rad2deg(z)
#     return np.array([x, y, z])
    

def get_3d_coords():
    ''' Returns measured points in world coordinates, units are cm'''
    # C D
    # B E
    # A F
    #  |cam
    # [0,0,0]==left bottom corner of A
    coords_3d = [
        [1, 1, 0],
        [1, 31, 0],
        [1, 61, 0],
        [23, 62, 0],
        [23, 32, 0],
        [23, 2, 0]
    ]
    # coords_3d = [
    #     [0, 0, 0],
    #     [0, 30, 0],
    #     [0, 60, 0],
    #     [22, 61, 0],
    #     [22, 31, 0],
    #     [22, 1, 0]
    # ]

    # ordered 1st by X, then by Y
    #coords_3d = sorted(coords_3d, key=itemgetter(1))
    #coords_3d = sorted(coords_3d, key=itemgetter(0))

    return np.array(coords_3d, dtype=np.float64)

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)