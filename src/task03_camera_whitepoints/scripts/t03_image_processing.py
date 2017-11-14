#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from operator import itemgetter

class image_converter:

    def __init__(self):
        rospy.loginfo("Initializing image_converter...")
        self.bridge = CvBridge()
        
        #rosrun image_view image_view image:=/image_processing/greyscale_img
        self.pub_greyscale = rospy.Publisher("/image_processing/greyscale_img", Image, queue_size=1)

        # for publishing the image with unwanted white area
        self.pub_bw_orig = rospy.Publisher("/image_processing/bw_img_orig", Image, queue_size=1)

        #rosrun image_view image_view image:=/image_processing/bw_img
        self.pub_bw = rospy.Publisher("/image_processing/bw_img", Image, queue_size=1)

        #rosrun image_view image_view image:=/app/camera/rgb/image_raw
        self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw", Image, self.process_image_cb, queue_size=1)

        rospy.loginfo("image_converter initialized!")

    def process_image_cb(self,data):
        rospy.loginfo("Subscriber has recieved the image")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # taken image -> grayscale image, publish.
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.publish_grayscale_img(gray_img)
        
        # grayscale -> black and white
        # Leaves just 1 white big unwanted area in the top right corner, which we'll get rid of later
        bw_img = self.grayscale_to_blackwhite(gray_img)
        self.publish_original_bw_img(bw_img)

        # publish edited BW image
        square_centroids = self.get_centroids_of_contours(bw_img)
        self.publish_bw_img(bw_img)
        print "\n-----------------"
        print "2D/image points (centroids of white squares):\n%s" % square_centroids
        print "-----------------"
        

        object_points = self.get_measured_3d_coords()
        print "\n-----------------"
        print "3D/object points - measured:\n%s" % object_points
        print "-----------------"
        
        image_points_np = np.array(square_centroids, dtype=np.float64) # convert to nparray

        self.process_2d_3d_points(image_points_np, object_points)

    def grayscale_to_blackwhite(self, grayscale_img):
        # 200 value is high enough to eliminate most darker pixels,
        # and low enough to turn the squares to white.
        # Higher values make it harder to eliminate the top-right area,
        # and the squares start to disappear before the unnecessary white pixels
        bi_gray_max = 255
        bi_gray_min = 200

        _, bw_img = cv2.threshold(grayscale_img, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)

        return bw_img

    def publish_grayscale_img(self, grayscale_img):
        try:
            self.pub_greyscale.publish(self.bridge.cv2_to_imgmsg(grayscale_img, "mono8"))
            rospy.loginfo("Greyscale image published")
        except CvBridgeError as e:
            print(e)

    def publish_original_bw_img(self, bw_img):
        try:
            self.pub_bw_orig.publish(self.bridge.cv2_to_imgmsg(bw_img, "mono8"))
            rospy.loginfo("Original black and white image published")
        except CvBridgeError as e:
            print(e)

    def publish_bw_img(self, bw_img):
        try:
            self.pub_bw.publish(self.bridge.cv2_to_imgmsg(bw_img, "mono8"))
            rospy.loginfo("Black and white image published")
        except CvBridgeError as e:
            print(e)

    def get_centroids_of_contours(self, bw_img):
        ''' Get centroids of provided contours in a sorted order and removes unwanted areas from b-w image '''

        # finding contours in the image
        # https://docs.opencv.org/3.1.0/d4/d73/tutorial_py_contours_begin.html
        _, contours, _ = cv2.findContours(bw_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        centroids = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            #print "area: %s" % area 

            # ignoring the remaining big white area
            if area < 1000 and area > 1:
                # using moments() function to calculate centroids of squares
                # https://stackoverflow.com/questions/9074202/opencv-2-centroid
                moment = cv2.moments(cnt)
                #print "moment: %s" % moment

                cx=0;cy=0

                m00 = moment['m00']

                if m00 != 0:
                    cx = int(moment['m10'] / m00)
                    cy = int(moment['m01'] / m00)
                
                centroids.append([cx,cy])
            else:
                # get rid of unwanted white areas
                bw_img = cv2.drawContours(bw_img, [cnt], 0, (0,0,0), cv2.FILLED)

        # sorted() is stable, so 1st sort by Y-reversed (image coords are +y-down)
        centroids = sorted(centroids, key=itemgetter(1)) 
        # and then by X
        centroids = sorted(centroids, key=itemgetter(0))

        return centroids

    def get_measured_3d_coords(self):
        ''' Returns measured points in world coordinates, units are centimeters.

        Beginning of the world frame (0,0,0) is in the left bottom corner of the left bottom square.'''
        # C D
        # B E
        # A F
        #  |cam
        # z|, y/, x_
        # [0,0,0]==left bottom corner of A
        coords_3d = [
            [1, 1, 0],
            [1, 31, 0],
            [1, 61, 0],
            [23, 62, 0],
            [23, 32, 0],
            [23, 2, 0]
        ]

        return np.array(coords_3d, dtype=np.float64)

    def process_2d_3d_points(self, image_points, object_points):
        ''' Computes rotation vector, translation vector and Euler angles '''

        #print "Image points size: %s; Object points size: %s" % (len(image_points), len(object_points))

        # intrinsic parameters
        fx=614.1699; fy=614.9002 # camera focal lengths
        cx=329.9491; cy=237.2788 # optical centers in pixels
        camera_matrix = np.matrix([
            [fx,0,  cx],
            [0, fy, cy],
            [0, 0,  1]])
        #print "Intrinsic parameter (camera) matrix:\n{0}".format(camera_matrix)

        # distortion parameters
        k1 = 0.1115
        k2 = -0.1089
        t1 = 0
        t2 = 0
        dist_coeffs = np.array([k1, k2, t1, t2])
        #print "Distortion parameter matrix:\n{0}".format(dist_coeffs)
        
        #rvec=output rotation vector, tvec=output translation vector
        retval_pnp, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        print "\nRotation vector:\n%s" % rvec
        print "\nTranslation vector:\n%s" % tvec

        mat_rotation, _ = cv2.Rodrigues(rvec)
        print "Rotation matrix:\n{0}".format(mat_rotation)

        inverse_rmat = np.transpose(mat_rotation) # or np.linalg.inv
        inverse_tvec = np.multiply(np.negative(inverse_rmat), tvec)
        print "Inverse rotation matrix:\n{0}".format(inverse_rmat)
        print "Inverse translation vector:\n{0}".format(inverse_tvec)

        rm21 = inverse_rmat[2,1]
        rm22 = inverse_rmat[2,2]
        
        yaw = np.arctan2(inverse_rmat[1,0], inverse_rmat[0,0])
        pitch = np.arctan2(-inverse_rmat[2,0], math.sqrt(rm21**2 + rm22**2))
        roll = np.arctan2(rm21, rm22)
        
        print "\nYaw:\t%s rad\t(%s deg)" % (yaw, np.rad2deg(yaw))
        print "Pitch:\t%s rad\t(%s deg)" % (pitch, np.rad2deg(pitch))
        print "Roll:\t%s rad\t(%s deg)" % (roll, np.rad2deg(roll))


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