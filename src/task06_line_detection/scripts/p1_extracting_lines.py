#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from operator import itemgetter

class LineExtractor(object):
    
    def __init__(self):
        rospy.loginfo("Initializing LineExtractor instance...")
        self.bridge = CvBridge()

        # image publishers
        # rosrun image_view image_view image:=/image_processing/img_rgb
        self.pub_rgb = rospy.Publisher("/image_processing/img_rgb", Image, queue_size=1)
        # rosrun image_view image_view image:=/image_processing/img_hsv
        self.pub_hsv = rospy.Publisher("/image_processing/img_hsv", Image, queue_size=1)
        # rosrun image_view image_view image:=/image_processing/img_ycrcb
        self.pub_ycrcb = rospy.Publisher("/image_processing/img_ycrcb", Image, queue_size=1)
        # rosrun image_view image_view image:=/image_processing/img_lines
        self.pub_lines = rospy.Publisher("/image_processing/img_lines", Image, queue_size=1)
        # rosrun image_view image_view image:=/image_processing/eroded
        self.pub_eroded = rospy.Publisher("/image_processing/eroded", Image, queue_size=1)

        # rosrun image_view image_view image:=/app/camera/rgb/image_raw
        self.sub_img = rospy.Subscriber("/app/camera/rgb/image_raw", Image, self.process_image_cb, queue_size=1)

        rospy.loginfo("LineExtractor instance initialized!")

    def process_img_as(self, cv2_constant, img, lower_bound, upper_bound):
        img_converted = cv2.cvtColor(img, cv2_constant)

        lower_bound_np = np.array(lower_bound)
        upper_bound_np = np.array(upper_bound)

        mask = cv2.inRange(img_converted, lower_bound_np, upper_bound_np)

        result = cv2.bitwise_and(img_converted, img_converted, mask=mask)

        return result

    def convert_imgmsg(self, img_msg):
        cv_image = None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        return cv_image

    def process_image_cb(self, img_msg):
        ''' Here all the magic happens '''

        rospy.loginfo("Subscriber has recieved the image")

        cv_image = self.convert_imgmsg(img_msg)

        if cv_image is None: print "cv_image is None! Nothing to do..."; return

        white = [255,255,255]
        GREY_VAL=235
        grey = [GREY_VAL, GREY_VAL, GREY_VAL]

        # sensitivity = 150
        hsv_bot = [0,0,245]
        hsv_top = [100,45,255] # Hue=<0,179> #[150,20,255]

        diff = 20
        ycrcb_bot = [240, 128-diff,128-diff]#[240, 0,0]
        ycrcb_top = [255, 128+diff,128+diff]#[255, 255,255]

        print "Bottom RGB=%s" % grey
        print "Top RGB=%s" %white
        print "Bottom HSV=%s" %hsv_bot
        print "Top HSV=%s" % hsv_top
        print "Bottom YCrCb=%s" %ycrcb_bot
        print "Top YCrCb=%s" %ycrcb_top

        img_rgb = self.process_img_as(cv2.COLOR_BGR2RGB, cv_image, grey, white)
        img_hsv = self.process_img_as(cv2.COLOR_BGR2HSV, cv_image, hsv_bot, hsv_top)
        img_ycrcb = self.process_img_as(cv2.COLOR_BGR2YCrCb, cv_image, ycrcb_bot, ycrcb_top)

        self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(img_rgb, "rgb8"))
        img_hsv_rgbspace = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2RGB)
        self.pub_hsv.publish(self.bridge.cv2_to_imgmsg(img_hsv_rgbspace, "rgb8"))
        img_ycrcb_rgbspace = cv2.cvtColor(img_ycrcb, cv2.COLOR_YCrCb2RGB)
        self.pub_ycrcb.publish(self.bridge.cv2_to_imgmsg(img_ycrcb_rgbspace, "rgb8"))

        self.find_lines(img_hsv_rgbspace)

        rospy.loginfo("Subscriber has processed the image")

    def erode_image(self, img, iters=1):
        kernel = np.ones((5,5), np.uint8)
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        erosion = cv2.erode(img, kernel, iterations=iters)
        return erosion

    def dilate_top(self, img, ratio):
        height, width, _ = img.shape

        int_height_ratio = int(height/ratio)
        img_part = img[0:int_height_ratio, 0:width] #if top else img[int_height_ratio:height, 0:width]

        kernel = np.ones((5,5), np.uint8)
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        dilation = cv2.dilate(img_part,kernel,iterations = 5)
        
        # if top:
        img[0:int_height_ratio, 0:width] = dilation
        # else:
        #     img[int_height_ratio:height, 0:width] = dilation
        # self.pub_eroded.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        return img

    def get_two_lines(self, lines):
        ''' averages the obtained lines into 2 lines, based on the angle difference '''
        l1_angles = []
        l2_angles = []

        for line in lines:
            if len(l1_angles) == 0:
                l1_angles.append(line[0]) # 1st iter
            elif abs(line[0][0]-l1_angles[0][0]) < 50: # TODO param should average to same line
                l1_angles.append(line[0]) # similar line to the 1st
            else: 
                l2_angles.append(line[0] )# the other line

        print "\n\nL1 lines=%s" % l1_angles
        print "\n\nL2 lines=%s" % l2_angles

        get_col = lambda mat,idx: [row[idx] for row in mat]
        avg_list = lambda l: reduce(lambda x,y: x+y, l) / len(l)

        l1_rho = avg_list(get_col(l1_angles, 0))
        l1_theta = avg_list(get_col(l1_angles, 1))
        l2_rho = avg_list(get_col(l2_angles, 0))
        l2_theta = avg_list(get_col(l2_angles, 1))

        print "\n---\nL1 rho=%s" % l1_rho
        print "L1 theta=%s" % l1_theta
        print "L2 rho=%s" % l2_rho
        print "L2 theta=%s\n---\n" % l2_theta

        l1 = [l1_rho, l1_theta]
        l2 = [l2_rho, l2_theta]

        return [l1, l2]
 

    def get_m_and_b(self, x1,y1, x2,y2):
        x1=float(x1); y1=float(y1); x2=float(x2); y2=float(y2)
        # y=mx+b
        m = (y2-y1) / (x2-x1) # m=Dy/Dx
        b = y1 - m*x1 # b=y-mx
        
        # assert b == (y2-m*x2) # jen pro sichr

        return (m,b)

    def find_lines(self, img):
        #https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html

        img_eroded = self.erode_image(img) # get rid of noise
        img_dilated_top = self.dilate_top(img_eroded, 1.5) # make upper part of the lines thicker
        img_eroded = self.erode_image(img_dilated_top,2) # erode again
        
        # print img_eroded.shape
        # height, width, _ = img_eroded.shape
        # eroded_1st_half = img_eroded[0:height/2, 0:width]
        img_gray = cv2.cvtColor(img_eroded, cv2.COLOR_RGB2GRAY)

        self.pub_eroded.publish(self.bridge.cv2_to_imgmsg(img_eroded, "rgb8"))
        # return

        edges = cv2.Canny(img_gray, 50, 150, apertureSize=3)

        lines = cv2.HoughLines(edges, 1, np.pi/180, 100)

        two_lines = self.get_two_lines(lines)
        # print "TwoLines=%s" % two_lines
        # return

        LINE_COLOR = (255, 0, 0) # red
        for line in two_lines:
            rho = line[0]
            theta = line[1]
            # for rho,theta in line:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))

            print_tuple = (a,b,x0,y0, x1,y1, x2,y2)
            print "Line!\na=%s; b=%s; x0=%s; y0=%s;  x1=%s; y1=%s;  x2=%s; y2=%s" % print_tuple

            cv2.line(img, (x1,y1), (x2,y2), LINE_COLOR, 2)

            m,b = self.get_m_and_b(x1,y1, x2,y2)
            print "---\nm=%s\nb=%s---" % (m,b)
            # TODO publish params

            # break
        self.pub_lines.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))


def main(args):
    rospy.init_node('line_extractor', anonymous=True)

    line_extractor = LineExtractor()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
