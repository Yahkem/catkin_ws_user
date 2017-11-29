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
        rospy.loginfo("Subscriber has recieved the image")

        cv_image = self.convert_imgmsg(img_msg)

        if cv_image is None:
            print "cv_image is None! byebye..."
            return

        white = [255,255,255]
        G=235
        grey = [G,G,G]
        # white_uint8 = np.uint8([[white]])
        # grey_uint8 = np.uint8([[grey]])
        # # grey_hsv = cv2.cvtColor(grey_uint8, cv2.COLOR_BGR2HSV)
        # white_hsv = cv2.cvtColor(white_uint8, cv2.COLOR_BGR2HSV)
        # grey_hsv = cv2.cvtColor(grey_uint8, cv2.COLOR_BGR2HSV)
        # grey_hsv[0][0][2] = grey_hsv[0][0][2]-50
        # # grey_hsv[0][0][1] = grey_hsv[0][0][1]
        # white_yuv = cv2.cvtColor(white_uint8, cv2.COLOR_BGR2YUV)
        # grey_yuv = cv2.cvtColor(white_uint8, cv2.COLOR_BGR2YUV)
        # grey_yuv[0][0][0] = grey_yuv[0][0][0]-100

        # sensitivity = 150
        hsv_bot = [0,0,245]
        hsv_top = [100,45,255] # Hue=<0,179> #[150,20,255]

        diff = 20
        ycrcb_bot = [240, 128-diff,128-diff]#[240, 0,0]
        ycrcb_top = [255, 128+diff,128+diff]#[255, 255,255]

        print "Bottom HSV=%s" %hsv_bot
        print "Top HSV=%s\n-----------------" % hsv_top
        print "Bottom YCrCb=%s" %ycrcb_bot
        print "Top YCrCb=%s" %ycrcb_top

        img_rgb = self.process_img_as(cv2.COLOR_BGR2RGB, cv_image, grey, white)
        img_hsv = self.process_img_as(cv2.COLOR_BGR2HSV, cv_image, hsv_bot, hsv_top)
        img_ycrcb = self.process_img_as(cv2.COLOR_BGR2YCrCb, cv_image, ycrcb_bot, ycrcb_top)

        self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(img_rgb, "rgb8"))
        img_hsv_rgbspace = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2RGB)
        self.pub_hsv.publish(self.bridge.cv2_to_imgmsg(img_hsv_rgbspace, "rgb8"))
        self.pub_ycrcb.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(img_ycrcb, cv2.COLOR_YCrCb2RGB), "rgb8"))

        self.find_lines(img_hsv_rgbspace)

    def erode_image(self, img, iters=1):
        kernel = np.ones((5,5), np.uint8)
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        erosion = cv2.erode(img, kernel, iterations=iters)
        return erosion

    def dilate_top(self, img, ratio, top=True):
        height, width, _ = img.shape

        int_height_ratio = int(height/ratio)
        img_part = img[0:int_height_ratio, 0:width] if top else img[int_height_ratio:height, 0:width]

        kernel = np.ones((5,5), np.uint8)
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        dilation = cv2.dilate(img_part,kernel,iterations = 5)
        
        if top:
            img[0:int_height_ratio, 0:width] = dilation
        else:
            img[int_height_ratio:height, 0:width] = dilation
        # self.pub_eroded.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        return img

    def get_two_lines(self, lines):
        l1_angles = []
        l2_angles = []

        for line in lines:
            if len(l1_angles) == 0:
                l1_angles.append(line[0])
                continue
            # print 11111
            # print line[0][0]
            # print l1_angles[0][0][0]
            if abs(line[0][0]-l1_angles[0][0]) < 100: # should average to same line
                l1_angles.append(line[0])
            else: # len(l2_angles) == 0: # the other line
                l2_angles.append(line[0])

        print "\n\nL1 lines=%s" % l1_angles
        print "\n\nL2 lines=%s" % l2_angles



    def find_lines(self, img):
        #https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html

        img_eroded = self.erode_image(img)
        img_dilated_top = self.dilate_top(img_eroded, 1.5)
        # img_dilated_top = self.dilate_top(img_eroded, 3)
        # img_dilated_top = self.dilate_top(img_dilated_top, 2, False)
        # img_dilated_top = self.dilate_top(img_eroded, 4)
        img_eroded = self.erode_image(img_dilated_top,2)
        
        # print img_eroded.shape
        # height, width, _ = img_eroded.shape
        # eroded_1st_half = img_eroded[0:height/2, 0:width]
        img_gray = cv2.cvtColor(img_eroded, cv2.COLOR_RGB2GRAY)

        self.pub_eroded.publish(self.bridge.cv2_to_imgmsg(img_eroded, "rgb8"))
        # return

        edges = cv2.Canny(img_gray, 50, 150, apertureSize=3)

        lines = cv2.HoughLines(edges, 1, np.pi/180, 100)

        # TODO avgs of lines
        self.get_two_lines(lines)
        # return

        print "lines=%s" %lines
        # two_lines = lines[:2]
        # print "two_lines=%s" %two_lines
        for line in lines:
            for rho,theta in line:
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
                cv2.line(img,(x1,y1),(x2,y2),(255,0,0),2)
            # break
        self.pub_lines.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))


def main(args):
    rospy.init_node('line_extractor', anonymous=True)
    # ic = image_converter()
    line_extractor = LineExtractor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

# cvtColorFlags = ['COLOR_BAYER_BG2BGR', 'COLOR_BAYER_BG2BGR_EA', 'COLOR_BAYER_BG2BGR_VNG', 'COLOR_BAYER_BG2GRAY', 
# 'COLOR_BAYER_BG2RGB', 'COLOR_BAYER_BG2RGB_EA', 'COLOR_BAYER_BG2RGB_VNG', 'COLOR_BAYER_GB2BGR', 
# 'COLOR_BAYER_GB2BGR_EA', 'COLOR_BAYER_GB2BGR_VNG', 'COLOR_BAYER_GB2GRAY', 'COLOR_BAYER_GB2RGB', 
# 'COLOR_BAYER_GB2RGB_EA', 'COLOR_BAYER_GB2RGB_VNG', 'COLOR_BAYER_GR2BGR', 'COLOR_BAYER_GR2BGR_EA', 
# 'COLOR_BAYER_GR2BGR_VNG', 'COLOR_BAYER_GR2GRAY', 'COLOR_BAYER_GR2RGB', 'COLOR_BAYER_GR2RGB_EA', 
# 'COLOR_BAYER_GR2RGB_VNG', 'COLOR_BAYER_RG2BGR', 'COLOR_BAYER_RG2BGR_EA', 'COLOR_BAYER_RG2BGR_VNG', 
# 'COLOR_BAYER_RG2GRAY', 'COLOR_BAYER_RG2RGB', 'COLOR_BAYER_RG2RGB_EA', 'COLOR_BAYER_RG2RGB_VNG', 
# 'COLOR_BGR2BGR555', 'COLOR_BGR2BGR565', 'COLOR_BGR2BGRA', 'COLOR_BGR2GRAY', 'COLOR_BGR2HLS', 
# 'COLOR_BGR2HLS_FULL', 'COLOR_BGR2HSV', 'COLOR_BGR2HSV_FULL', 'COLOR_BGR2LAB', 'COLOR_BGR2LUV', 
# 'COLOR_BGR2Lab', 'COLOR_BGR2Luv', 'COLOR_BGR2RGB', 'COLOR_BGR2RGBA', 'COLOR_BGR2XYZ', 'COLOR_BGR2YCR_CB', 
# 'COLOR_BGR2YCrCb', 'COLOR_BGR2YUV', 'COLOR_BGR2YUV_I420', 'COLOR_BGR2YUV_IYUV', 'COLOR_BGR2YUV_YV12', 'COLOR_BGR5552BGR', 
# 'COLOR_BGR5552BGRA', 'COLOR_BGR5552GRAY', 'COLOR_BGR5552RGB', 'COLOR_BGR5552RGBA', 'COLOR_BGR5652BGR', 'COLOR_BGR5652BGRA', 'COLOR_BGR5652GRAY', 'COLOR_BGR5652RGB', 'COLOR_BGR5652RGBA', 'COLOR_BGRA2BGR', 'COLOR_BGRA2BGR555', 'COLOR_BGRA2BGR565', 'COLOR_BGRA2GRAY', 'COLOR_BGRA2RGB', 'COLOR_BGRA2RGBA', 'COLOR_BGRA2YUV_I420', 'COLOR_BGRA2YUV_IYUV', 'COLOR_BGRA2YUV_YV12','COLOR_BayerBG2BGR', 'COLOR_BayerBG2BGR_EA', 'COLOR_BayerBG2BGR_VNG', 'COLOR_BayerBG2GRAY', 'COLOR_BayerBG2RGB', 
# 'COLOR_BayerBG2RGB_EA', 'COLOR_BayerBG2RGB_VNG', 'COLOR_BayerGB2BGR','COLOR_BayerGB2BGR_EA', 'COLOR_BayerGB2BGR_VNG', 'COLOR_BayerGB2GRAY', 'COLOR_BayerGB2RGB', 'COLOR_BayerGB2RGB_EA', 'COLOR_BayerGB2RGB_VNG', 'COLOR_BayerGR2BGR', 'COLOR_BayerGR2BGR_EA', 'COLOR_BayerGR2BGR_VNG', 'COLOR_BayerGR2GRAY', 'COLOR_BayerGR2RGB', 'COLOR_BayerGR2RGB_EA', 'COLOR_BayerGR2RGB_VNG', 'COLOR_BayerRG2BGR', 'COLOR_BayerRG2BGR_EA', 'COLOR_BayerRG2BGR_VNG', 'COLOR_BayerRG2GRAY', 
# 'COLOR_BayerRG2RGB', 'COLOR_BayerRG2RGB_EA', 'COLOR_BayerRG2RGB_VNG', 'COLOR_COLORCVT_MAX', 'COLOR_GRAY2BGR', 'COLOR_GRAY2BGR555', 'COLOR_GRAY2BGR565', 'COLOR_GRAY2BGRA', 'COLOR_GRAY2RGB', 'COLOR_GRAY2RGBA', 'COLOR_HLS2BGR', 'COLOR_HLS2BGR_FULL', 'COLOR_HLS2RGB', 'COLOR_HLS2RGB_FULL', 'COLOR_HSV2BGR', 'COLOR_HSV2BGR_FULL', 'COLOR_HSV2RGB', 'COLOR_HSV2RGB_FULL', 'COLOR_LAB2BGR', 'COLOR_LAB2LBGR', 'COLOR_LAB2LRGB', 'COLOR_LAB2RGB', 'COLOR_LBGR2LAB', 'COLOR_LBGR2LUV', 'COLOR_LBGR2Lab', 'COLOR_LBGR2Luv', 'COLOR_LRGB2LAB', 'COLOR_LRGB2LUV', 'COLOR_LRGB2Lab', 'COLOR_LRGB2Luv', 'COLOR_LUV2BGR', 'COLOR_LUV2LBGR', 'COLOR_LUV2LRGB', 'COLOR_LUV2RGB', 'COLOR_Lab2BGR', 'COLOR_Lab2LBGR', 'COLOR_Lab2LRGB', 'COLOR_Lab2RGB', 'COLOR_Luv2BGR', 'COLOR_Luv2LBGR', 'COLOR_Luv2LRGB', 'COLOR_Luv2RGB', 'COLOR_M_RGBA2RGBA', 'COLOR_RGB2BGR', 'COLOR_RGB2BGR555', 'COLOR_RGB2BGR565', 'COLOR_RGB2BGRA', 'COLOR_RGB2GRAY', 'COLOR_RGB2HLS', 'COLOR_RGB2HLS_FULL', 
# 'COLOR_RGB2HSV', 'COLOR_RGB2HSV_FULL', 'COLOR_RGB2LAB', 'COLOR_RGB2LUV', 'COLOR_RGB2Lab', 'COLOR_RGB2Luv', 'COLOR_RGB2RGBA', 'COLOR_RGB2XYZ', 'COLOR_RGB2YCR_CB', 'COLOR_RGB2YCrCb', 'COLOR_RGB2YUV', 'COLOR_RGB2YUV_I420', 'COLOR_RGB2YUV_IYUV', 'COLOR_RGB2YUV_YV12', 'COLOR_RGBA2BGR', 'COLOR_RGBA2BGR555', 'COLOR_RGBA2BGR565', 'COLOR_RGBA2BGRA', 'COLOR_RGBA2GRAY', 'COLOR_RGBA2M_RGBA', 'COLOR_RGBA2RGB', 'COLOR_RGBA2YUV_I420', 'COLOR_RGBA2YUV_IYUV', 
# 'COLOR_RGBA2YUV_YV12', 'COLOR_RGBA2mRGBA', 'COLOR_XYZ2BGR', 'COLOR_XYZ2RGB', 'COLOR_YCR_CB2BGR', 'COLOR_YCR_CB2RGB', 'COLOR_YCrCb2BGR', 'COLOR_YCrCb2RGB', 'COLOR_YUV2BGR', 'COLOR_YUV2BGRA_I420', 'COLOR_YUV2BGRA_IYUV', 'COLOR_YUV2BGRA_NV12', 'COLOR_YUV2BGRA_NV21', 'COLOR_YUV2BGRA_UYNV', 'COLOR_YUV2BGRA_UYVY', 'COLOR_YUV2BGRA_Y422', 'COLOR_YUV2BGRA_YUNV', 'COLOR_YUV2BGRA_YUY2', 'COLOR_YUV2BGRA_YUYV', 'COLOR_YUV2BGRA_YV12', 'COLOR_YUV2BGRA_YVYU', 'COLOR_YUV2BGR_I420', 'COLOR_YUV2BGR_IYUV', 'COLOR_YUV2BGR_NV12', 'COLOR_YUV2BGR_NV21', 'COLOR_YUV2BGR_UYNV', 'COLOR_YUV2BGR_UYVY', 'COLOR_YUV2BGR_Y422', 'COLOR_YUV2BGR_YUNV', 'COLOR_YUV2BGR_YUY2', 'COLOR_YUV2BGR_YUYV', 'COLOR_YUV2BGR_YV12', 'COLOR_YUV2BGR_YVYU', 'COLOR_YUV2GRAY_420', 'COLOR_YUV2GRAY_I420', 'COLOR_YUV2GRAY_IYUV', 'COLOR_YUV2GRAY_NV12', 'COLOR_YUV2GRAY_NV21', 
# 'COLOR_YUV2GRAY_UYNV', 'COLOR_YUV2GRAY_UYVY', 'COLOR_YUV2GRAY_Y422', 'COLOR_YUV2GRAY_YUNV', 'COLOR_YUV2GRAY_YUY2', 'COLOR_YUV2GRAY_YUYV', 'COLOR_YUV2GRAY_YV12', 'COLOR_YUV2GRAY_YVYU', 'COLOR_YUV2RGB', 'COLOR_YUV2RGBA_I420', 'COLOR_YUV2RGBA_IYUV', 'COLOR_YUV2RGBA_NV12', 'COLOR_YUV2RGBA_NV21', 'COLOR_YUV2RGBA_UYNV', 'COLOR_YUV2RGBA_UYVY', 'COLOR_YUV2RGBA_Y422', 'COLOR_YUV2RGBA_YUNV', 'COLOR_YUV2RGBA_YUY2', 'COLOR_YUV2RGBA_YUYV', 'COLOR_YUV2RGBA_YV12', 'COLOR_YUV2RGBA_YVYU', 'COLOR_YUV2RGB_I420', 'COLOR_YUV2RGB_IYUV', 'COLOR_YUV2RGB_NV12', 'COLOR_YUV2RGB_NV21', 'COLOR_YUV2RGB_UYNV', 'COLOR_YUV2RGB_UYVY', 'COLOR_YUV2RGB_Y422', 'COLOR_YUV2RGB_YUNV', 'COLOR_YUV2RGB_YUY2', 'COLOR_YUV2RGB_YUYV', 'COLOR_YUV2RGB_YV12', 'COLOR_YUV2RGB_YVYU', 'COLOR_YUV420P2BGR', 'COLOR_YUV420P2BGRA', 'COLOR_YUV420P2GRAY',
# 'COLOR_YUV420P2RGB', 'COLOR_YUV420P2RGBA', 'COLOR_YUV420SP2BGR', 'COLOR_YUV420SP2BGRA', 'COLOR_YUV420SP2GRAY', 'COLOR_YUV420SP2RGB', 'COLOR_YUV420SP2RGBA', 'COLOR_YUV420p2BGR', 'COLOR_YUV420p2BGRA', 'COLOR_YUV420p2GRAY', 'COLOR_YUV420p2RGB', 'COLOR_YUV420p2RGBA', 'COLOR_YUV420sp2BGR', 'COLOR_YUV420sp2BGRA', 'COLOR_YUV420sp2GRAY', 'COLOR_YUV420sp2RGB', 'COLOR_YUV420sp2RGBA', 'COLOR_mRGBA2RGBA']        
