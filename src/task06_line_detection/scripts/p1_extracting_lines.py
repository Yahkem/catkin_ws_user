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
        #rosrun image_view image_view image:=/image_processing/img_rgb
        self.pub_rgb = rospy.Publisher("/image_processing/img_rgb", Image, queue_size=1)
        #rosrun image_view image_view image:=/image_processing/img_hsv
        self.pub_hsv = rospy.Publisher("/image_processing/img_hsv", Image, queue_size=1)
        #rosrun image_view image_view image:=/image_processing/img_yuv
        self.pub_yuv = rospy.Publisher("/image_processing/img_yuv", Image, queue_size=1)

        # TODO image subscr
        #rosrun image_view image_view image:=/app/camera/rgb/image_raw
        self.sub_img = rospy.Subscriber("/app/camera/rgb/image_raw", Image, self.process_image_cb, queue_size=1)

        rospy.loginfo("LineExtractor instance initialized!")


    def process_image_cb(self, img_msg):
        rospy.loginfo("Subscriber has recieved the image")

        cv_image = None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        
        



def main(args):
    rospy.init_node('line_extractor', anonymous=True)
    # ic = image_converter()
    line_extractor = LineExtractor
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)