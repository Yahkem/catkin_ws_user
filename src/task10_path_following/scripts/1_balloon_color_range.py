#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from scipy import stats
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import json
import os

# Real coords in [cm]
from ColorBulb import ColorBulb

BULB_GREEN = [229, 114]
BULB_PURPLE = [229, 240]
BULB_RED = [355, 303]
BULB_BLUE = [418, 177]



class BulbColorDetector(object):
    
    def __init__(self):
        rospy.loginfo("Initializing BulbColorDetector instance...")

        self.bridge = CvBridge()

        # self.color_ranges_bgr = [
        #     ([225, 0, 0], [255, 70, 50]), # B
        #     ([0, 100, 0], [60, 200, 40]), # G
        #     ([0, 0, 130], [30, 50, 255]), # R
        #     ([200, 50, 130], [255, 100, 230]) # P
        # ]

        # BGR
        self.bulb_blue = ColorBulb("Blue", BULB_BLUE, [225, 0, 0], [255, 70, 50])
        self.bulb_green = ColorBulb("Green", BULB_GREEN, [0, 100, 0], [90, 200, 70])
        self.bulb_red = ColorBulb("Red", BULB_RED, [0, 0, 100], [90, 80, 255])
        self.bulb_purple = ColorBulb("Purple", BULB_PURPLE, [200, 50, 110], [255, 100, 230])
        
        #self.bulb_blue = ColorBulb("Blue", BULB_BLUE, [225, 0, 0], [255, 70, 50])
        #self.bulb_green = ColorBulb("Green", BULB_GREEN, [0, 100, 0], [60, 200, 40])
        #self.bulb_red = ColorBulb("Red", BULB_RED, [0, 0, 100], [70, 75, 255])
        #self.bulb_purple = ColorBulb("Purple", BULB_PURPLE, [200, 50, 110], [255, 100, 230])
        

        self.bulbs = [
            self.bulb_blue,
            self.bulb_green,
            self.bulb_red,
            self.bulb_purple
        ]

        # RAW=1..3-exposed
        # rosrun image_view image_view image:=/usb_cam/image_raw
        #self.sub_distorted = rospy.Subscriber("/image_processing/img_undistorted", Image, self.detect_bulbs, queue_size=1)
        self.sub_distorted = rospy.Subscriber("/usb_cam/image_raw", Image, self.detect_bulbs, queue_size=1)

        # Publishers of x and y image coordinates of the color bulbs
        # rostopic echo /bulb_coords/red
        self.pub_coords = rospy.Publisher("/bulb_coords", String, queue_size=1)

        # Setting up camera exposure
        os.system("v4l2-ctl --device=/dev/usb_cam --set-ctrl exposure_auto=1")
        os.system("v4l2-ctl --device=/dev/usb_cam --set-ctrl exposure_absolute=3")

        rospy.loginfo("BulbColorDetector instance initialized!")

    def detect_bulbs(self, img_msg):
        rospy.loginfo("Subscriber has recieved the image")

        cv_image = self.convert_imgmsg_to_bgr(img_msg)
        if cv_image is None: rospy.loginfo("cv_image is None! Nothing to do..."); return
 
        for bulb in self.bulbs:
            lower = np.array(bulb.lower_bound, np.uint8)
            upper = np.array(bulb.upper_bound, np.uint8)
        
            mask = cv2.inRange(cv_image, lower, upper)
            output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            non0 = cv2.findNonZero(mask) # [[[x,y]], [[x2,y2]], ... [[]]] <- WTF is this
            if non0 is None or len(non0)<15:  # lamp not found
                bulb.x_img = -1
                bulb.y_img = -1
                continue
            # print non0

            points = []
            for points_arr in non0:
                points.append(points_arr[0])

            points = np.array(points) # [[x,y], [x2,y2]...[]]
            # print points
            
            x_coords = sorted(self.get_col(points, 0))
            y_coords = sorted(self.get_col(points, 1))

            # trimmed mean in case of outliers (red vs purple)
            x_trimmed_mean = stats.trim_mean(x_coords, 0.1)
            y_trimmed_mean = stats.trim_mean(y_coords, 0.1)
            #x_trimmed_mean = self.trim_mean(x_coords, 2)
            #y_trimmed_mean = self.trim_mean(y_coords, 2)

            # print ("X=%s;Y=%s" % (x_trimmed_mean, y_trimmed_mean))

            bulb.x_img = int(round(x_trimmed_mean))
            bulb.y_img = int(round(y_trimmed_mean))

            #rospy.loginfo(bulb.img_coords_str())

            # Publish coords
            #cv2.line(cv_image, (bulb.x_img-10, bulb.y_img), (bulb.x_img+10, bulb.y_img), tuple(bulb.lower_bound), 2)
            #cv2.line(cv_image, (bulb.x_img, bulb.y_img-10), (bulb.x_img, bulb.y_img+10), tuple(bulb.lower_bound), 2)

            # UNCOMMENT in order to see the image and output from the mask
            #cv2.imshow("images", np.hstack([cv_image, output]))
            #cv2.waitKey(1)
        self.pub_coords.publish(json.dumps({'width': len(cv_image[0]), 'height': len(cv_image), 'bulbs': [bulb.save_serializable() for bulb in self.bulbs]}))

        #rospy.loginfo("Subscriber has processed the image")

    def trim_mean(self, data, m=2):
        data = np.array(data)
        d = np.abs(data - np.median(data))
        mdev = np.median(d)
        s = d/mdev if mdev else 0.
        return np.mean(data[s<m])

    def get_col(self, data, idx):
        return [row[idx] for row in data]

    def convert_imgmsg_to_bgr(self, img_msg):
        cv_image = None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        return cv_image

def main(args):
    rospy.init_node('balloon_color_range', anonymous=True)

    bulb_color_detector = BulbColorDetector() # Create BulbColorDetector object and listen to image messages

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
