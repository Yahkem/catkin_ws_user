#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from scipy import stats
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Real coords in [cm]
BULB_GREEN = [229, 114]
BULB_PURPLE = [229, 240]
BULB_RED = [355, 303]
BULB_BLUE = [418, 177]

class ColorBulb(object):
    def __init__(self, name, real_coords, lower_bound, upper_bound):
        self.name = name
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.x_img = -1
        self.y_img = -1
        self.x_real = real_coords[0]
        self.y_real = real_coords[1]

    def img_coords_str(self):
        return "%s bulb has image coordinates = [%s,%s]px" % (self.name, self.x_img, self.y_img)

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
        self.bulb_green = ColorBulb("Green", BULB_GREEN, [0, 100, 0], [60, 200, 40])
        self.bulb_red = ColorBulb("Red", BULB_RED, [0, 0, 130], [50, 75, 255])
        self.bulb_purple = ColorBulb("Purple", BULB_PURPLE, [200, 50, 130], [255, 100, 230])

        self.bulbs = [
            self.bulb_blue,
            self.bulb_green,
            self.bulb_red,
            self.bulb_purple
        ]

        # RAW=1..3-exposed
        # rosrun image_view image_view image:=/usb_cam/image_raw
        self.sub_distorted = rospy.Subscriber("/usb_cam/image_raw", Image, self.detect_bulbs, queue_size=1)

        # Publishers of x and y image coordinates of the color bulbs
        # rostopic echo /bulb_coords/x
        self.pub_coord_x = rospy.Publisher("/bulb_coords/x", Int32, queue_size=1)
        # rostopic echo /bulb_coords/y
        self.pub_coord_y = rospy.Publisher("/bulb_coords/y", Int32, queue_size=1)

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

            # print ("X=%s;Y=%s" % (x_trimmed_mean, y_trimmed_mean))

            bulb.x_img = int(round(x_trimmed_mean))
            bulb.y_img = int(round(y_trimmed_mean))

            rospy.loginfo(bulb.img_coords_str())

            # Publish coords
            self.pub_coord_x.publish(bulb.x_img)
            self.pub_coord_y.publish(bulb.y_img)

            # UNCOMMENT in order to see the image and output from the mask
            # cv2.imshow("images", np.hstack([cv_image, output]))
            # cv2.waitKey(0)

        rospy.loginfo("Subscriber has processed the image")

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
