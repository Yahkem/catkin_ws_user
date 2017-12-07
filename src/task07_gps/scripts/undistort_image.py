#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from scipy import stats
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# square_side_len = 2.4cm = 24mm
# rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/usb_cam/image_raw camera:=/usb_cam

# Contents of .yaml:

# image_width: 640
# image_height: 480
# camera_name: narrow_stereo
# camera_matrix:
#   rows: 3
#   cols: 3
#   data: [387.433710, 0.000000, 323.655623, 0.000000, 387.558526, 226.907571, 0.000000, 0.000000, 1.000000]
# distortion_model: plumb_bob
# distortion_coefficients:
#   rows: 1
#   cols: 5
#   data: [-0.295182, 0.063578, -0.003477, 0.004312, 0.000000]
# rectification_matrix:
#   rows: 3
#   cols: 3
#   data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
# projection_matrix:
#   rows: 3
#   cols: 4
#   data: [280.258972, 0.000000, 337.929569, 0.000000, 0.000000, 333.634064, 219.187340, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]


class ImageUndistorter(object):
    
    def __init__(self):
        rospy.loginfo("Initializing ImageUndistorter instance...")

        self.bridge = CvBridge()
        
        # rosrun image_view image_view image:=/usb_cam/image_raw
        self.sub_distorted = rospy.Subscriber("/usb_cam/image_raw", Image, self.undistort_image, queue_size=1)

        # Publisher
        # rosrun image_view image_view image:=/image_processing/img_undistorted
        self.pub_undistorted = rospy.Publisher("/image_processing/img_undistorted", Image, queue_size=1)

        rospy.loginfo("ImageUndistorter instance initialized!")

    def convert_imgmsg_to_bgr(self, img_msg):
        cv_image = None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        return cv_image

    def undistort_image(self, img_msg):
        rospy.loginfo("Subscriber has recieved the image")

        cv_image = self.convert_imgmsg_to_bgr(img_msg)
        if cv_image is None: rospy.loginfo("cv_image is None! Nothing to do..."); return

        camera_matrix = np.array([
            [387.433710, 0.000000, 323.655623], 
            [0.000000, 387.558526, 226.907571], 
            [0.000000, 0.000000, 1.000000]])

        dist_coeffs = np.array([-0.295182, 0.063578, -0.003477, 0.004312, 0.000000])

        # rect_matrix = np.array([[1.000000, 0.000000, 0.000000], [0.000000, 1.000000, 0.000000], [0.000000, 0.000000, 1.000000]])
        projection_matrix = np.array([
            [280.258972, 0.000000, 337.929569, 0.000000], 
            [0.000000, 333.634064, 219.187340, 0.000000], 
            [0.000000, 0.000000, 1.000000, 0.000000]
        ])

        # cv2.undistort(src, cameraMatrix, distCoeffs[, dst[, newCameraMatrix]]) -> dst
        undistorted_img = cv2.undistort(cv_image, camera_matrix, dist_coeffs, newCameraMatrix=projection_matrix)

        # Publish
        undistorted_img_msg = self.bridge.cv2_to_imgmsg(undistorted_img, "bgr8")
        self.pub_undistorted.publish(undistorted_img_msg)

        rospy.loginfo("Subscriber has processed the image")

        # cv2.imshow("original | undistorted", np.hstack([cv_image, undistorted_img]))
        # cv2.waitKey(0)


def main(args):
    rospy.init_node('undistort_image', anonymous=True)

    undistorter = ImageUndistorter() # create object to listen to image_raw topic and then undistort the image

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
