#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import sys
import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2 # Must come before cv_bridge on Jetson Ubuntu 20.04 for some reason!
from cv_bridge import CvBridge

"""
Color Encoding Transformer

Example Usage:
ROS_NAMESPACE=/numurus/dev_3dx/device_0/sensor_3dx/stereo_cam_driver/left_raw python image_color_converter.py left_color_converter image_raw_color bgr8
"""

class ColorSwapper(object):
    def __init__(self, name_topic_in, output_color_encoding):
        self.cv_bridge = CvBridge()
        rospy.loginfo("Converting Images from topic " + name_topic_in +
                      " to " + output_color_encoding)
        self.output_color_encoding = output_color_encoding
        self.ci_msg = None

        self.sub = rospy.Subscriber(
            name_topic_in, Image, self.image_cb, queue_size=5)
        self.ci_sub = rospy.Subscriber('camera_info', CameraInfo, self.ci_cb, queue_size=5)

        self.pub = rospy.Publisher(name_topic_in + '_' + self.output_color_encoding, Image, queue_size=5)
        self.ci_pub = rospy.Publisher('camera_info_' + output_color_encoding, CameraInfo, queue_size=5)

    def image_cb(self, img_msg):
        # Transform to cv2/numpy image
        img_in_cv2 = self.cv_bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='passthrough')

        # Get the color conversion code
        color_conversion_code = self.getColorConversionCode(img_msg.encoding)

        # Transform to desired output type
        if (color_conversion_code):
            new_img = cv2.cvtColor(img_in_cv2, color_conversion_code)
        else:
            new_img = img_in_cv2

        # Transform back to Image message
        new_img_msg = self.cv_bridge.cv2_to_imgmsg(
            new_img, encoding=self.output_color_encoding)
        self.pub.publish(new_img_msg)
        if self.ci_msg:
            self.ci_msg.header.stamp = new_img_msg.header.stamp
            self.ci_pub.publish(self.ci_msg)

    def ci_cb(self, ci_msg):
        self.ci_msg = ci_msg

    def getColorConversionCode(self, input_img_encoding):
        if ("bgra" in input_img_encoding):
            if ("bgr" in self.output_color_encoding) and ("bgra" not in self.output_color_encoding):
                return cv2.COLOR_BGRA2BGR

        print('Node does not yet support conversion ' + input_img_encoding + '-->' + self.output_color_encoding)
        return None

if __name__ == '__main__':
    argv = rospy.myargv(sys.argv)
    rospy.init_node(argv[1])
    cs = ColorSwapper(argv[2], argv[3])

    rospy.spin()
