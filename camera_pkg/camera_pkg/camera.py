#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


class Camera(Node):
    def __init__(self):
        super().__init__("Camera")
        self.publisher_video = self.create_publisher(Image, "/cam/image",10)
        timer_period = 1/30  # seconds, 1/a = where a is your frequency in Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0
        self.im_list = []
        # self.cv_image1 = cv2.imread('3.jpg')
        # self.cv_image2 = cv2.imread('2.jpg')
        self.bridge = CvBridge()
        self.cam = cv2.VideoCapture(0)

    def timer_callback(self):

        ret, image = self.cam.read()

        #### custom message
        my_msg = self.bridge.cv2_to_imgmsg(np.array(image), "bgr8")
        # my_msg.data[1] = self.bridge.cv2_to_imgmsg(np.array(self.cv_image2), "bgr8")
        #####
        self.publisher_video.publish(my_msg) ## custom message
        self.get_logger().info('You look beautiful in lens of camera <3')


def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = Camera()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()