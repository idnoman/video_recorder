#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from camera_interfaces.srv import SetBoolString
import os


class video_recorder(Node):

    def __init__(self):
        super().__init__("video_recorder")
        self.get_logger().info("video_recorder node has been initialized")

        self.cam_image_subscriber = self.create_subscription(Image, "/cam/image", self.callback_cam_image, 20)
        self.start_recording_service = self.create_service(SetBoolString, "start_recording", self.callback_start_recording)
        self.bridge = CvBridge()
        self.video_writer = None
        self.path = ""
        self.is_recording = False

        

    def callback_cam_image(self, msg : Image):
        if self.is_recording:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv.imshow("output_video", cv_image)
            cv.waitKey(1)

            if self.video_writer is None:
                height, width, channels = cv_image.shape
                fourcc = cv.VideoWriter_fourcc(*"DIVX")
                self.video_writer = cv.VideoWriter(self.path, fourcc, 30.0, (width, height))
            
            self.video_writer.write(cv_image)

    def callback_start_recording(self, request, response):

        """A service that turns on the recording.

        You can pass a filepath indicating where to save the recording
        or pass an empty string to save it in the current location. 
        """

        if not self.is_recording:
            self.path = request.path
            self.path = os.path.join(self.path, "output_video1.avi")
            self.is_recording = True
            response.success = True
            return response
        
        else:
            response.success = False
            return response


        

        


def main(args = None):
    rclpy.init(args=args)
    node = video_recorder()
    rclpy.spin(node)
    if node.video_writer is not None:
        node.video_writer.release()

    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()