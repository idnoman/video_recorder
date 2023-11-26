#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from camera_interfaces.srv import SetBoolString
from functools import partial

class Camera_controller(Node):

    def __init__(self):
        super().__init__("camera_controller")
        self.get_logger().info("camera_controller node has been initialized")



    def call_start_recording_server(self, path):

        """Calls the start_recording server 

        You can pass a filepath indicating where to save the recording
        or pass an empty string to save it in the current location. 
        """

        start_recording_client = self.create_client(SetBoolString, "start_recording")
        while not start_recording_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server start_recording")

        request = SetBoolString.Request()
        request.path = path

        future = start_recording_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_start_recording, path = path))

    def callback_call_start_recording(self, future, path):

        """Handles the response from call_start_recording_server
        
        If the call wasn't successful logs the error """

        try:
            response = future.result()
            self.get_logger().info("Response for path:" + path + " - Success = " + str(response.success))
        except Exception as e:
            self.get_logger().error("Service call failed for path:" + path + "err: %r" % (e,))




def main(args = None):
    rclpy.init(args=args)
    camera_controller = Camera_controller()
    camera_controller.call_start_recording_server("/home/tomasz/Desktop/Filmy")
    rclpy.spin(camera_controller)
    rclpy.shutdown()

if __name__ == "__main__":
    main()