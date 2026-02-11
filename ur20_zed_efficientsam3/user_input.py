#!/usr/bin/env python3
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from ur20_zed_interfaces.srv import PromptUser

class UserInput(Node): 
    def __init__(self):
        super().__init__("user_input") 
        self.bridge_ = CvBridge()
        self.window_name_ = "CAMERA DISPLAY"

        self.latest_frame_ = None
        self.captured_frame_ = None
        self.display_frame_ = None
        self.control_mode_ = False
        self.prompts_ = []

        self.image_subscriber_ = self.create_subscription(Image, "/zedx/zed_node/rgb/image_rect_color", self.latest_frame_callback, 10)
        self.prompt_client_ = self.create_client(PromptUser, "prompt_user_service")
        self.input_timer_ = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("User Input node initialized!")
        self.get_logger().info("Press ` to take a snapshot.")

    def latest_frame_callback(self, msg):
        try:
            cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame_ = cv_image
        except:
            self.get_logger().info("Image conversion to cv2 format failed.")

    def timer_callback(self):
        if self.latest_frame_ is None and self.captured_frame_ is None:
            return
        
        if not self.control_mode_:
            cv2.imshow(self.window_name_, self.latest_frame_)
        else:
            cv2.imshow(self.window_name_, self.display_frame_)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('`'):
            self.toggle_control_mode()
        elif key == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def mouse_callback(self, input, x, y, flags, param):
        if input == cv2.EVENT_LBUTTONDOWN and self.control_mode_:
            if len(self.prompts_) < 2:
                self.prompts_.append((x,y))

                if len(self.prompts_) == 1:
                    print(f" (+) Positive prompt: ({x},{y})")
                    cv2.drawMarker(self.display_frame_, (x,y), (0,255,255), markerType=cv2.MARKER_STAR, markerSize=20, thickness=3)
                elif len(self.prompts_) == 2:
                    print(f" (-) Negative prompt: ({x},{y})")
                    cv2.drawMarker(self.display_frame_, (x,y), (0,0,255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=20, thickness=3)
                    self.request_prompts()

    def toggle_control_mode(self):
        if not self.control_mode_:
            if self.latest_frame_ is None:
                self.get_logger().warn("No frame received!")
                return
            self.control_mode_ = True
            self.captured_frame_ = self.latest_frame_.copy()
            self.display_frame_ = self.captured_frame_.copy()
            self.prompts_ = []

            cv2.setMouseCallback(self.window_name_, self.mouse_callback)
            self.get_logger().info("Frame captured, select prompts.")
        else:
            self.control_mode_ = False
            self.get_logger().info("Returning to live feed...")
            cv2.setMouseCallback(self.window_name_, lambda *args:None)
        
    def request_prompts(self):
        while not self.prompt_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for segmentation prompt service...")
        self.get_logger().info("Requesting segmentation...")
        req = PromptUser.Request()
        req.user_image = self.bridge_.cv2_to_imgmsg(self.captured_frame_, encoding="bgr8")

        pt_pos = Point()
        pt_pos.x = float(self.prompts_[0][0])
        pt_pos.y = float(self.prompts_[0][1])
        pt_pos.z = 0.0
        pt_neg = Point()
        pt_neg.x = float(self.prompts_[1][0])
        pt_neg.y = float(self.prompts_[1][1])
        pt_neg.z = 0.0
        
        req.prompts = [pt_pos, pt_neg]
        
        self.future_ = self.prompt_client_.call_async(req)
        self.future_.add_done_callback(self.response_user_prompt_callback)

    def response_user_prompt_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"SUCCESS! Area= {response.area:.2f} px")
                self.get_logger().info(f"Server message: {response.message}")
            else:
                self.get_logger().error(f"FAILUR: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        self.toggle_control_mode()

def main(args=None):
    rclpy.init(args=args)
    node = UserInput() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()