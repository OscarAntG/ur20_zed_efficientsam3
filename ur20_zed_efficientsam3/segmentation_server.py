#!/usr/bin/env python3
import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from ur20_zed_interfaces.srv import PromptUser

class SegmentationServer(Node): 
    def __init__(self):
        super().__init__("segmentation_server") 
        self.bridge_ = CvBridge()

        self.prompt_server_ = self.create_service(PromptUser, "prompt_user_service", self.perform_segmentation_callback)
        self.get_logger().info("Segmentation Server node has started!")

    def perform_segmentation_callback(self, request:PromptUser.Request, response:PromptUser.Response):
        self.get_logger().info("Received request. Processing...")
        try:
            cv_image = self.bridge_.imgmsg_to_cv2(request.user_image, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            response.success = False
            response.message = "Failed to convert input image."
            return response
        
        pt_pos = [request.prompts[0].x, request.prompts[0].y]
        pt_neg = [request.prompts[1].x, request.prompts[1].y]

        # FastAPI request

        # Test fields
        mask_binary = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        area_pixels = cv2.countNonZero(mask_binary)
        
        colored_mask = np.zeros_like(cv_image)
        colored_mask[mask_binary > 0] = [0,255,0]
        combined_image = cv2.addWeighted(cv_image, 0.7, colored_mask, 0.3, 0)

        # Populate response
        response.success = True
        response.message = "Segmentation complete!"
        response.area = float(area_pixels)
        response.user_image = self.bridge_.cv2_to_imgmsg(combined_image, encoding="bgr8")

        self.get_logger().info(f"Sending response.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationServer() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()