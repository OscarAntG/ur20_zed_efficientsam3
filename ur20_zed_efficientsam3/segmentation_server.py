#!/usr/bin/env python3
import cv2
import numpy as np
import requests
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
        
        # FastAPI request
        mask_binary = self.call_sam3_api(cv_image, request.prompts[0], request.prompts[1])
        if mask_binary is None:
            response.success = False
            response.message = "Mask segmentation failed."
            return response
        area_pixels = cv2.countNonZero(mask_binary)
        
        colored_mask = np.zeros_like(cv_image)
        colored_mask[mask_binary > 0] = [0,0,255]
        combined_image = cv2.addWeighted(cv_image, 0.7, colored_mask, 0.3, 0)

        # Populate response
        response.success = True
        response.message = "Segmentation complete!"
        response.area = float(area_pixels)
        response.user_image = self.bridge_.cv2_to_imgmsg(combined_image, encoding="bgr8")

        self.get_logger().info(f"Sending response.")
        return response
    
    def call_sam3_api(self, cv_image, pt_pos, pt_neg):
        # Encoding RAW image to JPG for faster data transfer
        _, img_encoded = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        files = {'image': ('query.jpg', img_encoded.tobytes(), 'image/jpeg')}
        data = {'pos_x': pt_pos.x, 'pos_y': pt_pos.y, 'neg_x': pt_neg.x, 'neg_y': pt_neg.y}

        api_url = "http://localhost:8000/segment"
        try:
            self.get_logger().info(f"Sending request to {api_url}...")
            response = requests.post(api_url, files=files, data=data, timeout=10.0)

            if response.status_code == 200:
                mask_bytes = np.frombuffer(response.content, np.uint8)
                mask_decoded = cv2.imdecode(mask_bytes, cv2.IMREAD_GRAYSCALE)
                return mask_decoded
            else:
                self.get_logger().error(f"API Error {response.status_code}: {response.text}")
                return None
        except requests.exceptions.RequestException as e:
            self.get_logger().info(f"Connection failed: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationServer() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()