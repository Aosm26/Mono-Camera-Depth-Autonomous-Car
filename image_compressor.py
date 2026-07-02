#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/image_raw/compressed',
            10)
        self.get_logger().info("Image Compressor Node started. Subscribed to /camera/image_raw, publishing to /camera/image_raw/compressed")

    def listener_callback(self, data):
        try:
            # Parse raw image dimensions and content
            height = data.height
            width = data.width
            np_arr = np.frombuffer(data.data, dtype=np.uint8)
            
            # Convert encoding appropriately
            if data.encoding == 'rgb8':
                cv_image = np_arr.reshape((height, width, 3))
                # Convert RGB to BGR for OpenCV imencode
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            elif data.encoding == 'bgr8':
                cv_image = np_arr.reshape((height, width, 3))
            else:
                # Generic fallback if encoding differs
                cv_image = np_arr.reshape((height, width, 3))
            
            # Compress image to JPEG format (Quality: 80)
            success, encoded_image = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not success:
                self.get_logger().error("Failed to compress image")
                return

            # Instantiate and populate the CompressedImage message
            msg = CompressedImage()
            msg.header = data.header
            msg.format = 'jpeg'
            msg.data = encoded_image.tobytes()

            # Publish compressed topic
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_compressor = ImageCompressor()
    try:
        rclpy.spin(image_compressor)
    except KeyboardInterrupt:
        pass
    image_compressor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
