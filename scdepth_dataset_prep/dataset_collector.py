#!/usr/bin/env python3
import os
import sys
import argparse
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class DatasetCollector(Node):
    def __init__(self, output_dir, scene_name, skip_frames):
        super().__init__('dataset_collector')
        
        self.output_dir = output_dir
        self.skip_frames = skip_frames
        self.bridge = CvBridge()
        
        # Determine scene name automatically if not provided
        if not scene_name:
            self.scene_dir = self.get_next_scene_dir()
        else:
            self.scene_dir = os.path.join(self.output_dir, 'training', scene_name)
            
        os.makedirs(self.scene_dir, exist_ok=True)
        self.get_logger().info(f"Dataset target directory: {self.scene_dir}")
        
        self.cam_txt_saved = False
        self.frame_count = 0
        self.saved_count = 0
        
        # Subscriptions
        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.get_logger().info("Dataset Collector Node initialized. Waiting for topics /camera/image_raw and /camera/camera_info...")

    def get_next_scene_dir(self):
        training_dir = os.path.join(self.output_dir, 'training')
        os.makedirs(training_dir, exist_ok=True)
        
        existing_scenes = [d for d in os.listdir(training_dir) if os.path.isdir(os.path.join(training_dir, d)) and d.startswith('scene_')]
        existing_indices = []
        for s in existing_scenes:
            try:
                idx = int(s.split('_')[1])
                existing_indices.append(idx)
            except ValueError:
                pass
                
        next_idx = max(existing_indices) + 1 if existing_indices else 0
        scene_name = f"scene_{next_idx:03d}"
        return os.path.join(training_dir, scene_name)

    def camera_info_callback(self, msg):
        if self.cam_txt_saved:
            return
            
        cam_txt_path = os.path.join(self.scene_dir, 'cam.txt')
        try:
            # CameraInfo.k is a list of 9 elements representing the 3x3 K intrinsics matrix
            K = np.array(msg.k).reshape((3, 3))
            
            # Save matrix to file with space separation
            np.savetxt(cam_txt_path, K, fmt='%.8f', delimiter=' ')
            self.cam_txt_saved = True
            self.get_logger().info(f"Camera intrinsics saved to: {cam_txt_path}")
            self.get_logger().info(f"Intrinsics Matrix K:\n{K}")
        except Exception as e:
            self.get_logger().error(f"Failed to save camera info: {e}")

    def image_callback(self, msg):
        if not self.cam_txt_saved:
            # We don't save images before saving intrinsics to keep consistency
            return
            
        self.frame_count += 1
        
        # Apply frame skipping if configured
        if self.skip_frames > 0 and (self.frame_count % (self.skip_frames + 1) != 0):
            return
            
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Filename format: 000000.jpg, 000001.jpg, ...
            filename = f"{self.saved_count:06d}.jpg"
            filepath = os.path.join(self.scene_dir, filename)
            
            # Save image
            cv2.imwrite(filepath, cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            
            self.saved_count += 1
            if self.saved_count % 50 == 0 or self.saved_count == 1:
                self.get_logger().info(f"Saved {self.saved_count} frames so far in {self.scene_dir}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to save image frame: {e}")

def main():
    parser = argparse.ArgumentParser(description="Collect Gazebo simulation frames and camera info for SC-Depth training.")
    parser.add_argument('--output_dir', type=str, default='/home/aosm/Mono Camera Depth Autonomuos Car/scdepth_dataset_prep/dataset', help='Dataset root path')
    parser.add_argument('--scene_name', type=str, default='', help='Specific scene folder name (default: auto scene_XXX)')
    parser.add_argument('--skip_frames', type=int, default=0, help='Number of frames to skip between saves (default: 0)')
    
    # Extract arguments before passing to rclpy
    args, unknown = parser.parse_known_args()
    
    # Initialize ROS2
    rclpy.init(args=sys.argv)
    
    collector = DatasetCollector(
        output_dir=args.output_dir,
        scene_name=args.scene_name,
        skip_frames=args.skip_frames
    )
    
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info(f"Killed by user. Total frames saved in this scene: {collector.saved_count}")
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
