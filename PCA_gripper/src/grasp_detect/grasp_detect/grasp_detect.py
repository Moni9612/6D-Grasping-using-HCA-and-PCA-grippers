import rclpy
from rclpy.node import Node
import os
import subprocess
import scipy.io as scio
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from PIL import Image as PILImage
import cv2
import csv

class RealSenseDataSaver(Node):
    def __init__(self):
        super().__init__('realsense_data_saver')

        # ROS 2 subscriptions
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        # CV Bridge for converting ROS2 Image messages to OpenCV images
        self.bridge = CvBridge()

        # Storage for images, camera info, and grasp pose data
        self.color_image = None
        self.depth_image = None
        self.grasp_pose_data = None  # Cache for Grasp ID == 1 data

        self.demo_run = False  # Flag to check if demo.py has run
        self.grasp_processed = False  # Flag to indicate if Grasp ID 1 has been processed

        # Path to save images, CSV, etc.
        self.save_path = '/home/moniesha/anygrasp_sdk/anygrasp_sdk/grasp_detection/example_data'
        
        # Ensure save directory exists
        os.makedirs(self.save_path, exist_ok=True)

        self.get_logger().info("RealSense Data Saver Node Initialized")

    def color_callback(self, msg):
        if self.grasp_processed:
            return  
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            color_image_path = os.path.join(self.save_path, 'color.png')
            PILImage.fromarray(cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)).save(color_image_path)
            self.get_logger().info(f"Color image saved at: {color_image_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to process color image: {e}")

    def depth_callback(self, msg):
        if self.grasp_processed:
            return  

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_image_path = os.path.join(self.save_path, 'depth.png')
            PILImage.fromarray(self.depth_image.astype(np.uint16)).save(depth_image_path)

            self.get_logger().info(f"Depth image saved at: {depth_image_path}")
            self.verify_and_run_demo()
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")
    
    def run_script_in_docker(self, container_name, script_path, args=None):
        command = ['docker', 'exec', container_name, 'python3', script_path]
        if args:
            command.extend(args)

        self.get_logger().info(f"Running command in Docker: {' '.join(command)}")

        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()

        self.get_logger().info(f"STDOUT:\n{stdout.decode()}")

        if process.returncode != 0:
            self.get_logger().error(f"Script failed with STDERR:\n{stderr.decode()}")
        else:
            self.get_logger().info("demo.py script executed successfully.")


    def verify_and_run_demo(self):
        if self.demo_run or self.grasp_processed:
            return

        color_image_path = os.path.join(self.save_path, 'color.png')
        depth_image_path = os.path.join(self.save_path, 'depth.png')

        if os.path.exists(color_image_path) and os.path.exists(depth_image_path):
            self.get_logger().info("All required files are successfully saved. Proceeding to run demo.py.")
            script_path = "/app/anygrasp_sdk/grasp_detection/demo.py"
            checkpoint_path = '/app/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar'
            container_name = "0848908cb6e9"

            try:
                self.run_script_in_docker(
                    container_name,
                    script_path,
                    args=['--checkpoint_path', checkpoint_path, '--top_down_grasp']
                )
                self.get_logger().info("demo.py script executed successfully.")
                self.demo_run = True
                self.grasp_processed = True
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Failed to run demo.py: {e}")
        else:
            self.get_logger().warning("Required files are not found. Skipping demo.py execution.")


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseDataSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down RealSense Data Saver Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

