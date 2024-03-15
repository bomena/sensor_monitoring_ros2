#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import random
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
import time

class PointCloudToImage(Node):
    def __init__(self):
        super().__init__('point_cloud_to_image_converter')
        self.bridge = CvBridge()
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_pub1 = self.create_publisher(CompressedImage, "/converted_image1/compressed", 10)
        self.image_pub2 = self.create_publisher(CompressedImage, "/converted_image2/compressed", 10)
        self.image_pub3 = self.create_publisher(CompressedImage, "/converted_image3/compressed", 10)

        self.subscription1 = self.create_subscription(PointCloud2, "/ouster1/points", self.callback1, best_effort_qos)
        self.subscription2 = self.create_subscription(PointCloud2, "/ouster2/points", self.callback2, best_effort_qos)
        self.subscription3 = self.create_subscription(PointCloud2, "/ouster3/points", self.callback3, best_effort_qos)

        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.interval = 1


    def callback1(self, data):
        self.process_data(data, self.image_pub1)

    def callback2(self, data):
        self.process_data(data, self.image_pub2)

    def callback3(self, data):
        self.process_data(data, self.image_pub3)

    def process_data(self, data, publisher):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_time >= self.interval:
            self.last_time = current_time

            pc_generator = point_cloud2.read_points(data, field_names=("x", "y"), skip_nans=True)

            # pc_generator: 4.38690185546875e-05

            # x, y 데이터 추출 및 소수점 둘째 자리로 반올림
            points_list = [(round(point[0], 2), round(point[1], 2)) for point in pc_generator]
            
            # 유니크한 좌표만 선택
            unique_points = np.unique(points_list, axis=0)

            image = self.convert_to_image(unique_points)

            try:
                ros_image = self.bridge.cv2_to_compressed_imgmsg(image, 'jpg')
                publisher.publish(ros_image)
            except CvBridgeError as e:
                self.get_logger().error(str(e))

    def convert_to_image(self, points):
        height, width = 880, 880
        image = np.zeros((height, width, 3), np.uint8)

        fixed_max_x, fixed_min_x = 15, -15
        fixed_max_y, fixed_min_y = 15, -15

        x_scaled = (points[:, 0] - fixed_min_x) / (fixed_max_x - fixed_min_x)
        y_scaled = (points[:, 1] - fixed_min_y) / (fixed_max_y - fixed_min_y)
        x_scaled = (x_scaled * width).astype(np.int32)
        y_scaled = ((1 - y_scaled) * height).astype(np.int32)

        valid_indices = (x_scaled >= 0) & (x_scaled < width) & (y_scaled >= 0) & (y_scaled < height)

        image[y_scaled[valid_indices], x_scaled[valid_indices]] = (255, 255, 255)

        # 0.0620983921420000000 s : convert_to_image
        # 0.0022649765014648438 s : convert_to_image_new
        # 0.0005433559417724609 s : convert_to_image_new (only x, y & int)

        return image

def main(args=None):
    rclpy.init(args=args)
    point_cloud_to_image_converter = PointCloudToImage()

    try:
        rclpy.spin(point_cloud_to_image_converter)
    except KeyboardInterrupt:
        point_cloud_to_image_converter.get_logger().info('Keyboard interrupt (Ctrl+C) detected, shutting down...')
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            point_cloud_to_image_converter.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
