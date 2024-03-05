#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
import time

class PointCloudToImage(Node):
    def __init__(self):
        super().__init__('point_cloud_to_image_converter')
        self.bridge = CvBridge()
        # 세 개의 이미지 퍼블리셔 생성
        self.image_pub1 = self.create_publisher(CompressedImage, "/converted_image1/compressed", 1)
        self.image_pub2 = self.create_publisher(CompressedImage, "/converted_image2/compressed", 1)
        self.image_pub3 = self.create_publisher(CompressedImage, "/converted_image3/compressed", 1)

        # 세 개의 LiDAR 데이터 구독
        self.subscription1 = self.create_subscription(PointCloud2, "/ouster1/points", self.callback1, qos_profile_sensor_data)
        self.subscription2 = self.create_subscription(PointCloud2, "/ouster2/points", self.callback2, qos_profile_sensor_data)
        self.subscription3 = self.create_subscription(PointCloud2, "/ouster3/points", self.callback3, qos_profile_sensor_data)

        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.interval = 1  # 이미지를 전송할 시간 간격 (초)

    # 각 LiDAR 데이터 소스에 대한 콜백 함수
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

            # PointCloud2 데이터를 numpy 배열로 변환
            pc_array = self.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(list(pc_array))

            # 이미지 변환
            image = self.convert_to_image(points)

            # OpenCV 이미지를 ROS CompressedImage 메시지로 변환하여 발행
            try:
                ros_image = self.bridge.cv2_to_compressed_imgmsg(image, 'jpg')
                publisher.publish(ros_image)
            except CvBridgeError as e:
                self.get_logger().error(str(e))

    def convert_to_image(self, points):
        height = 411
        width = 880
        image = np.zeros((height, width, 3), np.uint8)

        # Define fixed scale factors based on expected range of x and y values
        fixed_max_x = 40  # Adjust based on expected max value of x
        fixed_min_x = -40  # Adjust based on expected min value of x
        fixed_max_y = 20  # Adjust based on expected max value of y
        fixed_min_y = -20  # Adjust based on expected min value of y

        for point in points:
            # Use fixed scale factors for conversion
            x_scaled = int(((point[0] - fixed_min_x) / (fixed_max_x - fixed_min_x)) * width)
            y_scaled = int(((point[1] - fixed_min_y) / (fixed_max_y - fixed_min_y)) * height)
            y_scaled = height - y_scaled  # Adjust for image coordinate system

            if 0 <= x_scaled < width and 0 <= y_scaled < height:
                image[y_scaled, x_scaled] = (255, 255, 255)  # White point

        return image

def main(args=None):
    rclpy.init(args=args)
    point_cloud_to_image_converter = PointCloudToImage()
    rclpy.spin(point_cloud_to_image_converter)
    # Destroy the node explicitly
    point_cloud_to_image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
