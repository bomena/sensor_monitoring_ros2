#!/usr/bin/env python3  # Use python3 for ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import String
import json

class SensorSyncChecker(Node):
    def __init__(self):
        super().__init__('sensor_sync_checker')
        # 센서 데이터 타임스탬프 저장
        self.sensor_data_timestamps = {'camera1': 0, 'camera2': 0, 'lidar': 0}
        # 동기화 상태를 전송하기 위한 퍼블리셔
        self.sync_status_publisher = self.create_publisher(String, '/sensor_sync_status', 10)
        # Subscriptions
        self.subscription_lidar = self.create_subscription(PointCloud2, "/ouster/points", self.callback_lidar, 10)
        self.subscription_camera1 = self.create_subscription(CompressedImage, "/cam1/image_color/compressed", self.callback_camera1, 10)
        self.subscription_camera2 = self.create_subscription(CompressedImage, "/cam2/image_color/compressed", self.callback_camera2, 10)
        # Timer
        self.timer = self.create_timer(5, self.check_sync)

    def callback_camera1(self, data):
        self.sensor_data_timestamps['camera1'] = self.get_clock().now().to_sec()

    def callback_camera2(self, data):
        self.sensor_data_timestamps['camera2'] = self.get_clock().now().to_sec()

    def callback_lidar(self, data):
        self.sensor_data_timestamps['lidar'] = self.get_clock().now().to_sec()

    def check_sync(self):
        timestamps = list(self.sensor_data_timestamps.values())
        max_time_diff = max(timestamps) - min(timestamps)
        # 동기화 상태와 최대 시간 차이를 JSON 형식으로 전송
        sync_data = {
            "is_synced": max_time_diff < 0.05,
            "max_time_diff": max_time_diff
        }
        self.sync_status_publisher.publish(String(data=json.dumps(sync_data)))

def main(args=None):
    rclpy.init(args=args)
    sensor_sync_checker = SensorSyncChecker()
    rclpy.spin(sensor_sync_checker)
    # Destroy the node explicitly
    sensor_sync_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    # ########################### MODIFY ##################################
    # rospy.Subscriber("/ouster/points", PointCloud2, callback_lidar)
    # rospy.Subscriber("/cam1/image_color/compressed", CompressedImage, callback_camera1)
    # rospy.Subscriber("/cam2/image_color/compressed", CompressedImage, callback_camera2)

    # # rospy.Subscriber("/os_cloud_node/points", PointCloud2, callback_lidar)
    # # rospy.Subscriber("/color1/image_color/compressed", CompressedImage, callback_camera1)
    # # rospy.Subscriber("/color2/image_color/compressed", CompressedImage, callback_camera2)
    # #####################################################################