#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup  # 이 줄을 추가하세요
import json
import threading

class SensorSyncChecker(Node):
    def __init__(self):
        super().__init__('sensor_sync_checker')
        self.sensor_data_timestamps = {'camera1': 0, 'camera2': 0, 'lidar1': 0, 'lidar2': 0, 'lidar3': 0}
        self.sync_status_publisher = self.create_publisher(String, '/sensor_sync_status', 10)
        
        self.callback_group = ReentrantCallbackGroup()  # 여기에서 사용됩니다

        # 센서별 서브스크립션 생성
        self.create_subscription(CompressedImage, "/a65/image_raw/compressed", self.callback_camera1, 10, callback_group=self.callback_group)
        self.create_subscription(CompressedImage, "/blackfly/image_raw/compressed", self.callback_camera2, 10, callback_group=self.callback_group)
        self.create_subscription(PointCloud2, "/ouster1/points", self.callback_lidar1, 10, callback_group=self.callback_group)
        self.create_subscription(PointCloud2, "/ouster2/points", self.callback_lidar2, 10, callback_group=self.callback_group)
        self.create_subscription(PointCloud2, "/ouster3/points", self.callback_lidar3, 10, callback_group=self.callback_group)

        self.timer = self.create_timer(5, self.check_sync)
        self.lock = threading.Lock()

    def callback_camera1(self, msg):
        with self.lock:
            self.sensor_data_timestamps['camera1'] = self.get_clock().now().nanoseconds / 1e9

    def callback_camera2(self, msg):
        with self.lock:
            self.sensor_data_timestamps['camera2'] = self.get_clock().now().nanoseconds / 1e9

    def callback_lidar1(self, msg):
        with self.lock:
            self.sensor_data_timestamps['lidar1'] = self.get_clock().now().nanoseconds / 1e9

    def callback_lidar2(self, msg):
        with self.lock:
            self.sensor_data_timestamps['lidar2'] = self.get_clock().now().nanoseconds / 1e9

    def callback_lidar3(self, msg):
        with self.lock:
            self.sensor_data_timestamps['lidar3'] = self.get_clock().now().nanoseconds / 1e9

    def check_sync(self):
        with self.lock:
            # 최대 시간 차이 계산
            timestamps = list(self.sensor_data_timestamps.values())
            max_time_diff = max(timestamps) - min(timestamps)
            
            # 동기화 상태와 최대 시간 차이를 JSON 형식으로 전송
            sync_data = {
                "is_synced": max_time_diff < 0.05,  # 50ms 미만이면 동기화된 것으로 간주
                "max_time_diff": max_time_diff
            }
            self.sync_status_publisher.publish(String(data=json.dumps(sync_data)))

def main(args=None):
    rclpy.init(args=args)
    sensor_sync_checker = SensorSyncChecker()

    try:
        rclpy.spin(sensor_sync_checker)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_sync_checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
