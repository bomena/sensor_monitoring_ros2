import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Bool
import time

class SensorSyncCheckNode(Node):
    def __init__(self):
        super().__init__('sensor_sync_check_node')
        self.publisher = self.create_publisher(Bool, '/sensor_sync_status', 10)

        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.sensor_subscriptions = []  # Renamed from `subscriptions` to `sensor_subscriptions`
        self.last_msg_times = {}

        sensors = [
            ("/a65/image_raw/compressed", CompressedImage, reliable_qos),
            ("/blackfly/image_raw/compressed", CompressedImage, reliable_qos),
            ("/ouster1/points", PointCloud2, best_effort_qos),
            ("/ouster2/points", PointCloud2, best_effort_qos),
            ("/ouster3/points", PointCloud2, best_effort_qos),
        ]

        for topic, msg_type, qos in sensors:
            subscription = self.create_subscription(
                msg_type, topic, lambda msg, topic=topic: self.sensor_callback(msg, topic), qos
            )
            self.sensor_subscriptions.append(subscription)  # Use the new attribute name here
            self.last_msg_times[topic] = self.get_clock().now()

        self.timer = self.create_timer(1.0, self.check_sensor_sync)
        self.sync_status_history = []

    def sensor_callback(self, msg, topic):
        self.last_msg_times[topic] = self.get_clock().now()

    def check_sensor_sync(self):
        current_time = self.get_clock().now()
        time_diffs = [current_time - self.last_msg_times[topic] for topic in self.last_msg_times]

        # Consider out of sync if time difference is greater than a threshold (e.g., 100ms)
        out_of_sync = any(diff.nanoseconds > 100e6 for diff in time_diffs)

        sync_status = not out_of_sync
        self.sync_status_history.append(sync_status)
        if len(self.sync_status_history) > 10:
            self.sync_status_history.pop(0)

        # False가 7개 이상일 경우 False 발행, 아니면 True 발행
        if self.sync_status_history.count(False) >= 7:
            final_status = False
        else:
            final_status = True

        self.publisher.publish(Bool(data=final_status))

def main(args=None):
    try:
        rclpy.init(args=args)
        sensor_sync_check_node = SensorSyncCheckNode()
        rclpy.spin(sensor_sync_check_node)
    except KeyboardInterrupt:
        sensor_sync_check_node.get_logger().info('사용자에 의해 노드 종료')
    except rclpy.executors.ExternalShutdownException:
        sensor_sync_check_node.get_logger().info('외부에서 노드 종료 요청됨')
    finally:
        if rclpy.ok():
            sensor_sync_check_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
