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

        self.create_subscription(CompressedImage, "/a65/image_raw/compressed", self.sensor_callback, reliable_qos)
        self.create_subscription(CompressedImage, "/blackfly/image_raw/compressed", self.sensor_callback, reliable_qos)
        self.create_subscription(PointCloud2, "/ouster1/points", self.sensor_callback, best_effort_qos)
        self.create_subscription(PointCloud2, "/ouster2/points", self.sensor_callback, best_effort_qos)
        self.create_subscription(PointCloud2, "/ouster3/points", self.sensor_callback, best_effort_qos)

        self.received_data = False

    def sensor_callback(self, msg):
        self.received_data = True

    def check_sensor_sync(self):
        check_intervals = 0.09  # 90ms
        attempts = 10
        out_of_sync_count = 0

        for _ in range(attempts):
            self.received_data = False
            time.sleep(check_intervals)
            if not self.received_data:
                out_of_sync_count += 1

        sync_status = out_of_sync_count <= attempts // 2
        self.publisher.publish(Bool(data=sync_status))
        if sync_status:
            self.get_logger().info('Sensor sync is fine.')
        else:
            self.get_logger().warn('Sensor sync might be off.')

def main(args=None):
    rclpy.init(args=args)
    sensor_sync_check_node = SensorSyncCheckNode()
    sensor_sync_check_node.check_sensor_sync()
    rclpy.spin(sensor_sync_check_node)
    sensor_sync_check_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()