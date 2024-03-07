#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import os
import glob
from std_msgs.msg import String, Bool
import datetime

# rosbag record path
path = "/home/dataset"

class RosbagControlNode(Node):
    def __init__(self):
        super().__init__('rosbag_control_node')
        self.rosbag_process = None
        self.status_pub = self.create_publisher(Bool, '/rosbag_status', 10)
        self.size_pub = self.create_publisher(String, '/rosbag_size', 10)
        self.subscription = self.create_subscription(Bool, '/rosbag_record', self.rosbag_record_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def get_directory_size(self, path):
        total_size = 0
        for dirpath, dirnames, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                total_size += os.path.getsize(fp)
        return total_size / (1024 ** 3)

    def rosbag_record_callback(self, msg):
        # Path to your record.sh script
        script_path = "/home/Web/sensor_monitoring_ros2/record.sh"

        if msg.data and not self.rosbag_process:
            self.get_logger().info("Starting rosbag recording via record.sh...")
            # Ensure the script is executable and specify the correct path
            self.rosbag_process = subprocess.Popen([script_path], shell=True)
        elif msg.data and self.rosbag_process:
            self.get_logger().info("Stopping rosbag recording...")
            self.rosbag_process.terminate()  # or use .kill() if .terminate() does not work as expected
            self.rosbag_process = None

    def timer_callback(self):
        if self.rosbag_process:
            status = True
            size = self.get_directory_size("/home/dataset")
            size_msg = f"{size:.3f} GB"
        else:
            status = False
            size_msg = "0 GB"
        self.status_pub.publish(Bool(data=status))
        self.size_pub.publish(String(data=size_msg))

def main(args=None):
    rclpy.init(args=args)
    rosbag_control_node = RosbagControlNode()
    try:
        rclpy.spin(rosbag_control_node)
    finally:
        rosbag_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()