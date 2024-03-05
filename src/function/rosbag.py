#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import os
import glob
from std_msgs.msg import String, Bool
import datetime

# rosbag record path
path = "/home/user/Documents/monitoring"

rosbag_process = None

class RosbagControlNode(Node):
    def __init__(self):
        super().__init__('rosbag_control_node')
        self.status_pub = self.create_publisher(Bool, '/rosbag_status', 10)
        self.size_pub = self.create_publisher(String, '/rosbag_size', 10)
        self.subscription = self.create_subscription(String, '/rosbag_record', self.rosbag_record, 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def check_rosbag_status(self):
        global path
        list_of_files = glob.glob(path + '/*.bag.active')
        return True if list_of_files else False

    def get_rosbag_size(self):
        global path
        list_of_files = glob.glob(path + '/*.bag.active')
        if not list_of_files:
            return "0 GB"

        latest_file = max(list_of_files, key=os.path.getctime)
        size = os.path.getsize(latest_file)
        size_gb = size / (1024 * 1024 * 1024)
        return f"{size_gb:.3f} GB"

    def rosbag_record(self, data):
        global rosbag_process
        if data.data == "ON":
            self.get_logger().info("----------------------------ON------------------------------")
            if rosbag_process is not None:
                subprocess.call(["pkill", "-f", "rosbag record"])
                rosbag_process = None
            rosbag_process = subprocess.Popen(["/home/user/Documents/monitoring/test.sh"])
        elif data.data == "OFF":
            self.get_logger().info("----------------------------OFF------------------------------")
            if rosbag_process:
                subprocess.call(["pkill", "-f", "rosbag record"])
                rosbag_process = None

    def timer_callback(self):
        status = self.check_rosbag_status()
        size = self.get_rosbag_size()
        self.status_pub.publish(Bool(data=status))
        self.size_pub.publish(String(data=size))

def main(args=None):
    rclpy.init(args=args)
    rosbag_control_node = RosbagControlNode()
    rclpy.spin(rosbag_control_node)
    # Clean up
    rosbag_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
