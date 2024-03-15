#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import os
import shutil
from std_msgs.msg import String, Bool
import signal

# rosbag record path
path = "/home/dataset"

class RosbagControlNode(Node):
    def __init__(self):
        super().__init__('rosbag_control_node')
        self.rosbag_process = None
        self.last_size = 0  # 이전 크기 저장 변수
        self.last_update_time = None  # 마지막 업데이트 시간
        self.status_pub = self.create_publisher(Bool, '/rosbag_status', 10)
        self.size_pub = self.create_publisher(String, '/rosbag_size', 10)
        self.subscription = self.create_subscription(Bool, '/rosbag_record', self.rosbag_record_callback, 10)
        self.timer = self.create_timer(5.0, self.timer_callback)

    def get_directory_size(self, path):
        total_size = 0
        for dirpath, dirnames, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                total_size += os.path.getsize(fp)
        return total_size / (1024 ** 3)  # GB로 변환

    def rosbag_record_callback(self, msg):
        # Path to your record.sh script
        script_path = "/home/Web/sensor_monitoring_ros2/src/record.sh"

        if msg.data and not self.rosbag_process:
            self.get_logger().info("Starting rosbag recording via record.sh...")
            self.rosbag_process = subprocess.Popen(['bash', script_path], preexec_fn=os.setsid)
            self.last_update_time = rclpy.clock.Clock().now()  # 녹음 시작 시간 갱신
        elif not msg.data and self.rosbag_process:
            self.get_logger().info("Stopping rosbag recording...")
            # Send SIGINT to the process group to cleanly terminate rosbag recording
            os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)
            self.rosbag_process.wait()  # Process가 종료될 때까지 기다림
            self.rosbag_process = None

    def timer_callback(self):
        current_size = self.get_directory_size(path)  # 현재 크기
        total, used, _ = shutil.disk_usage(path)  # 디스크 사용량 확인

        # 총 공간과 사용 중인 공간을 GB 단위로 변환
        total_gb = total / (1024 ** 3)
        used_gb = used / (1024 ** 3)

        if self.rosbag_process:
            status = True
            size_msg = f"{used_gb:.0f}GB / {total_gb:.0f}GB"
            if self.last_size == current_size and (self.last_update_time and ((rclpy.clock.Clock().now() - self.last_update_time).nanoseconds / 1e9) > 10):
                self.get_logger().warn("Rosbag recording seems to be stuck.")
            else:
                if self.last_size != current_size:
                    self.last_update_time = rclpy.clock.Clock().now()
        else:
            status = False
            size_msg = "0 GB"
            self.last_size = 0  # 기록이 중지되면 이전 크기를 리셋

        self.last_size = current_size
        self.status_pub.publish(Bool(data=status))
        self.size_pub.publish(String(data=size_msg))

        try:
            if rclpy.ok():  # 컨텍스트가 여전히 유효한지 확인
                current_size = self.get_directory_size(path)
                total, used, _ = shutil.disk_usage(path)
                # 콜백 로직의 나머지 부분...
        except rclpy._rclpy_pybind11.RCLError as e:
            self.get_logger().error(f'timer_callback에서 오류 발생: {e}')


def main(args=None):
    rclpy.init(args=args)
    rosbag_control_node = RosbagControlNode()
    try:
        rclpy.spin(rosbag_control_node)
    except KeyboardInterrupt:
        rosbag_control_node.get_logger().info('Keyboard interrupt (Ctrl+C) detected, shutting down...')
    except rclpy.executors.ExternalShutdownException:
        pass  # 여기서는 특별히 처리할 필요 없음
    finally:
        # rosbag_process가 실행 중인 경우 프로세스 그룹 종료
        if rosbag_control_node.rosbag_process:
            os.killpg(os.getpgid(rosbag_control_node.rosbag_process.pid), signal.SIGTERM)
            rosbag_control_node.rosbag_process = None
        
        # 노드 및 rclpy 종료
        rosbag_control_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()