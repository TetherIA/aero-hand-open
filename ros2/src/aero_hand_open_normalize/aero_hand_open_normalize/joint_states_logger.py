#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aero_hand_open_msgs.msg import JointControl
import csv
from datetime import datetime
import os


class JointStatesLogger(Node):
    def __init__(self):
        super().__init__("joint_states_logger_minimal")

        # File name: joint_states_YYYYmmdd-HHMMSS.csv
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.filename = os.path.join(script_dir, f"joint_states_{timestamp}.csv")
        self.get_logger().info(f"Saving data to {self.filename}")

        # Open CSV file
        self.csv_file = open(self.filename, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        # Write header
        header = ["time_sec"] + [f"joint_{i}" for i in range(16)]
        self.writer.writerow(header)

        # Subscribe to topic
        self.subscription = self.create_subscription(
            JointControl, "right/joint_control", self.joint_state_callback, 10
        )

    def joint_state_callback(self, msg: JointControl):
        # Assume msg.position length is fixed at 16
        row = [msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9] + list(
            msg.target_positions[:16]
        )
        self.writer.writerow(row)
        self.csv_file.flush()  # Write immediately to prevent data loss

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointStatesLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user (Ctrl+C).")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

