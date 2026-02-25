#!/usr/bin/env python3
# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from aero_hand_open_msgs.msg import JointControl, ActuatorStates, ActuatorControl

import numpy as np

from aero_open_sdk.aero_hand import AeroHand
from aero_open_sdk.aero_hand_constants import AeroHandConstants


class AeroHandNode(Node):
    def __init__(self):
        super().__init__("aero_hand_node")
        ## Parameters
        self.declare_parameter("right_port", "")
        self.declare_parameter("left_port", "")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("feedback_frequency", 100.0)
        self.declare_parameter("control_space", "joint")
        self.declare_parameter("speed", 32766) # from 0 to 32766
        self.declare_parameter("torque", 700) # from 0 to 1000

        right_port = self.get_parameter("right_port").value
        left_port = self.get_parameter("left_port").value
        baudrate = self.get_parameter("baudrate").value
        feedback_frequency = self.get_parameter("feedback_frequency").value
        control_space = self.get_parameter("control_space").value
        speed = self.get_parameter("speed").value
        torque = self.get_parameter("torque").value
        
        ## Initialize hands and subscribers/publishers based on provided ports
        if right_port not in ("", "none"):
            try:
                if right_port == "auto":
                    right_port = None  # auto detect port
                self.right_hand = AeroHand(port=right_port, baudrate=baudrate)
                for i in range(7):
                    self.right_hand.set_speed(i, speed)
                    self.right_hand.set_torque(i, torque)
            except Exception as e:
                self.get_logger().error(
                    f"Failed to initialize Right hand on port {right_port}: {e}"
                )
                self.get_logger().warn(
                    "Right hand will be unavailable. The node will continue without it."
                )

            if hasattr(self, "right_hand"):
                if control_space == "joint":
                    self.joint_states_sub_right = self.create_subscription(
                        JointControl,
                        "right/joint_control",
                        self.joint_states_right_callback,
                        10,
                    )
                elif control_space == "actuator":
                    self.actuator_control_sub_right = self.create_subscription(
                        ActuatorControl,
                        "right/actuator_control",
                        self.actuator_control_right_callback,
                        10,
                    )
                else:
                    self.get_logger().error(f"Invalid control space: {control_space}")
                    raise ValueError(
                        f'Invalid control space: {control_space}, expected "joint" or "actuator"'
                    )

                self.hand_state_pub_right = self.create_publisher(
                    ActuatorStates, "right/actuator_states", 10
                )

        if left_port not in ("", "none"):
            try:
                if left_port == "auto":
                    left_port = None  # auto detect port
                self.left_hand = AeroHand(port=left_port, baudrate=baudrate)
                for i in range(7):
                    self.left_hand.set_speed(i, speed)
                    self.left_hand.set_torque(i, torque)
            except Exception as e:
                self.get_logger().error(
                    f"Failed to initialize Left hand on port {left_port}: {e}"
                )
                self.get_logger().warn(
                    "Left hand will be unavailable. The node will continue without it."
                )

            if hasattr(self, "left_hand"):
                if control_space == "joint":
                    self.joint_states_sub_left = self.create_subscription(
                        JointControl,
                        "left/joint_control",
                        self.joint_states_left_callback,
                        10,
                    )
                elif control_space == "actuator":
                    self.actuator_control_sub_left = self.create_subscription(
                        ActuatorControl,
                        "left/actuator_control",
                        self.actuator_control_left_callback,
                        10,
                    )
                else:
                    self.get_logger().error(f"Invalid control space: {control_space}")
                    raise ValueError(
                        f'Invalid control space: {control_space}, expected "joint" or "actuator"'
                    )

                self.hand_state_pub_left = self.create_publisher(
                    ActuatorStates, "left/actuator_states", 10
                )

        ## At least one hand should be configured
        if right_port in ("", "none") and left_port in ("", "none"):
            self.get_logger().error(
                "Both right_port and left_port are disabled. Please provide at least one port."
            )
            raise ValueError(
                "Both right_port and left_port are disabled. Please provide at least one port."
            )

        ## Error if no hands were successfully initialized
        if not hasattr(self, "right_hand") and not hasattr(self, "left_hand"):
            self.get_logger().error(
                "No hands were successfully initialized. "
                "Please check your connections and port configuration."
            )
            raise RuntimeError(
                "No hands were successfully initialized. "
                "Please check your connections and port configuration."
            )

        ## Joint Limits
        self.joint_ll = AeroHandConstants.joint_lower_limits
        self.joint_ul = AeroHandConstants.joint_upper_limits
        self.actuator_ll = AeroHandConstants.actuation_lower_limits
        self.actuator_ul = AeroHandConstants.actuation_upper_limits

        self.feedback_timer = self.create_timer(
            1.0 / feedback_frequency, self.feedback_callback
        )

        self.get_logger().info("Aero hand node has been started.")

    def feedback_callback(self):
        if hasattr(self, "right_hand"):
            right_hand_state = ActuatorStates()
            right_hand_state.header.stamp = self.get_clock().now().to_msg()
            right_hand_state.side = "right"
            try:
                right_hand_state.actuations = self.right_hand.get_actuations()
                right_hand_state.actuator_speeds = self.right_hand.get_actuator_speeds()
                right_hand_state.actuator_currents = (
                    self.right_hand.get_actuator_currents()
                )
                right_hand_state.actuator_temperatures = (
                    self.right_hand.get_actuator_temperatures()
                )
            except Exception as e:
                self.get_logger().warn(f"Error getting right hand state")
                return
            self.hand_state_pub_right.publish(right_hand_state)

        if hasattr(self, "left_hand"):
            left_hand_state = ActuatorStates()
            left_hand_state.header.stamp = self.get_clock().now().to_msg()
            left_hand_state.side = "left"
            try:
                left_hand_state.actuations = self.left_hand.get_actuations()
                left_hand_state.actuator_speeds = self.left_hand.get_actuator_speeds()
                left_hand_state.actuator_currents = (
                    self.left_hand.get_actuator_currents()
                )
                left_hand_state.actuator_temperatures = (
                    self.left_hand.get_actuator_temperatures()
                )
            except Exception as e:
                self.get_logger().warn(f"Error getting left hand state. Error: {e}")
                return
            self.hand_state_pub_left.publish(left_hand_state)

    def joint_states_right_callback(self, msg: JointControl):
        if not hasattr(self, "right_hand"):
            self.get_logger().warn("Right hand is not initialized.")
            return
        if len(msg.target_positions) != 16:
            self.get_logger().warn(
                f"Expected 16 joint positions for right hand, but got {len(msg.target_positions)}."
            )
            return
        ## Clamp the joint values to the limits
        joint_values = [np.rad2deg(jv) for jv in msg.target_positions]
        joint_values = np.clip(joint_values, self.joint_ll, self.joint_ul).tolist()
        self.right_hand.set_joint_positions(joint_values)

    def joint_states_left_callback(self, msg: JointControl):
        if not hasattr(self, "left_hand"):
            self.get_logger().warn("Left hand is not initialized.")
            return
        if len(msg.target_positions) != 16:
            self.get_logger().warn(
                f"Expected 16 joint positions for left hand, but got {len(msg.target_positions)}."
            )
            return
        joint_values = [np.rad2deg(jv) for jv in msg.target_positions]
        joint_values = np.clip(joint_values, self.joint_ll, self.joint_ul).tolist()
        self.left_hand.set_joint_positions(joint_values)

    def actuator_control_right_callback(self, msg: ActuatorControl):
        if not hasattr(self, "right_hand"):
            self.get_logger().warn("Right hand is not initialized.")
            return
        if len(msg.actuation_positions) != 7:
            self.get_logger().warn(
                f"Expected 7 actuator positions for right hand, but got {len(msg.actuation_positions)}."
            )
            return
        ## Clamp the actuator values to the limits
        actuation_values = np.clip(
            msg.actuation_positions, self.actuator_ll, self.actuator_ul
        ).tolist()
        self.right_hand.set_actuations(actuation_values)

    def actuator_control_left_callback(self, msg: ActuatorControl):
        if not hasattr(self, "left_hand"):
            self.get_logger().warn("Left hand is not initialized.")
            return
        if len(msg.actuation_positions) != 7:
            self.get_logger().warn(
                f"Expected 7 actuator positions for left hand, but got {len(msg.actuation_positions)}."
            )

            return
        actuation_values = np.clip(
            msg.actuation_positions, self.actuator_ll, self.actuator_ul
        ).tolist()
        self.left_hand.set_actuations(actuation_values)


def main(args=None):
    rclpy.init(args=args)
    aero_hand_node = AeroHandNode()
    rclpy.spin(aero_hand_node)
    aero_hand_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
