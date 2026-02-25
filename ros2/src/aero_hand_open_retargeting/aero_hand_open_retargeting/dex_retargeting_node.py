#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from aero_hand_open_msgs.msg import HandMocap, JointControl
from ament_index_python.packages import get_package_share_directory

import os
import numpy as np
from dex_retargeting.retargeting_config import RetargetingConfig
from aero_open_sdk.aero_hand_constants import AeroHandConstants

class DexRetargetingNode(Node):
    def __init__(self):
        super().__init__("dex_retargeting_node")

        ## Declare parameters
        self.declare_parameter("input_topic", "webcam_mocap_data")
        self.declare_parameter("viz", True)
        self.declare_parameter("retargeting_method", "dexpilot")
        
        ## Get parameters
        input_topic = self.get_parameter("input_topic").value
        self.viz = self.get_parameter("viz").value
        self.retargeting_method = self.get_parameter("retargeting_method").value

        ## Subscribe to Hand Mocap data
        self.create_subscription(HandMocap, input_topic, self.pose_callback, 10)

        right_hand_config = self.make_config(self.retargeting_method, "right")
        left_hand_config = self.make_config(self.retargeting_method, "left")
        self.right_retargeter = RetargetingConfig.from_dict(right_hand_config).build()
        self.left_retargeter = RetargetingConfig.from_dict(left_hand_config).build()

        self.right_joint_names = [f"right_{name}" for name in AeroHandConstants.joint_names]
        self.left_joint_names = [f"left_{name}" for name in AeroHandConstants.joint_names]

        self.joint_control_pub_right = self.create_publisher(JointControl, "right/joint_control", 10)
        self.joint_control_pub_left = self.create_publisher(JointControl, "left/joint_control", 10)

        if self.viz:
            self.joint_states_pub_right = self.create_publisher(JointState, "right/joint_states", 10)
            self.joint_states_pub_left = self.create_publisher(JointState, "left/joint_states", 10)

            self.mediapipe_markers_pub_right = self.create_publisher(Marker, "right/mediapipe_markers", 10)
            self.mediapipe_markers_pub_left = self.create_publisher(Marker, "left/mediapipe_markers", 10)

        self.get_logger().info(f"Dex Retargeting Node started with method: {self.retargeting_method}, topic: {input_topic}")
        # TODO: use normalize_config from aero_hand_open_retargeting instead
        self.scale_factors = [
            [1.5, np.deg2rad(0)],    # Thumb CMC Abduction
            [2, np.deg2rad(-60)],    # Thumb flex tendon
            [3.5, np.deg2rad(0)],    # Thumb MCP/IP tendon
            [1.15, np.deg2rad(-10)], # Index finger tendon
            [1.15, np.deg2rad(-10)], # Middle finger tendon
            [1.15, np.deg2rad(-10)], # Ring finger tendon
            [1.2, np.deg2rad(-10)]   # Little finger tendon
        ]

    def make_config(self, retargeting_method, hand_side):
        pkg_path = get_package_share_directory("aero_hand_open_description")
        urdf_path = os.path.join(pkg_path, "urdf", f"aero_hand_open_{hand_side}.urdf")
        if retargeting_method == "position":
            return {
                "type": retargeting_method.lower(),
                "urdf_path": urdf_path,
                "target_link_names": [
                    f"{hand_side}_thumb_tip_link",
                    f"{hand_side}_index_tip_link",
                    f"{hand_side}_middle_tip_link",
                    f"{hand_side}_ring_tip_link",
                    f"{hand_side}_pinky_tip_link",
                ],
                "target_link_human_indices": [4, 9, 14, 19, 24],
                "scaling_factor": 1.2,
            }
        elif retargeting_method == "vector":
            return {
                "type": retargeting_method.lower(),
                "urdf_path": urdf_path,
                "target_origin_link_names": [f"{hand_side}_base_link"] * 5,
                "target_task_link_names": [
                    f"{hand_side}_thumb_tip_link",
                    f"{hand_side}_index_tip_link",
                    f"{hand_side}_middle_tip_link",
                    f"{hand_side}_ring_tip_link",
                    f"{hand_side}_pinky_tip_link",
                ],
                "target_link_human_indices": [
                    [0, 0, 0, 0, 0],
                    [4, 9, 14, 19, 24],
                ],
                "scaling_factor": 1.2,
                "low_pass_alpha": 0.9,
            }
        elif retargeting_method == "dexpilot":
            return {
                "type": retargeting_method.lower(),
                "urdf_path": urdf_path,
                "wrist_link_name": f"{hand_side}_base_link",
                "finger_tip_link_names": [
                    f"{hand_side}_thumb_tip_link",
                    f"{hand_side}_index_tip_link",
                    f"{hand_side}_middle_tip_link",
                    f"{hand_side}_ring_tip_link",
                    f"{hand_side}_pinky_tip_link",
                ],
                "target_link_human_indices": [
                    [9, 14, 19, 24, 14, 19, 24, 19, 24, 24, 0, 0, 0, 0, 0],
                    [4,  4,  4,  4,  9,  9,  9, 14, 14, 19, 4, 9, 14, 19, 24],
                ],
                "scaling_factor": 1.2,
                "low_pass_alpha": 0.9,
            }

    def retarget_hand(self, pose_data: np.ndarray, retargeter: RetargetingConfig) -> np.ndarray:
        retargeting_type = retargeter.optimizer.retargeting_type
        indices = retargeter.optimizer.target_link_human_indices

        if retargeting_type == "POSITION":
            reference_values = pose_data[indices, :]
        else:
            origin_indices = indices[0, :]
            task_indices = indices[1, :]
            reference_values = pose_data[task_indices, :] - pose_data[origin_indices, :]
        joint_values = retargeter.retarget(reference_values)
        return joint_values

    def publish_joint_controls(self, joint_values: list, hand_side: str):
        joint_control_msg = JointControl()
        joint_control_msg.header.stamp = self.get_clock().now().to_msg()
        joint_control_msg.target_positions = joint_values

        if hand_side == "right":
            self.joint_control_pub_right.publish(joint_control_msg)
        elif hand_side == "left":
            self.joint_control_pub_left.publish(joint_control_msg)

    def publish_joint_states(self, joint_values: list, hand_side: str):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.position = joint_values

        if hand_side == "right":
            joint_state_msg.name = self.right_joint_names
            self.joint_states_pub_right.publish(joint_state_msg)
        elif hand_side == "left":
            joint_state_msg.name = self.left_joint_names
            self.joint_states_pub_left.publish(joint_state_msg)

    def apply_scale_factors(self, joint_values):
        """Apply scale factors to joint values.

        Args:
            joint_values: List of joint values to be scaled

        Returns:
            Modified joint values with scaling applied
        """
        # Thumb cmc abd and flex joints
        joint_values[0] = joint_values[0] * self.scale_factors[0][0] + self.scale_factors[0][1]
        joint_values[1] = joint_values[1] * self.scale_factors[1][0] + self.scale_factors[1][1]

        # Thumb MCP/IP joints share the same scale factor
        for i in range(2, 4):
            joint_values[i] = joint_values[i] * self.scale_factors[2][0] + self.scale_factors[2][1]

        # Apply scale factors to finger joints (3 joints per finger)
        finger_map = [
            (range(4, 7), 3),    # Index finger
            (range(7, 10), 4),   # Middle finger
            (range(10, 13), 5),  # Ring finger
            (range(13, 16), 6),  # Pinky finger
        ]

        for joint_range, scale_idx in finger_map:
            scale, offset = self.scale_factors[scale_idx]
            for i in joint_range:
                joint_values[i] = joint_values[i] * scale + offset

        return joint_values

    def publish_mediapipe_markers(self, landmarks: np.ndarray, hand_side: str):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = f"{hand_side}_base_link"
        marker.type = Marker.SPHERE_LIST  # 👈 Sphere list, not points
        marker.action = Marker.ADD

        # Uniform sphere size
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # Bright green color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Add all Mediapipe landmark points
        for lm in landmarks:
            p = Point(x=float(lm[0]), y=float(lm[1]), z=float(lm[2]))
            marker.points.append(p)

        # Publish to appropriate topic
        if hand_side == "right":
            self.mediapipe_markers_pub_right.publish(marker)
        elif hand_side == "left":
            self.mediapipe_markers_pub_left.publish(marker)

    def pose_callback(self, msg: HandMocap):
        hand_side = msg.side
        data = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in msg.keypoints])

        ## Hacks to make DexRetarget work better
        ## 1. Move the data slightly backwards to align well with the hand base link
        data += np.array([0.0, 0.01, 0.0])

        ## 2. Move the pinky finger data UP if the finger is kind of straight
        if data[24][2] > 0.12:
            data[24][2] += 0.02

        if hand_side == "right":
            joint_values = self.retarget_hand(data, self.right_retargeter)
            ## Rearrange the joint values to match the Aero Hand joint order
            joint_values = [
                joint_values[self.right_retargeter.joint_names.index(name)]
                if name in self.right_retargeter.joint_names
                else 0.0
                for name in self.right_joint_names
            ]
        elif hand_side == "left":
            joint_values = self.retarget_hand(data, self.left_retargeter)
            ## Rearrange the joint values to match the Aero Hand joint order
            joint_values = [
                joint_values[self.left_retargeter.joint_names.index(name)]
                if name in self.left_retargeter.joint_names
                else 0.0
                for name in self.left_joint_names
            ]
        else:
            self.get_logger().error(f"Unknown hand side: {hand_side}")
            return

        # Apply scale factors to joint values
        joint_values = self.apply_scale_factors(joint_values)
        joint_values = np.clip(
            joint_values,
            np.deg2rad(AeroHandConstants.joint_lower_limits),
            np.deg2rad(AeroHandConstants.joint_upper_limits),
        ).tolist()
        self.publish_joint_controls(joint_values, hand_side)

        if self.viz:
            self.publish_joint_states(joint_values, hand_side)
            self.publish_mediapipe_markers(data, hand_side)


def main(args=None):
    rclpy.init(args=args)
    dex_retargeting_node = DexRetargetingNode()
    rclpy.spin(dex_retargeting_node)
    dex_retargeting_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
