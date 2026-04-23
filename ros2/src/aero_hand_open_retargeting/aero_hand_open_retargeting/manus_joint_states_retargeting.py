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

import numpy as np

from manus_ros2_msgs.msg import ManusGlove
from aero_hand_open_msgs.msg import JointControl
from aero_open_sdk.aero_hand_constants import AeroHandConstants
from aero_hand_open_retargeting.utils.normalize import normalize_joint_state
from aero_hand_open_retargeting.utils.load_normalize_config import load_normalize_config


class ManusJointStatesRetargeting(Node):
    def __init__(self):
        super().__init__("manus_joint_states_retargeting")

        self.manus_glove_sub1 = self.create_subscription(
            ManusGlove, "manus_glove_0", self.glove_callback, 10
        )
        self.manus_glove_sub2 = self.create_subscription(
            ManusGlove, "manus_glove_1", self.glove_callback, 10
        )

        self.joint_states_right_pub = self.create_publisher(
            JointControl, "right/joint_control", 10
        )
        self.joint_states_left_pub = self.create_publisher(
            JointControl, "left/joint_control", 10
        )

        ## Joint Limits
        self.joint_ll_rad = np.deg2rad(AeroHandConstants.joint_lower_limits)
        self.joint_ul_rad = np.deg2rad(AeroHandConstants.joint_upper_limits)

        self.use_normalization = True
        self.normalize_config = load_normalize_config("mohit")

        self.get_logger().info("Manus Joint States Retargeting Node has been started.")

    def glove_callback(self, msg: ManusGlove):
        hand_poses = [manus_raw_node.pose for manus_raw_node in msg.raw_nodes]
        joint_values = [np.deg2rad(angle.value) for angle in msg.ergonomics]

        ## Convert values to Aero Hand conventions
        ## CMC flex
        joint_values[1] = np.deg2rad(90) - joint_values[1]
        if msg.side == "Left":
            ## Flip the abduction for left hand
            for abd_idx in [4, 8, 12, 16]:
                joint_values[abd_idx] *= -1

        ## Calculate the Thumb CMC abd and flex angles from pose data
        ## thumb cmc abd
        thumb_cmc = np.array(
            [
                hand_poses[1].position.x,
                hand_poses[1].position.y,
                hand_poses[1].position.z,
            ]
        )
        thumb_mcp = np.array(
            [
                hand_poses[2].position.x,
                hand_poses[2].position.y,
                hand_poses[2].position.z,
            ]
        )

        vec = thumb_mcp - thumb_cmc
        ## Clip the y component to avoid erroneous values
        vec[1] = min(vec[1], -1e-3)

        if msg.side == "Right":
            thumb_cmc_abd = np.arctan2(-vec[1], vec[0])
        else:
            thumb_cmc_abd = np.arctan2(-vec[1], -vec[0])
        thumb_cmc_flex = np.arctan2(vec[2], np.sqrt(vec[0] ** 2 + vec[1] ** 2))

        joint_values[0] = thumb_cmc_abd
        joint_values[1] = thumb_cmc_flex - np.deg2rad(30)

        ## Remove the abduction values as Aero Hand does not have finger abductions
        abduction_indices = [4, 8, 12, 16]
        joint_values = [
            jv for idx, jv in enumerate(joint_values) if idx not in abduction_indices
        ]

        ## Clamp the joint values to the limits
        joint_values = np.clip(
            joint_values, self.joint_ll_rad, self.joint_ul_rad
        ).tolist()

        ## Normalizing joints to account for morphological differences between human hand and robot hand
        if self.use_normalization:
            for i in range(len(joint_values)):
                joint_values[i] = normalize_joint_state(
                    joint_values[i], i, self.normalize_config
                )

        self.publish_joint_states(msg.side, joint_values)

    def publish_joint_states(self, side: str, joint_values: list):
        js_msg = JointControl()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.target_positions = joint_values
        if side == "Right":
            self.joint_states_right_pub.publish(js_msg)
        else:
            self.joint_states_left_pub.publish(js_msg)


def main():
    rclpy.init()
    manus_joint_states_retargeting = ManusJointStatesRetargeting()
    rclpy.spin(manus_joint_states_retargeting)
    rclpy.shutdown()


if __name__ == "__main__":
    main()