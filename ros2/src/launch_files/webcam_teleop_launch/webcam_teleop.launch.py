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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ## Set the ports for the right and left hands here.
    ## If you want to use only one hand, set the other port to ''
    ## To find the port for your hand on Linux, you can use the command:
    ##   ls /dev/serial/by-id/
    ## Example port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9J6J4J3-if00-port0
    # Defaults (can be overridden from CLI via launch arguments, e.g. `right_hand_port:=...`)
    default_right_hand_port = "auto"  # auto detect port
    default_left_hand_port = ""  # empty string or "none" mean no port specified, will not initialize the left hand
    default_baudrate = "921600"
    default_feedback_frequency = "100.0"  # Hz

    right_hand_port = LaunchConfiguration("right_hand_port")
    left_hand_port = LaunchConfiguration("left_hand_port")
    baudrate = LaunchConfiguration("baudrate")
    feedback_frequency = LaunchConfiguration("feedback_frequency")

    ## Nodes
    webcam_mocap = Node(
        package="webcam_mocap",
        executable="webcam_mocap",
        name="webcam_mocap",
        output="screen",
        emulate_tty=True,
    )

    dex_retargeting_node = Node(
        package="aero_hand_open_retargeting",
        executable="dex_retargeting_node",
        name="dex_retargeting_node",
        output="screen",
        emulate_tty=True,
    )

    aero_hand_node = Node(
        package="aero_hand_open",
        executable="aero_hand_node",
        name="aero_hand_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "right_port": right_hand_port,
                "left_port": left_hand_port,
                "baudrate": ParameterValue(baudrate, value_type=int),
                "feedback_frequency": ParameterValue(feedback_frequency, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "right_hand_port",
                default_value=default_right_hand_port,
                description="Serial port for right hand (or 'auto' to auto-detect).",
            ),
            DeclareLaunchArgument(
                "left_hand_port",
                default_value=default_left_hand_port,
                description="Serial port for left hand ('none' disables left hand).",
            ),
            DeclareLaunchArgument(
                "baudrate",
                default_value=default_baudrate,
                description="Serial baudrate for hand(s).",
            ),
            DeclareLaunchArgument(
                "feedback_frequency",
                default_value=default_feedback_frequency,
                description="Hand feedback frequency in Hz.",
            ),
            webcam_mocap,
            dex_retargeting_node,
            aero_hand_node,
        ]
    )
