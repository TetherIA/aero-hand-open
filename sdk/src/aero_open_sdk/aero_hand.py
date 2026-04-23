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

import os
import time
import struct
from serial import Serial, SerialTimeoutException
from typing import Iterator, List, Tuple

from aero_open_sdk.aero_hand_constants import AeroHandConstants
from aero_open_sdk.joints_to_actuations import MOTOR_PULLEY_RADIUS, JointsToActuationsModel
from aero_open_sdk.actuations_to_joints import ActuationsToJointsModelCompact

## Setup Modes
HOMING_MODE = 0x01
SET_ID_MODE = 0x02
TRIM_MODE = 0x03

## Command Modes
CTRL_POS = 0x11
CTRL_TOR = 0x12

## Request Modes
GET_ALL = 0x21
GET_POS = 0x22
GET_VEL = 0x23
GET_CURR = 0x24
GET_TEMP = 0x25

## Setting Modes
SET_SPE = 0x31
SET_TOR = 0x32

_UINT16_MAX = 65535

_RAD_TO_DEG = 180.0 / 3.141592653589793
_DEG_TO_RAD = 3.141592653589793 / 180.0

class AeroHand:
    def __init__(self, port=None, baudrate=921600):
        ## Connect to serial port
        if port is None:
            print("No port specified. Attempting to auto-detect Aero Hand serial port...")
            port = self._detect_port()
        self.ser = Serial(port, baudrate, timeout=0.01, write_timeout=0.01)

        ## Clean Buffers before starting
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        aero_hand_constants = AeroHandConstants()

        self.joint_names = aero_hand_constants.joint_names
        self.joint_lower_limits = aero_hand_constants.joint_lower_limits
        self.joint_upper_limits = aero_hand_constants.joint_upper_limits

        self.actuation_names = aero_hand_constants.actuation_names
        self.actuation_lower_limits = aero_hand_constants.actuation_lower_limits
        self.actuation_upper_limits = aero_hand_constants.actuation_upper_limits

        self.joints_to_actuations_model = JointsToActuationsModel()
        self.actuations_to_joints_model = ActuationsToJointsModelCompact()

    def _detect_port(self):

        base_path = '/dev/serial/by-id/'
        esp_32_prefix = 'usb-Espressif_USB_JTAG_serial_debug_unit_'

        if not os.path.exists(base_path):
            raise RuntimeError(
                "Could not find /dev/serial/by-id/.\n"
                "  → No serial-by-id symlinks found.\n"
                "  → Is this running on Linux? Is the Aero Hand connected?"
                "If running on Windows, please refer to the documentation to specify the port manually."
            )

        detected_ports = [d for d in os.listdir(base_path) if esp_32_prefix in d]

        if len(detected_ports) == 0:
            raise RuntimeError("No Aero Hand serial port detected. Check connection and try again.")
        elif len(detected_ports) > 1:
            raise RuntimeError("Multiple Aero Hand serial ports detected. Please specify the port manually.")
        else:
            return os.path.join(base_path, detected_ports[0])

    def create_trajectory(self, trajectory: List[Tuple[List[float], float]]) -> Iterator[List[float]]:
        rate = 100  # Hz

        def _interp_keypoints(start, end, t):
            return [start[i] + t * (end[i] - start[i]) for i in range(len(start))]

        for i in range(1, len(trajectory)):
            prev_keypoint, _ = trajectory[i - 1]
            curr_keypoint, duration = trajectory[i]

            num_steps = int(duration * rate)

            for step in range(1, num_steps + 1):
                t = step / num_steps
                yield _interp_keypoints(prev_keypoint, curr_keypoint, t)

    def run_trajectory(self, trajectory: list):
        ## Linerly interpolate between trajectory points
        interpolated_traj = self.create_trajectory(trajectory)
        for waypoint in interpolated_traj:
            self.set_joint_positions(waypoint)
            time.sleep(0.01)
        return

    def convert_seven_joints_to_sixteen(self, positions: list) -> list:
        return [
            positions[0], positions[1], positions[2], positions[2],
            positions[3], positions[3], positions[3],
            positions[4], positions[4], positions[4],
            positions[5], positions[5], positions[5],
            positions[6], positions[6], positions[6],
        ]

    def set_joint_positions(self, positions: list):
        """
        Set the joint positions of the Aero Hand.

        Args:
            positions (list): A list of 16 joint positions. (degrees)
        """
        assert len(positions) in (16, 7), "Expected 16 or 7 Joint Positions"
        if len(positions) == 7:
            positions = self.convert_seven_joints_to_sixteen(positions)
        ## Clamp the positions to the joint limits.
        positions = [
            max(
                self.joint_lower_limits[i],
                min(positions[i], self.joint_upper_limits[i]),
            )
            for i in range(16)
        ]

        ## Convert to actuations
        actuations = self.joints_to_actuations_model.hand_actuations(positions)

        ## Normalize actuation to uint16 range. (0-65535)
        actuations = [
            (actuations[i] - self.actuation_lower_limits[i])
            / (self.actuation_upper_limits[i] - self.actuation_lower_limits[i])
            * _UINT16_MAX
            for i in range(7)
        ]
        try:
            self._send_data(CTRL_POS, [int(a) for a in actuations])
        except SerialTimeoutException as e:
            print(f"Serial Timeout while sending joint positions: {e}")
            return

    def tendon_to_actuations(self, tendon_extension: float) -> float:
        """
        Convert tendon extension (mm) to actuator actuations (degrees).
        Args:
            tendon_extension (float): Tendon extension in mm.
        Returns:
            float: actuator actuations in degrees.
        """

        return (tendon_extension / MOTOR_PULLEY_RADIUS) * _RAD_TO_DEG

    def actuations_to_tendon(self, actuation: float) -> float:
        """
        Convert actuator actuations (degrees) to tendon extension (mm).
        Args:
            actuation (float): actuator actuations in degrees.
        Returns:
            float: Tendon extension in mm.
        """

        return (actuation * MOTOR_PULLEY_RADIUS) * _DEG_TO_RAD

    def set_actuations(self, actuations: list):
        """
        This function is used to set the actuations of the hand directly.
        Use this with caution as Thumb actuations are not independent i.e. setting one
        actuation requires changes in other actuations. We use the joint to
        actuations model to handle this. But this function give you direct access.
        If the actuations are not coupled correctly, it will cause Thumb tendons to
        derail.
        Args:
            actuations (list): A list of 7 actuations in degrees
            actuator actuations sequence being:
            (thumb_cmc_abd_act, thumb_cmc_flex_act, thumb_tendon, index_tendon, middle_tendon, ring_tendon, pinky_tendon)
        """
        assert len(actuations) == 7, "Expected 7 Actuations"

        ## Clamp the actuations to the limits.
        actuations = [
            max(
                self.actuation_lower_limits[i],
                min(actuations[i], self.actuation_upper_limits[i]),
            )
            for i in range(7)
        ]

        ## Normalize actuation to uint16 range. (0-65535)
        actuations = [
            (actuations[i] - self.actuation_lower_limits[i])
            / (self.actuation_upper_limits[i] - self.actuation_lower_limits[i])
            * _UINT16_MAX
            for i in range(7)
        ]

        try:
            self._send_data(CTRL_POS, [int(a) for a in actuations])
        except SerialTimeoutException as e:
            print(f"Error while writing to serial port: {e}")
            return

    def _wait_for_ack(self, opcode: int, timeout_s: float) -> bytes:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.ser.read(16)
            if len(frame) != 16:
                continue
            if frame[0] == (opcode & 0xFF) and frame[1] == 0x00:
                return frame[2:]
        raise TimeoutError(f"ACK (opcode 0x{opcode:02X}) not received within {timeout_s}s")

    def set_id(self, id: int, current_limit: int):
        """This fn is used by the GUI to set actuator IDs and current limits for the first time."""
        if not (0 <= id <= 253):
            raise ValueError("new_id must be 0..253")
        if not (0 <= current_limit <= 1023):
            raise ValueError("current_limit must be in between 0..1023")

        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        payload = [0] * 7
        payload[0] = id & 0xFF   # stored in low byte of word0
        payload[1] = current_limit & 0x03FF
        self._send_data(SET_ID_MODE, payload)
        payload = self._wait_for_ack(SET_ID_MODE, 5.0)
        old_id, new_id, cur_limit = struct.unpack_from("<HHH", payload, 0)
        return {"Old_id": old_id, "New_id": new_id, "Current_limit": cur_limit}

    def set_speed(self, id: int, speed: int):
        """
        Set the speed of a specific actuator.This speed setting is max by default when the motor moves.
        This is different from speed control mode. It only affect the dynamic of motion execution during position control.
        Args:
            id (int): Actuator ID (0..6)
            speed (int): Speed value (0..32766)
        """
        if not (0 <= id <= 6):
            raise ValueError("id must be 0..6")
        if not (0 <= speed <= 32766):
            raise ValueError("speed must be in range 0..32766")
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        payload = [0] * 7
        payload[0] = id & 0xFFFF
        payload[1] = speed & 0xFFFF
        self._send_data(SET_SPE, payload)
        payload = self._wait_for_ack(SET_SPE, 2.0)
        id, speed_val = struct.unpack_from("<HH", payload, 0)
        return {"Servo ID": id, "Speed": speed_val}

    def set_torque(self, id: int, torque: int):
        """
         Set the torque of a specific actuator. This torque setting is max by default when the motor moves.
         This is different from torque control mode. It only affect the dynamic of motion execution during position control.
         Args:
            id (int): Actuator ID (0..6)
            torque (int): Torque value (0..1000)
        """
        if not (0 <= id <= 6):
            raise ValueError("id must be 0..6")
        if not (0 <= torque <= 1000):
            raise ValueError("torque must be in range 0..1000")
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        payload = [0] * 7
        payload[0] = id & 0xFFFF
        payload[1] = torque & 0xFFFF
        self._send_data(SET_TOR, payload)
        payload = self._wait_for_ack(SET_TOR, 2.0)
        id, torque_val = struct.unpack_from("<HH", payload, 0)
        return {"Servo ID": id, "Torque": torque_val}

    def trim_servo(self, id: int, degrees: int):
        """This fn is used by the GUI to fine tune the actuator positions."""
        if not (0 <= id <= 6):
            raise ValueError("id must be 0..6")
        if not (-360 <= degrees <= 360):
            raise ValueError("degrees out of range")

        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        payload = [0] * 7
        payload[0] = id & 0xFFFF
        payload[1] = degrees & 0xFFFF
        self._send_data(TRIM_MODE, payload)
        payload = self._wait_for_ack(TRIM_MODE, 2.0)
        id, extend = struct.unpack_from("<HH", payload, 0)
        return {"Servo ID": id, "Extend Count": extend}

    def ctrl_torque(self, torque: List[int]):
        """
        Set the same torque value for all 7 servos using the CTRL_TOR command.
        Args:
            torque (list[int]): Torque values (0..1000)
        """
        if not all(0 <= t <= 1000 for t in torque):
            raise ValueError("torque must be in range 0..1000")
        payload = [t & 0xFFFF for t in torque]
        self._send_data(CTRL_TOR, payload)

    def _send_data(self, header: int, payload: List[int] = [0] * 7):
        assert self.ser is not None, "Serial port is not initialized"
        assert len(payload) == 7, "Payload must be a list of 7 integers in Range 0-65535"
        assert all(0 <= v <= 65535 for v in payload), "Payload values must be in Range 0-65535"
        msg = struct.pack("<2B7H", header & 0xFF, 0x00, *(v & 0xFFFF for v in payload))
        self.ser.write(msg)
        self.ser.flush()

    def send_homing(self, timeout_s: float = 175.0):
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._send_data(HOMING_MODE)
        payload = self._wait_for_ack(HOMING_MODE, timeout_s)
        if all(b == 0 for b in payload):
            return True
        else:
            raise ValueError(f"Unexpected HOMING payload: {payload.hex()}")

    def get_forward_kinematics(self):
        raise NotImplementedError("This method is not yet implemented")

    def get_joint_positions(self):
        raise NotImplementedError("This method is not yet implemented")

    def get_joint_positions_compact(self):
        """
        Get the joint positions from the hand in the compact 7 joint representation.
        Returns:
            list: A list of 7 joint positions. (degrees)
        """
        actuations = self.get_actuations()
        ## If there was an error getting actuations, return None
        if actuations is None:
            return None
        ## Convert to radians
        actuations = [act * _DEG_TO_RAD for act in actuations]

        ## Get Joint Positions
        joint_positions = self.actuations_to_joints_model.hand_joints(actuations)

        ## Convert to degrees
        joint_positions = [pos * _RAD_TO_DEG for pos in joint_positions]

        return joint_positions

    def get_actuations(self):
        """
        Get the actuation values from the hand.
        Returns:
            list: A list of 7 actuations. (degrees)
        """
        ## Clear input buffer to avoid stale data
        self.ser.reset_input_buffer()

        try:
            self._send_data(GET_POS)
        except SerialTimeoutException as e:
            print(f"Error while writing to serial port: {e}")
            return None

        ## Read the response
        resp = self.ser.read(2 + 7 * 2)  # 2
        if len(resp) != 16:
            print(f"Timeout while reading actuations. Got {len(resp)} bytes.")
            return None
        data = struct.unpack("<2B7H", resp)
        if data[0] != GET_POS:
            print(f"Invalid response from hand in get_actuations. Expected {GET_POS}, got {data[0]}")
            self.ser.reset_input_buffer()
            return None
        positions_uint16 = data[2:]
        ## Convert to degrees
        positions = [
            self.actuation_lower_limits[i]
            + (positions_uint16[i] / _UINT16_MAX)
            * (self.actuation_upper_limits[i] - self.actuation_lower_limits[i])
            for i in range(7)
        ]
        return positions

    def get_actuator_currents(self):
        """
        Get the actuator currents from the hand.
        Returns:
            list: A list of 7 actuator currents. (mA)
        """
        ## Clear input buffer to avoid stale data
        self.ser.reset_input_buffer()

        try:
            self._send_data(GET_CURR)
        except SerialTimeoutException as e:
            print(f"Error while writing to serial port: {e}")
            return None

        ## Read the response, signed values
        resp = self.ser.read(2 + 7 * 2)  # 2
        if len(resp) != 16:
            print(f"Timeout while reading currents. Got {len(resp)} bytes.")
            return None
        data = struct.unpack("<2B7h", resp)
        if data[0] != GET_CURR:
            print(f"Invalid response from hand in get_actuator_currents. Expected {GET_CURR}, got {data[0]}")
            self.ser.reset_input_buffer()
            return None
        ## Convert to mA using the conversion factor 1 unit = 6.5 mA as per Feetech documentation
        currents_mA = [val * 6.5 for val in data[2:]]
        return currents_mA

    def get_actuator_temperatures(self):
        """
        Get the actuator temperatures from the hand.
        Returns:
            list: A list of 7 actuator temperatures. (Degree Celsius)
        """
        self.ser.reset_input_buffer()

        try:
            self._send_data(GET_TEMP)
        except SerialTimeoutException as e:
            print(f"Error while writing to serial port: {e}")
            return None

        ## Read the response, unsigned values
        resp = self.ser.read(2 + 7 * 2)  # 2
        if len(resp) != 16:
            print(f"Timeout while reading temperatures. Got {len(resp)} bytes.")
            return None
        data = struct.unpack("<2B7H", resp)
        if data[0] != GET_TEMP:
            print(f"Invalid response from hand in get_actuator_temperatures. Expected {GET_TEMP}, got {data[0]}")
            self.ser.reset_input_buffer()
            return None
        ## Temperatures are in degree Celsius directly
        temperatures = [float(val) for val in data[2:]]
        return temperatures

    def get_actuator_speeds(self):
        """
        Get the actuator speeds from the hand.
        Returns:
            list: A list of 7 actuator speeds. (RPM)
        """
        self.ser.reset_input_buffer()

        try:
            self._send_data(GET_VEL)
        except SerialTimeoutException as e:
            print(f"Error while writing to serial port: {e}")
            return None

        ## Read the response, signed values
        resp = self.ser.read(2 + 7 * 2)  # 2
        if len(resp) != 16:
            print(f"Timeout while reading speeds. Got {len(resp)} bytes.")
            return None
        data = struct.unpack("<2B7h", resp)
        if data[0] != GET_VEL:
            print(f"Invalid response from hand in get_actuator_speeds. Expected {GET_VEL}, got {data[0]}")
            self.ser.reset_input_buffer()
            return None
        ## Convert to RPM using the conversion factor 1 unit = 0.732 RPM as per Feetech documentation
        speeds_rpm = [val * 0.732 for val in data[2:]]
        return speeds_rpm

    def close(self):
        self.ser.close()