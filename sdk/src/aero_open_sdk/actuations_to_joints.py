#!/usr/bin/env python3
from typing import List, Tuple

from aero_open_sdk.joints_to_actuations import (
    MOTOR_PULLEY_RADIUS,
    FingerCoeffs,
    ThumbFlexCoeffs,
    ThumbIPCoeffs,
)
from aero_open_sdk.aero_hand_constants import AeroHandConstants


class ActuationsToJointsModelCompact:
    """
    A model to convert actuator movements to joint compact representation for the Aero Hand Open.
    Our compact joint representation is described at: https://docs.tetheria.ai/docs/sdk#compact-joint-representation
    This representation uses 3 values for the thumb (CMC abduction, CMC flexion, MCP/IP combined)
    and 1 value for each finger (MCP, PIP, DIP combined).
    Thus, the total joint positions are represented using 7 values.
    """

    def __init__(self) -> None:
        self.finger_coeffs = FingerCoeffs()
        self.thumb_flex_coeffs = ThumbFlexCoeffs()
        self.thumb_ip_coeffs = ThumbIPCoeffs()

        self.actuations_ll = AeroHandConstants.actuation_lower_limits
        self.actuations_ul = AeroHandConstants.actuation_upper_limits

        self.joints_ll = AeroHandConstants.joint_lower_limits
        self.joints_ul = AeroHandConstants.joint_upper_limits

    def finger_joints(self, finger_tendon_act: float) -> float:
        """
        Convert finger tendon actuation to joint angles.
        Args:
            finger_tendon_act (float): Finger tendon actuation. (radians)
        Returns:
            float: Joint angle for MCP, PIP, and DIP (assumed same for all). (radians)
        """
        ## Convert actuator movement to tendon movement in mm.
        tendon_movement = finger_tendon_act * MOTOR_PULLEY_RADIUS

        ## Assume same angles for all the joints in the finger.
        joint_angle = tendon_movement / (
            self.finger_coeffs.mcp_flex_coeff
            + self.finger_coeffs.pip_coeff
            + self.finger_coeffs.dip_coeff
        )
        return joint_angle

    def thumb_joints(
        self, cmc_abd_act: float, cmc_flex_act: float, thumb_tendon_act: float
    ) -> Tuple[float, float, float]:
        """
        Convert thumb actuator movements to joint angles.
        Args:
            cmc_abd_act (float): CMC abduction actuation. (radians)
            cmc_flex_act (float): CMC flexion actuation. (radians)
            thumb_tendon_act (float): Thumb tendon actuation. (radians)
        Returns:
            tuple: Joint angles for CMC abduction, CMC flexion, MCP/IP. (radians)
        """
        ## CMC abduction is a direct mapping.
        cmc_abd_joint = cmc_abd_act

        ## CMC flexion
        flex_tendon_movement = cmc_flex_act * MOTOR_PULLEY_RADIUS
        cmc_flex_joint = (
            flex_tendon_movement - self.thumb_flex_coeffs.cmc_abd_coeff * cmc_abd_joint
        ) / self.thumb_flex_coeffs.cmc_flex_coeff

        ## Thumb MCP/IP joints
        thumb_tendon_movement = thumb_tendon_act * MOTOR_PULLEY_RADIUS
        mcp_ip_joint = (
            thumb_tendon_movement
            - self.thumb_ip_coeffs.cmc_abd_coeff * cmc_abd_joint
            + self.thumb_ip_coeffs.cmc_flex_coeff * cmc_flex_joint
        ) / (self.thumb_ip_coeffs.mcp_coeff + self.thumb_ip_coeffs.ip_coeff)
        return cmc_abd_joint, cmc_flex_joint, mcp_ip_joint

    def hand_joints(self, actuations: List[float]) -> List[float]:
        """
        Convert actuator movements to joint positions.
        Args:
            actuations (list): A list of 7 actuator movements. (radians)
        Returns:
            list: A list of 7 joint positions. (radians)
        """
        joints = []
        joints += self.thumb_joints(*actuations[0:3])

        ## Loop over the four fingers and get the joint values.
        for i in range(4):
            joints.append(self.finger_joints(actuations[3 + i]))
        return joints