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

from dataclasses import dataclass
from typing import List, Tuple

MOTOR_PULLEY_RADIUS = 9.000 # mm

## All coeffs are in mm/radian.
@dataclass
class FingerCoeffs:
    mcp_flex_coeff: float = 12.4912
    pip_coeff: float = 7.3211
    dip_coeff: float = 9.0000


@dataclass
class ThumbFlexCoeffs:
    cmc_abd_coeff: float = 2.5000
    cmc_flex_coeff: float = 12.4931


@dataclass
class ThumbIPCoeffs:
    cmc_abd_coeff: float = 2.5000
    cmc_flex_coeff: float = 2.5000
    mcp_coeff: float = 9.4372
    ip_coeff: float = 12.5000


class JointsToActuationsModel:
    """
    A model to convert joint positions to actuator movements for the Aero Hand Open.
    """

    def __init__(self) -> None:
        self.finger_coeffs = FingerCoeffs()
        self.thumb_flex_coeffs = ThumbFlexCoeffs()
        self.thumb_ip_coeffs = ThumbIPCoeffs()

    def finger_actuations(self, mcp_flex: float, pip: float, dip: float) -> float:
        ## Finger Tendon Linear Actuation Model
        ## finger_tendon = mcp_flex_coeff * mcp_flex + pip_coeff * pip + dip_coeff * dip
        return (
            self.finger_coeffs.mcp_flex_coeff * mcp_flex
            + self.finger_coeffs.pip_coeff * pip
            + self.finger_coeffs.dip_coeff * dip
        ) / MOTOR_PULLEY_RADIUS

    def thumb_actuations(
        self, cmc_abd: float, cmc_flex: float, mcp: float, ip: float
    ) -> Tuple[float, float, float]:
        ## Thumb CMC abduction and flexion are linear mappings.
        thumb_cmc_abd_actuation = cmc_abd

        ## Thumb CMC Tendon Linear Actuation Model
        ## thumb_flex_actuation = cmc_abd_coeff * cmc_abd + cmc_flex_coeff * cmc_flex
        thumb_cmc_flex_actuation = (
            self.thumb_flex_coeffs.cmc_abd_coeff * cmc_abd
            + self.thumb_flex_coeffs.cmc_flex_coeff * cmc_flex
        ) / MOTOR_PULLEY_RADIUS

        ## Thumb tendon based on linear modeling.
        ## thumb_tendon = cmc_abd_coeff * cmc_abd - cmc_flex_coeff * cmc_flex + mcp_coeff * mcp + ip_coeff * ip
        thumb_tendon_actuation = (
            self.thumb_ip_coeffs.cmc_abd_coeff * cmc_abd
            - self.thumb_ip_coeffs.cmc_flex_coeff * cmc_flex
            + self.thumb_ip_coeffs.mcp_coeff * mcp
            + self.thumb_ip_coeffs.ip_coeff * ip
        ) / MOTOR_PULLEY_RADIUS

        return thumb_cmc_abd_actuation, thumb_cmc_flex_actuation, thumb_tendon_actuation

    def hand_actuations(self, joint_positions: List[float]) -> List[float]:
        """
        Convert joint positions to actuator movements.
        Args:
            joint_positions (list): A list of 16 joint positions. (degrees)
        Returns:
            list: A list of 7 actuator movements. (degrees)
        """
        actuations = []
        actuations += self.thumb_actuations(*joint_positions[0:4])

        ## Loop over the four fingers and get the actuation values.
        for i in range(4):
            actuations.append(
                self.finger_actuations(*joint_positions[4 + i * 3 : 4 + (i + 1) * 3])
            )
        return actuations