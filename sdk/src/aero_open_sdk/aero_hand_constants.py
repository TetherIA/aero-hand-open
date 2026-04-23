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
from typing import Tuple


@dataclass(frozen=True)
class AeroHandConstants:
    ## Joints (16)
    joint_names: Tuple[str, ...] = (
        "thumb_cmc_abd", "thumb_cmc_flex", "thumb_mcp", "thumb_ip", ## 4 Joints in thumb
        "index_mcp_flex", "index_pip", "index_dip",               ## 3 Joints in index
        "middle_mcp_flex", "middle_pip", "middle_dip",            ## 3 Joints in middle
        "ring_mcp_flex", "ring_pip", "ring_dip",               ## 3 Joints in ring
        "pinky_mcp_flex", "pinky_pip", "pinky_dip"                ## 3 Joints in pinky
    )

    joint_lower_limits: Tuple[float, ...] = (
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    )
    joint_upper_limits: Tuple[float, ...] = (
        100.0, 55.0, 90.0, 90.0,
        90.0, 90.0, 90.0,
        90.0, 90.0, 90.0,
        90.0, 90.0, 90.0,
        90.0, 90.0, 90.0
    )

    ## Actuations (7)
    actuation_names: Tuple[str, ...] = (
        "thumb_cmc_abd_act", "thumb_cmc_flex_act", "thumb_tendon_act",      ## 3 Actuators in thumb
        "index_tendon_act", "middle_tendon_act", "ring_tendon_act", "pinky_tendon_act" ## One actuator per finger
    )

    actuation_lower_limits: Tuple[float, ...] = (
        0.0, 0.0, -15.2789, 0.0, 0.0, 0.0, 0.0
    )
    actuation_upper_limits: Tuple[float, ...] = (
        100.0, 104.1250, 247.1500, 288.1603, 288.1603, 288.1603, 288.1603
    )