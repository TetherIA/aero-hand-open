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

from setuptools import find_packages, setup

package_name = "aero_hand_open_normalize"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md"]),
    ],
    install_requires=["setuptools", "numpy", "pandas", "scipy", "pyyaml", "matplotlib"],
    zip_safe=True,
    maintainer="mohit",
    maintainer_email="mohityadav@tetheria.ai",
    description="Normalization utilities for aero hand open retargeting",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_states_logger = aero_hand_open_normalize.joint_states_logger:main",
            "find_peaks_and_valleys_joint = aero_hand_open_normalize.find_peaks_and_valleys_joint:main",
        ],
    },
)

