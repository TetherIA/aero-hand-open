# Release Notes

## 2026-02-12

### Firmware v0.1.5
- If the motors are not configured correctly during assembly by accident, e.g. if the toruqe limit is incorrectly set as 800, fix it during homing.

## 2026-02-06

### Firmware v0.1.4
- Update the overheat protection to be 70 degreee and 500 torque
- Remove auto-homing when serial is closed

## 2026-01-20

### ROS2 Release
- Apple Vision Pro based teleoperation support
- Code refactoring with improved launch files and folder structure
- Automatic port detection and support
- Fixed left hand URDF bug

### SDK Release
- Improved examples for `power_grasp` and `torque_control`

### Firmware v0.1.2
- Added overheating protection ([learn more](https://docs.tetheria.ai/docs/hardware_faq#motor-torque-and-temperature-protection-behavior))

### Firmware v0.1.3
- Reduced homing time to 4 seconds
- Automatic homing on USB connection

### Hardware Releases
- CAD files for mount adapters, including Piper arm and tripod mount adapters

---

## 2025-12-10

### ROS2 Release
- Webcam-based teleoperation support
- Automatic port recognition

### SDK Release
- Automatic port recognition

### Firmware v0.1.1
- Fixed derailment issue during torque mode
- Various bug fixes and stability improvements

---

## 2025-11-25

### Simulation & Reinforcement Learning
- Released simulation and RL tools (under `sim_rl` folder)
- Officially merged into [DeepMind MuJoCo Playground repository](https://github.com/google-deepmind/mujoco_playground/tree/main/mujoco_playground/_src/manipulation/aero_hand)

---

## 2025-11-12

### Hardware Releases
- Hand design CAD files released

---

## 2025-10-27

### ROS2 Release
- Initial ROS2 support with Manus glove teleoperation
- Hand URDF models for visualization and simulation
- AI policy deployment nodes

### SDK & Firmware Updates
- Official SDK and firmware v0.1.0 release

---

## 2025-10-13

### Initial Release
- Initial release of SDK, firmware, and documentation