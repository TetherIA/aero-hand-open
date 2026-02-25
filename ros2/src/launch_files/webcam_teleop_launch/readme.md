## Webcam teleoperation

This launch file starts a local webcam-based hand pose estimator and uses it to control the Aero hand.


### Hardware requirements

- A Linux computer with a webcam
- An Aero hand connected to the computer


### Software requirements

- `mediapipe`
- `opencv-python`
- `dex_retargeting`



### Run

```bash
ros2 launch src/launch_files/webcam_teleop_launch/webcam_teleop.launch.py
```

### Common overrides (recommended)

```bash
# Set left hand port explicitly (and disable right hand)
ros2 launch src/launch_files/webcam_teleop_launch/webcam_teleop.launch.py right_hand_port:=none left_hand_port:=auto

# Tune hand connection settings
ros2 launch src/launch_files/webcam_teleop_launch/webcam_teleop.launch.py baudrate:=921600 feedback_frequency:=100.0
```