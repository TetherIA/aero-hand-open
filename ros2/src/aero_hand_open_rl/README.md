# Deploying the Z-Rotation Policy for Tetheria Aero Hand Open

This guide describes how to **deploy a trained Z-rotation policy** using reinforcement learning (RL), based on the task implemented in **[MuJoCo Playground](https://github.com/TetherIA/mujoco_playground_pr)** for the **Tetheria Aero Hand Open**.

---

## 🧩 Dependencies

The following repositories are required:

1. **[Aero-Hand-Open SDK](https://github.com/TetherIA/aero-open-sdk)**
2. **[Aero-Open-Firmware](https://github.com/TetherIA/aero-open-firmware)**
3. **[MuJoCo Playground](https://github.com/google-deepmind/mujoco_playground)**

---

## ⚙️ Install the Dependencies

### 1) Install Aero Hand SDK from source

Clone and install the SDK from source:

```bash
git clone https://github.com/TetherIA/aero-open-sdk.git
cd aero-open-sdk
```

Checkout the working commit:

```bash
git checkout 7247833ca5b45460b1d08591c67e6116800379cb
```

Install in editable mode:

```bash
pip install -e .
```

For detailed installation instructions, see the [SDK guide](https://github.com/TetherIA/aero-hand-open/tree/main/sdk).

---

### 2) Get the correct firmware binary

Clone the firmware repository:

```bash
git clone git@github.com:TetherIA/aero-open-firmware.git
cd aero-open-firmware
```

Checkout the correct firmware version:

```bash
git checkout 46bc858cf07f8c8858887ff11c5362b4078bc869
```

---

### 3) Install MuJoCo Playground from source

Clone from our maintained fork:

```bash
git clone git@github.com:google-deepmind/mujoco_playground.git
cd mujoco_playground
```

Then follow the [installation guide](https://github.com/google-deepmind/mujoco_playground?tab=readme-ov-file#from-source).

---

## 🚀 Run the Deployment

### 1) Flash the firmware

1. Launch the GUI:
   ```bash
   aero-hand-gui
   ```
2. Locate the serial port connected to the hand.  
3. Click the **Upload Firmware** button.  
4. In the file browser, navigate to:
   ```text
   aero-open-firmware/main/bin
   ```
   Select `firmware_v0.1.0_righthand.bin` and click **Open**.  
5. Close the GUI after flashing — keeping it open may interfere with serial communication.

---

### 2) Run the ROS 2 nodes

Enter the ROS 2 workspace:

```bash
cd aero-open-ros2
```

Build the required packages:

```bash
colcon build --select-packages aero_hand_open aero_hand_open_rl
```

Open two terminals.

**Terminal 1 — Start the communication node**
```bash
source install/setup.bash
ros2 run aero_hand_open aero_hand_node
```

**Terminal 2 — Run the policy node**
```bash
source install/setup.bash
ros2 run aero_hand_open_rl rl_z_rotation_deploy 
```

---

## 🧠 Notes

- A pretrained policy is already provided for direct deployment.  
- Ensure no other applications (e.g., GUI) are using the same serial port before running the ROS 2 nodes.
