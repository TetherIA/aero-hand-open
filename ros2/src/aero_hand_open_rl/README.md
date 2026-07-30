# Deploying the Z-Rotation Policy for Tetheria Aero Hand Open

This guide describes how to **deploy a trained Z-rotation policy** using reinforcement learning (RL), based on the task implemented in **[MuJoCo Playground](https://github.com/TetherIA/mujoco_playground_pr)** for the **Tetheria Aero Hand Open**.

---

## 🧩 Dependencies

The following repositories are required:

1. **[Aero-Hand-Open SDK](https://github.com/TetherIA/aero-hand-open/tree/main/sdk)** — ships with this repository
2. **[Aero-Open-Firmware](https://github.com/TetherIA/aero-hand-open/tree/main/firmware)** — prebuilt binaries ship with this repository
3. **[MuJoCo Playground](https://github.com/google-deepmind/mujoco_playground)**

---

## ⚙️ Install the Dependencies

### 1) Install Aero Hand SDK from source

The SDK lives in this repository under `sdk/`. Clone the repository and install it in
editable mode:

```bash
git clone https://github.com/TetherIA/aero-hand-open.git
cd aero-hand-open/sdk
pip install -e .
```

For detailed installation instructions, see the [SDK guide](https://github.com/TetherIA/aero-hand-open/tree/main/sdk).

---

### 2) Get the correct firmware binary

The prebuilt firmware binaries are shipped inside the repository you cloned in step 1, so
there is nothing extra to clone.

The Z-rotation policy was validated against firmware **v0.1.0**, archived at
[`firmware/main/bin/rl_z_rotation/`](https://github.com/TetherIA/aero-hand-open/tree/main/firmware/main/bin/rl_z_rotation):

```text
firmware/main/bin/rl_z_rotation/firmware_v0.1.0_righthand.bin   # right hand
firmware/main/bin/rl_z_rotation/firmware_v0.1.0_lefthand.bin    # left hand
```

> **Note**
> `rl_z_rotation/` pins the exact build this guide was tested with. The binaries in the
> parent [`firmware/main/bin/`](https://github.com/TetherIA/aero-hand-open/tree/main/firmware/main/bin)
> folder track firmware `main` and are newer even where the filename matches — see the
> [folder README](https://github.com/TetherIA/aero-hand-open/tree/main/firmware/main/bin/rl_z_rotation)
> for details.

Note the path to the binary — you will select it from the GUI file browser in
[Run the Deployment → Flash the firmware](#1-flash-the-firmware).

---

### 3) Install MuJoCo Playground from source

Clone from our maintained fork:

```bash
git clone git@github.com:google-deepmind/mujoco_playground.git
cd mujoco_playground
```

Then follow the [installation guide](https://github.com/google-deepmind/mujoco_playground?tab=readme-ov-file#from-source).

---

## 🧱 3D-Printed Hardware

Deploying the Z-rotation policy on the **physical** hand requires two 3D-printed parts. The STEP (`.stp`) source files are provided in this package's `resource/` folder:

| File | Description |
| ---- | ----------- |
| [`60_60_60_block.stp`](resource/60_60_60_block.stp) | The 60 × 60 × 60 mm cube that the policy is trained to rotate about the Z-axis. Ideally printed to weigh **117 g** (tune infill/material so the printed cube matches this target mass). |
| [`mount.stp`](resource/mount.stp) | Mount/fixture used to hold the hand in place during deployment. |

Print both parts before running the deployment steps below.

---

## 🚀 Run the Deployment

### 1) Flash the firmware

1. Launch the GUI:
   ```bash
   aero-hand-gui
   ```
2. Locate the serial port connected to the hand.  
3. Click the **Upload Firmware** button.  
4. In the file browser, navigate to the pinned firmware folder of this repository:
   ```text
   aero-hand-open/firmware/main/bin/rl_z_rotation
   ```
   Select `firmware_v0.1.0_righthand.bin` (or `firmware_v0.1.0_lefthand.bin` for a left
   hand) and click **Open**.  
5. Close the GUI after flashing — keeping it open may interfere with serial communication.

---

### 2) Run the ROS 2 nodes

Enter the ROS 2 workspace, which also ships in this repository:

```bash
cd aero-hand-open/ros2
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
