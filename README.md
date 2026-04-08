![Aero Hand Overview](assets/banner.png)

<p align="center">
  <a href="https://tetheria.github.io/aero-hand-open/"><img src="https://img.shields.io/badge/project-page-brightgreen" alt="Project Page"></a>
  <a href="https://docs.tetheria.ai/"><img src="https://img.shields.io/badge/doc-page-orange" alt="Documentation"></a>
  <!-- <a href="https://github.com/TetherIA/aero-hand-open/issues"><img src="https://img.shields.io/github/issues/RoboVerseOrg/RoboVerse?color=yellow" alt="Issues"></a> -->
  <a href="https://github.com/TetherIA/aero-hand-open/discussions"><img src="https://img.shields.io/github/discussions/RoboVerseOrg/RoboVerse?color=blueviolet" alt="Discussions"></a>
  <a href="http://discord.gg/ZQKWK7NebQ"><img src="https://img.shields.io/discord/1356345436927168552?logo=discord&color=blue" alt="Discord"></a>
  <a href="https://shop.tetheria.ai/"><img src="https://img.shields.io/badge/Shop-shopping-purple?logo=shopify" alt="Shop"></a>
  <a href="https://www.linkedin.com/company/tetheria/"><img src="https://img.shields.io/badge/LinkedIn-Follow-blue?logo=linkedin" alt="LinkedIn"></a>
  <a href="https://x.com/TetherIA_ai"><img src="https://img.shields.io/badge/X-Follow-black?logo=x" alt="X"></a>
  <a href="https://www.youtube.com/@TetherIA_ai"><img src="https://img.shields.io/badge/YouTube-Subscribe-red?logo=youtube" alt="YouTube"></a>
</p>

Aero Hand Open is an **open-source**, **tendon-driven** robotic hand designed and developed by TetherIA for dexterous manipulation research. Unlike expensive proprietary solutions, this hand focuses on **simplicity**, **reliability**, and **accessibility** by using standard 3D printing and off-the-shelf electronic components.

Each joint is optimized for mechanical efficiency through tendon actuation, enabling smooth and natural motion while maintaining a **lightweight** and **compact** design, making it perfect for research labs, educational institutions, and robotics enthusiasts who need an **affordable** yet **capable** manipulation platform.

> **📚 Learn More:** https://tetheria.github.io/aero-hand-open/   
> **🛒 Shop:** https://shop.tetheria.ai/


# Aero Hand Open features
- 7 DoFs robotic hand with 5 fingers (16 joints in total)
- Tendon-driven architecture for smooth and natural motion
- Fully 3D-printed structure, modular and easy to assemble
- Lightweight design — **389 g**
- Affordable — complete kit for **$314 USD**
- Open-source hardware and firmware
- Independent Python SDK, and compatible with ESP32 and ROS2 systems

# Table of Contents
- [Aero Hand Open features](#aero-hand-open-features)
- [Overview](#overview)
- [Resources](#resources)
  - [Bill of Materials (BOM)](#bill-of-materials-bom)
  - [CAD Files and 3D Models](#cad-files-and-3d-models)
  - [Assembly Guide](#assembly-guide)
  - [PCB Design](#pcb-design)
  - [Hardware Setup](#hardware-setup)
  - [Software SDK](#software-sdk)
  - [ROS2 and Teleoperation](#ros2-and-teleoperation)
  - [Simulation](#simulation)
  - [Reinforcement Learning](#reinforcement-learning-tools)
- [Getting help](#Getting-help)
- [License — TL;DR](#license--tldr)
- [Disclaimer](#disclaimer)
- [Project Updates & Community](#project-updates--community)
  - [Updates History](#updates-history)
  - [FAQ](#faq)
  - [Contact](#contact)


# Overview

![Overview1](assets/overview1.png)

![Overview1](assets/overview2.png)

![Overview1](assets/overview3.png)

![Overview1](assets/overview4.png)

![Overview1](assets/overview5.png)

![Overview1](assets/overview6.png)

![Overview1](assets/overview7.png)


# Resources
## Bill of Materials (BOM)
The complete list of components required to build Aero Hand Open can be found here:

 [👉 Aero Hand Open – Bill of Materials](./hardware/Assembly/BOM.csv)

This document includes all mechanical, electronic, and printed parts — such as motors, tendons, bearings, fasteners, and 3D-printed components.

Each item is listed with its part number, vendor, quantity, and estimated cost to help you easily source or substitute parts.

## CAD Files and 3D Models
STEP files and one click print files can be found here:

[👉 Aero Hand Open – CAD](./hardware/CAD/).

[👉 Aero Hand Open – Onshape Link](https://cad.onshape.com/documents/afc7e0ca7eb6d412ec8771f8/w/bc4d7e45e17e23d622d2bad2/e/c711982b7882da925263fb55?renderMode=0&uiState=69d6d9eafca41cd78ff4328c)

All parts are printed on a Bambu X1C, 0.4mm nozzle, 0.2mm layer height, tree support enabled, select support on build plate only checkbox. We recommend keeping the same print orientation and settings. The print orientation will minimize supports and optimize surface tolerances/smoothness for interfacing components.


## Assembly Guide
The step-by-step assembly instructions for Aero Hand Open are provided in the following document:

[👉 Aero Hand Open – Assembly Guide](https://docs.tetheria.ai/docs/assembly)

This guide covers the entire build process, from mounting the actuators to routing the tendons and connecting the electronics. Each finger module can be assembled independently and attached to the palm afterward, allowing easier maintenance and quick replacement.

## PCB Design
All design files - including Gerber files, KiCad project files, BOM, and CPL - are available in [PCB folder](./hardware/PCB/).

Referring to [PCB doc](https://docs.tetheria.ai/docs/pcb) for more technical details.



## Hardware Setup

Please refer to [hardware_setup](https://docs.tetheria.ai/docs/hardware_setup) doc 


## Software SDK
Please refer to [`sdk/README.md`](sdk/README.md) and [sdk](https://docs.tetheria.ai/docs/sdk) doc

### Sequencing Demo
The Sequencing Demo demonstrates how different finger motions can be combined into continuous, pre-defined sequences.

This script enables users to perform complex gestures — such as pinching, opening the palm, or making a peace sign — by automatically coordinating joint movements in a timed sequence.

You can find the example code in the SDK example folder. 

👉 [`sdk/examples/`](sdk/examples)

Once the SDK is installed and the serial port has been configured, you can run any of the example scripts directly using Python as follows:

```bash
python run_sequence.py
```
[🎥 Watch the demo](assets/sequence_square.mp4)

## ROS2 and Teleoperation
The Aero Hand Open integrates seamlessly with ROS2 **humble** for advanced robotics applications.

Refer to the [ROS2](https://docs.tetheria.ai/docs/ros2) doc and [`ros2/`](./ros2/) folder for complete setup instructions and source code.

## Simulation

We provide high-fidelity simulation models for the **Tetheria Aero Hand Open**.

**Currently Supported**
- **MuJoCo**

Support for additional simulation platforms is under active development and will be released in future updates.  
Refer to the [RL and Sim](https://docs.tetheria.ai/docs/hand_sim) documentation and the [`sim_rl/simulation/`](./sim_rl/simulation/) directory for detailed explanations and model files.

## Reinforcement Learning Tools

We offer reinforcement learning tools built on top of state-of-the-art frameworks such as **mujoco_playground**, enabling users to train custom policies for the Aero Hand Open with minimal effort.

Refer to the [RL and Sim](https://docs.tetheria.ai/docs/hand_sim) documentation and the [`sim_rl/mujoco_playground/`](./sim_rl/mujoco_playground/) directory for full setup instructions and training procedures.  
An example of deploying trained policies in ROS is also provided in the [`ros2/`](./ros2/) directory.


# Getting help

**GitHub Issues** – Bugs + feature requests only.

**GitHub Discussions** –
- Build / electronics / mechanical help
- SDK / ROS2 / sim/RL "how do I…"
- Design proposals and feedback

**Discord** –
- Quick questions, live debugging, voice/screen‑share
- Social chat, lab intros, "I printed my first hand!"
- Live events and office hours

# License — TL;DR

[![Commercial integration of purchased units: ALLOWED](https://img.shields.io/badge/Commercial%20integration%20of%20purchased%20units-ALLOWED-brightgreen)](#license--tldr)

- You **can** integrate Aero Hand units you purchase from TetherIA into **commercial robots and products** you sell.
- **Software (firmware & SDK):** Apache-2.0 — commercial use OK (with notices).
- **Design files (CAD/STEP/STL, drawings, BOM, docs):** CC BY‑NC‑SA 4.0 — **non‑commercial** only; derivatives must use the same license with attribution.
- Want to **manufacture/print parts** or **make your own hands** from our design files for commercial use (spares, kits, or clones)? → **Commercial manufacturing license required**.
- **Commercial licensing & volume buys:** see **LICENSE.md** (or email us at contact@tetheria.ai).

See [**LICENSE.md**](LICENSE.md) for definitions, examples, and contact details.

  
© 2025 TetherIA Inc. All rights reserved.

# Disclaimer
Aero Hand Open is an open-source research prototype intended for educational and experimental purposes only.

While every effort has been made to ensure build accuracy and functionality, this design has not been validated for prolonged or heavy-duty use.

Users should exercise caution when assembling, operating, or modifying this device. TetherIA Inc. and its contributors shall not be held liable for any personal injury, property damage, or other losses resulting from the use, misuse, or modification of this design. By using this project, you acknowledge and accept full responsibility for any associated risks.

Important Notes:
- 3D-printed parts may exhibit tolerance variations depending on printer and material settings.
- The tendon-driven mechanism requires regular tension adjustment to maintain consistent motion.
- Overloading the joints or applying excessive torque may cause mechanical deformation or servo damage.
- This project is provided as is, without any warranty or guarantee of fitness for a particular purpose.

We encourage the community to share improvements, feedback, and modifications through pull requests or GitHub discussions.

Your contributions will help make Aero Hand Open more reliable and versatile for the entire robotics community.

# Project Updates & Community
## Updates History
See [RELEASE_NOTES.md](RELEASE_NOTES.md) for details.

## FAQ
Q1: Can I modify and redistribute the Aero Hand Open design?

A: Yes, as long as it complies with the CC BY-NC-SA 4.0 license (non-commercial, attribution required, share alike).

Q2: What 3D printer and material do you recommend?

A: Any FDM printer with a ≥200×200 mm bed. PLA works best for strength and dimensional accuracy.

More questions please refer to our [online documentation](https://docs.tetheria.ai/docs/hardware_faq) 

## Contact
For questions, feedback, or collaboration inquiries, please reach out to us through the following channels:

 🛒 Shop: [Aero Hand Open – TetherIA Store](https://shop.tetheria.ai/) 

 📚 Docs: [TetherIA Docs](https://docs.tetheria.ai/)

 📧 Email: support@tetheria.ai

 🌐 Website: [tetheria.ai](http://tetheria.ai)

 🐙 GitHub: [TetherIA](https://github.com/TetherIA)

 💬 Discord: [TetherIA Discord Channel](http://discord.gg/ZQKWK7NebQ)

  🐦 X/Twitter: [TetherIA X/Twitter Account](https://x.com/TetherIA_ai)

  📺 YouTube: [TetherIA YouTube Account](https://www.youtube.com/@TetherIA_ai)

  💼 LinkedIn: [TetherIA LinkedIn Account](https://www.linkedin.com/company/tetheria/)


We welcome discussions, contributions, and new ideas from the community.

If you have improvements to the design, firmware, or control software, feel free to open a Pull Request or start a Discussion on GitHub.
