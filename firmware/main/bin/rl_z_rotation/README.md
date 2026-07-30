# Firmware binaries for the RL Z-rotation policy

These binaries are the exact build the **Z-rotation RL policy** was validated against, kept
here so the [deployment guide](../../../../ros2/src/aero_hand_open_rl/README.md) does not
depend on access to the private `aero-open-firmware` repository.

## Provenance

Copied verbatim from `TetherIA/aero-open-firmware`, `main/bin/`, at commit
`46bc858cf07f8c8858887ff11c5362b4078bc869`.

| File | Size (bytes) | SHA-256 |
| ---- | ------------ | ------- |
| `firmware_v0.1.0_righthand.bin` | 349248 | `a112b5370e854c24d503c201fdea9e316a81b4d3839c45bbe4eca6beaa96786d` |
| `firmware_v0.1.0_lefthand.bin` | 349248 | `a112b5370e854c24d503c201fdea9e316a81b4d3839c45bbe4eca6beaa96786d` |

## ⚠️ These are not the latest v0.1.0 binaries

The files in the parent [`bin/`](..) folder track the firmware `main` branch and are newer,
even where the filename is the same. The v0.1.0 binaries were rebuilt twice after commit
`46bc858`:

- *Updated full speed after homing and handconfig for left or right hand* (upstream #11)
- *Added Set speed, Set torque and torque control mode* (upstream #12)

Use this folder only to reproduce the Z-rotation deployment; for anything else use the
maintained binaries in [`bin/`](..).
