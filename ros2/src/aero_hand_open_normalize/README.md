# Normalization Scripts

## Purpose

These scripts calibrate the min/max range of each hand joint after retargeting.

Due to morphological differences between human and robot hands, the retargeted joint values need to be scaled to achieve optimal performance. This normalization process maps the observed range of motion to the robot's actual joint limits.

---

## Method

### Step 1: Record Joint Data
Run the teleoperation pipeline(without normalization*):
```bash
ros2 run manus_ros2 manus_data_publisher
```
We need to run the `manus_joint_states_retargeting` node without normalization. To do this, we can comment out the normalization code in the `manus_joint_states_retargeting.py` file. Concretely, comment out lines 108-111:
```python
# for i in range(joint_values.shape[0]):
#     joint_values[i] = normalize_joint_state(
#         joint_values[i], i, self.normalize_config
#     )
```
Now we run the `manus_joint_states_retargeting` node:
```bash
ros2 run aero_hand_open_retargeting manus_joint_states_retargeting
```

Run the `joint_states_logger` python script alongside your teleoperation pipeline:

```bash
cd aero-hand-open/ros2/src/aero_hand_open_normalize/aero_hand_open_normalize
python3 joint_states_logger.py
```

While the logger is running:
- Move each finger joint from min to max position, slowly
- Repeat 3–5 times per joint
- Move joints individually, then all together

Press `Ctrl+C` to stop. A CSV file will be saved in this package folder (e.g., `joint_states_20260119-120545.csv`).

### Step 2: Generate Normalization Config

Run the peak detection script on your recorded data:

```bash
python3 find_peaks_and_valleys_joint.py \
  --csv_file joint_states_20260119-120545.csv \
  --user joe
```

### Step 3: Apply Normalization

Use the generated config file in your retargeting pipeline. See `aero_hand_open_retargeting/manus_joint_states_retargeting.py` for an example implementation(Line: 50) `self.normalize_config = load_normalize_config("joe")`. Update the <user> with your name. And decomment the normalization code in the `manus_joint_states_retargeting.py` file. Concretely, uncomment lines 108-111:
```python
for i in range(joint_values.shape[0]):
    joint_values[i] = normalize_joint_state(
        joint_values[i], i, self.normalize_config
    )
```

### Step 4: Fine-Tune (Optional)

After testing the updated teleoperation, you can manually edit the YAML config to further tweak the scaling values.

