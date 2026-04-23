#!/usr/bin/env python3
import sys
import os
import argparse
import pandas as pd
import numpy as np
from scipy.signal import find_peaks
import yaml
import matplotlib.pyplot as plt

try:
    from ament_index_python.packages import get_package_share_directory

    _HAS_AMENT = True
except ImportError:
    _HAS_AMENT = False

# Joint names from aero_hand_open_msgs/msg/JointControl.msg (source of truth)
JOINT_NAME_MAP = {
    0: "thumb_cmc_abd",
    1: "thumb_cmc_flex",
    2: "thumb_mcp",
    3: "thumb_ip",
    4: "index_mcp_flex",
    5: "index_pip",
    6: "index_dip",
    7: "middle_mcp_flex",
    8: "middle_pip",
    9: "middle_dip",
    10: "ring_mcp_flex",
    11: "ring_pip",
    12: "ring_dip",
    13: "pinky_mcp_flex",
    14: "pinky_pip",
    15: "pinky_dip",
}


def analyze_joint(t, y, min_gap_sec):
    """Compute median of top-5 peaks and 3rd smallest of all valleys."""
    if len(t) >= 2:
        avg_dt = float(np.mean(np.diff(t)))
        if not np.isfinite(avg_dt) or avg_dt <= 0:
            avg_dt = None
    else:
        avg_dt = None

    min_distance_samples = (
        1 if avg_dt is None else max(1, int(round(min_gap_sec / avg_dt)))
    )

    peaks_all, _ = find_peaks(y, distance=min_distance_samples)
    if len(peaks_all) == 0:
        p = int(np.argmax(y))
        peaks_top = np.array([p] * 5)
    else:
        idx_sorted_by_val = np.argsort(y[peaks_all])[::-1]
        peaks_top = peaks_all[idx_sorted_by_val[:5]]
        peaks_top = np.unique(np.sort(peaks_top))
        while len(peaks_top) < 5:
            peaks_top = np.append(peaks_top, peaks_top[-1])
    peaks_top = peaks_top[:5]

    peak_values = y[peaks_top].astype(float)
    peaks_median = float(np.median(peak_values)) if peak_values.size > 0 else None

    # Find minimum value within each segment between peaks
    all_valley_indices = []
    for k in range(4):
        a, b = int(peaks_top[k]), int(peaks_top[k + 1])
        if b <= a + 1:
            all_valley_indices.append(a)
            continue
        seg = y[a : b + 1]
        seg_min_val = float(np.min(seg))
        seg_min_rel = np.where(seg == seg_min_val)[0]
        all_valley_indices.extend((seg_min_rel + a).tolist())

    valley_values = (
        [float(y[i]) for i in all_valley_indices] if all_valley_indices else []
    )
    valleys_third_smallest = (
        float(sorted(valley_values)[2]) if len(valley_values) >= 3 else None
    )

    return peaks_top, all_valley_indices, peaks_median, valleys_third_smallest


def main():
    parser = argparse.ArgumentParser(
        description="Find top-5 peaks and 3rd smallest valley for a joint, output YAML config and plot."
    )
    parser.add_argument(
        "--csv_file", help="Input CSV file containing time_sec, joint_0..joint_3"
    )
    parser.add_argument(
        "--joints",
        type=int,
        nargs="*",
        default=list(range(16)),
        help="List of joint indexes (0–15). Default: all joints 0-15",
    )
    parser.add_argument(
        "--user",
        type=str,
        default="default_user",
        help="User name for config file naming. Default: default_user",
    )
    parser.add_argument(
        "--min-gap",
        type=float,
        default=1.0,
        help="Minimum time gap between peaks (s). Default: 1.0",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help=(
            "Output path for the YAML config file. If not specified, writes to "
            "the aero_hand_open_retargeting package share directory (via ament index), "
            "or falls back to ./normalize_<user>.yaml in the current directory."
        ),
    )
    args = parser.parse_args()

    csv_file = args.csv_file
    if not os.path.exists(csv_file):
        print(f"❌ File not found: {csv_file}")
        sys.exit(1)

    # Validate joint indexes
    joints_to_process = args.joints
    for joint_idx in joints_to_process:
        if joint_idx not in JOINT_NAME_MAP:
            print(f"❌ Invalid joint index {joint_idx}. Must be 0–15.")
            sys.exit(1)

    df = pd.read_csv(csv_file)
    t = df["time_sec"].to_numpy()

    # ===== YAML Config Path Resolution =====
    if args.output:
        config_path = args.output
    elif _HAS_AMENT:
        try:
            pkg_share = get_package_share_directory("aero_hand_open_retargeting")
            config_path = os.path.join(pkg_share, "config", f"normalize_{args.user}.yaml")
        except Exception as e:
            print(f"⚠️  Could not find aero_hand_open_retargeting package: {e}")
            config_path = f"normalize_{args.user}.yaml"
            print(f"   Falling back to current directory: {config_path}")
    else:
        config_path = f"normalize_{args.user}.yaml"
        print(f"⚠️  ament_index not available. Writing to current directory: {config_path}")

    config_dir = os.path.dirname(config_path)
    if config_dir:
        os.makedirs(config_dir, exist_ok=True)

    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            try:
                config = yaml.safe_load(f) or {}
            except yaml.YAMLError:
                config = {}
    else:
        config = {}

    # Store results for plotting
    results = {}

    # Process each joint
    for joint_idx in joints_to_process:
        joint_col = f"joint_{joint_idx}"
        joint_key = JOINT_NAME_MAP[joint_idx]

        if joint_col not in df.columns:
            print(f"⚠️  Column {joint_col} not found in CSV. Skipping {joint_key}.")
            continue

        y = df[joint_col].to_numpy()
        peaks_top, valleys_idx, peak_val, valley_val = analyze_joint(t, y, args.min_gap)

        print(f"\n✅ Joint {joint_idx}: {joint_key}")
        print(f"  Peak : {peak_val}")
        print(f"  Valley: {valley_val}")

        config[joint_key] = {"peak": peak_val, "valley": valley_val}
        results[joint_idx] = {
            "key": joint_key,
            "y": y,
            "peaks": peaks_top,
            "valleys": valleys_idx,
            "peak_val": peak_val,
            "valley_val": valley_val,
        }

    # ===== YAML Writing =====
    with open(config_path, "w") as f:
        yaml.safe_dump(config, f, sort_keys=False)

    print(f"\n📝 Saved normalize parameters to {config_path}")

    # ===== Plot =====
    num_joints = len(results)
    if num_joints == 0:
        print("⚠️  No joints to plot.")
        return

    # Calculate grid size for subplots
    cols = min(4, num_joints)
    rows = (num_joints + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows), squeeze=False)
    axes = axes.flatten()

    for i, (joint_idx, data) in enumerate(results.items()):
        ax = axes[i]
        joint_key = data["key"]
        y = data["y"]
        peaks_top = data["peaks"]
        valleys_idx = data["valleys"]

        ax.plot(t, y, label=joint_key, lw=1.0)
        ax.scatter(t[peaks_top], y[peaks_top], c="red", s=50, label="Peaks")
        if valleys_idx:
            ax.scatter(
                [t[j] for j in valleys_idx],
                [y[j] for j in valleys_idx],
                c="green",
                s=50,
                marker="s",
                label="Valleys",
            )
        ax.set_title(f"{joint_key}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position")
        ax.grid(True)
        ax.legend(fontsize=8)

    # Hide unused subplots
    for j in range(num_joints, len(axes)):
        axes[j].set_visible(False)

    plt.suptitle(f"Peaks & Valleys (≥{args.min_gap:.1f}s apart)", fontsize=14)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

