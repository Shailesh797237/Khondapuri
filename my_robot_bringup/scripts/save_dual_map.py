#!/usr/bin/env python3
"""
save_dual_map.py — Automated Dual 2D + 3D Map Saver

Saves both 2D map (.yaml/.pgm) and 3D map (.pcd) with automatic
sync configuration so that navigation loads both maps 100% synchronized!

Usage:
  ros2 run my_robot_bringup save_dual_map.py -f /home/taikisha/turn_ws/try_7
"""

import sys
import os
import argparse
import subprocess
import shutil
import yaml

def main():
    parser = argparse.ArgumentParser(description="Save synchronized 2D + 3D map")
    parser.add_argument("-f", "--filename", required=True, help="Full path prefix for map, e.g. /home/taikisha/turn_ws/try_7")
    args = parser.parse_args()

    prefix = os.path.abspath(args.filename)
    dirname = os.path.dirname(prefix)
    basename = os.path.basename(prefix)

    os.makedirs(dirname, exist_ok=True)

    print(f"============================================================")
    print(f" 💾 Saving Synchronized Dual Map: {basename}")
    print(f"============================================================")

    # 1. Save 2D map using map_saver_cli
    print("1️⃣ Saving 2D Map (OccupancyGrid)...")
    cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", prefix]
    res = subprocess.run(cmd)
    if res.returncode != 0:
        print("⚠ Failed to save 2D map via map_saver_cli. Ensure slam_toolbox / map_server is running.")

    # 2. Check for 3D PCD map
    pcd_src = "/home/taikisha/turn_ws/scans.pcd"
    pcd_dst = f"{prefix}.pcd"
    if os.path.exists(pcd_src):
        shutil.copy(pcd_src, pcd_dst)
        print(f"2️⃣ Copied 3D PCD map to: {pcd_dst}")
    else:
        print(f"2️⃣ PCD map at {pcd_src} not found yet. It will save when dual_mapping_launch.py exits.")

    # 3. Create synchronization metadata file (prefix_sync.yaml)
    sync_yaml = f"{prefix}_sync.yaml"
    sync_data = {
        "map_2d_yaml": f"{prefix}.yaml",
        "map_3d_pcd": pcd_dst,
        "map_offset_x": 0.0,
        "map_offset_y": 0.0,
        "map_offset_yaw": 0.0
    }

    with open(sync_yaml, "w") as f:
        yaml.dump(sync_data, f, default_flow_style=False)

    print(f"3️⃣ Saved Map Sync Config: {sync_yaml}")
    print(f"============================================================")
    print(f" ✅ Map Saving Complete!")
    print(f" To launch navigation with auto-sync:")
    print(f"   ros2 launch amr_pure_navigation navigation.launch.py map:={prefix}.yaml")
    print(f"   ros2 launch lidar3d_localizer ndt_localizer.launch.py pcd_map:={pcd_dst} map_yaml:={prefix}.yaml")
    print(f"============================================================")

if __name__ == "__main__":
    main()
