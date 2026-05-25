#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Trigger the full pick → carry → place demo on the cpp moveit_control
# node, bypassing pilot / atlas / pick_skill entirely. Useful for
# debugging the demo poses + gripper widths in isolation, or for
# canned showcase runs where you just want to fire the gesture from
# a terminal.
#
# Prerequisites:
#   * piper_moveit_rbnx is ACTIVE (i.e. its rbnx Driver(CMD_INIT) has
#     completed → move_group + cpp moveit_control_node_yolo are
#     running). The cpp node advertises /moveit_control/full_demo;
#     this script just calls it with std_srvs/srv/Trigger.
#   * piper_ctl_rbnx is ACTIVE (the cpp node's start-up self-test
#     would have failed otherwise; if the self-test failed,
#     full_demo will refuse to drive the arm — same as any other
#     grasp command — and you'll see RCLCPP_FATAL in the cpp logs).
#
# Usage:
#   bash scripts/run_full_demo.sh
#
# What it does (cpp-side fullDemoCallback):
#   leg 1  open gripper to demo_gripper_width (0.08)
#          → moveArmtoDemo()                   [first fixed pose]
#          → close gripper to grasp_close_width (0.025)
#   leg 2  moveArmtoInit()                     [carry, gripper closed]
#   leg 3  moveArmtoDemoPlace()                [second fixed pose]
#          → sleep place_open_pause_s (2.0 s)
#          → open gripper to demo_gripper_width (0.08)
#          → moveArmtoInit()                   [park, gripper open]
#
# Tuning (operator edits + colcon build piper_moveit_control):
#   * cpp moveArmtoDemo()      degrees[]   — pick joint angles
#   * cpp moveArmtoDemoPlace() degrees[]   — place joint angles
#   * ros params:
#       demo_gripper_width  (default 0.08)
#       grasp_close_width   (default 0.025)
#       place_open_pause_s  (default 2.0)
#
# Exit code: 0 on Trigger response success=true, non-zero otherwise.
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

SERVICE="/moveit_control/full_demo"
echo "[run_full_demo] calling $SERVICE (std_srvs/srv/Trigger) ..."

# Single-shot Trigger call. Trigger Response carries `success` (bool)
# + `message` (string) — we parse the rendered yaml output.
# `ros2 service call` does NOT exit non-zero on response.success=false
# (it only fails on transport errors), so success grepping is the
# only way to surface a logical failure into the shell exit code.
OUTPUT="$(ros2 service call "$SERVICE" std_srvs/srv/Trigger '{}' 2>&1 | tee /dev/stderr || true)"

if echo "$OUTPUT" | grep -qE 'success=True|success: True|success: true'; then
    echo "[run_full_demo] OK"
    exit 0
else
    echo "[run_full_demo] FAILED — see cpp moveit_control_node_yolo logs for details"
    exit 1
fi
