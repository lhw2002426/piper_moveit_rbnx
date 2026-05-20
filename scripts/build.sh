#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Build phase. Two steps:
#   1. colcon build the vendored ROS packages:
#        graspnet_msgs              msg/srv (Python bindings + cpp deps)
#        piper_msgs                 PiperStatusMsg + PosCmd + Enable.srv
#        piper_humble               empty shell, but referenced by
#                                   piper_with_gripper_moveit's cmakelists
#        piper_moveit/...           MoveIt config (install-only ament_cmake)
#        piper_moveit_control       C++ — moveit_control_node_yolo executable
#   2. rbnx codegen --mcp:
#        atlas_pb2 / atlas_pb2_grpc        (Service runtime)
#        manipulation_mcp.py               (ExecuteGrasp_Request/_Response)
#        geometry_msgs_mcp.py              (PoseStamped + nested types)
#        std_msgs_mcp.py / builtin_interfaces_mcp.py
#
# Prerequisites on the host (Linux + ROS humble):
#   apt install ros-humble-moveit ros-humble-moveit-ros-planning-interface \
#               ros-humble-moveit-configs-utils ros-humble-tf2-ros \
#               ros-humble-tf2-geometry-msgs ros-humble-control-msgs \
#               ros-humble-rclcpp-action
#   (everything else is moveit_ros_planning_interface's transitive deps)
#
# Output layout:
#   rbnx-build/codegen/proto_gen/                atlas stubs
#   rbnx-build/codegen/robonix_mcp_types/        manipulation_mcp etc.
#   rbnx-build/ws/install/<each_pkg>/            colcon outputs
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[piper_moveit/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/ws/src rbnx-build/data

# Symlink each vendored ROS package into the colcon ws.
# (Note: piper_moveit/ is a wrapper directory containing two
# packages. colcon recurses into it, so a single symlink is enough.)
for sub in graspnet_msgs piper_msgs piper_humble piper_moveit piper_moveit_control; do
    ln -snf "$PKG/src/$sub" "$PKG/rbnx-build/ws/src/$sub"
done

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

echo "[piper_moveit/build] colcon build (full piper_moveit stack)"
cd "$PKG/rbnx-build/ws"
# We don't pin --packages-select, so colcon picks up the full
# vendored set in topological order (msgs before moveit_control,
# etc.). The MoveIt configs are install-only ament_cmake (~1s);
# moveit_control_node_yolo is the only real cpp build (~30-60s).
colcon build --symlink-install \
    --event-handlers console_direct+ \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cd "$PKG"

FLAGS=(--out-dir "$PKG/rbnx-build/codegen" --mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[piper_moveit/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[piper_moveit/build] done."
