#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Start phase. Source ROS + colcon overlay + codegen PYTHONPATH, then
# exec the python module.
#
# Same overlay-injection trick as yolo_world_rbnx/scripts/start.sh:
# when the outer shell already has an unrelated colcon overlay sourced
# (e.g. operator's ~/.bashrc sources tracing_ws/install/setup.bash),
# colcon's idempotent prefix marker can cause our
# `source $PKG/rbnx-build/ws/install/setup.bash` to silently NO-OP on
# AMENT_PREFIX_PATH / PYTHONPATH / LD_LIBRARY_PATH. We work around it
# by walking every per-package install prefix under our overlay and
# explicitly prepending its share / python site / lib directories.
#
# Symptoms of NOT doing this (observed in the field):
#
#   1. `from graspnet_msgs.msg import GraspPose` raises
#      ModuleNotFoundError                             ← needs PYTHONPATH
#      → rclpy bridge thread CRASHES, but the lifecycle RPC stays
#        alive, so the package looks ACTIVE while doing nothing.
#
#   2. `ros2 launch ... piper_moveit_rbnx.launch.py` raises
#      PackageNotFoundError: 'piper_with_gripper_moveit' not found
#                                                      ← needs AMENT_PREFIX_PATH
#      → the moveit launch tree never starts; no /arm/arm_status flow.
#
#   3. rclpy.create_publisher dlopens
#      libgraspnet_msgs__rosidl_typesupport_*.so       ← needs LD_LIBRARY_PATH
#
# We inject all three for EVERY package under rbnx-build/ws/install/,
# bypassing the idempotent guard. Cheap and idempotent — duplicates
# in the path vars are harmless.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

OVERLAY_INSTALL="$PKG/rbnx-build/ws/install"
if [[ -f "$OVERLAY_INSTALL/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "$OVERLAY_INSTALL/setup.bash"; set -u
else
    echo "[piper_moveit/start] ERR: colcon overlay missing — run scripts/build.sh" >&2
    exit 2
fi

# ── Direct AMENT_PREFIX_PATH / PYTHONPATH / LD_LIBRARY_PATH injection ──
# Walk every per-package install prefix and prepend the relevant dirs.
_prepend_unique() {
    local var="$1" val="$2"
    local cur="${!var:-}"
    case ":${cur}:" in
        *":${val}:"*) ;;
        *) export "$var"="${val}${cur:+:${cur}}" ;;
    esac
}

if [[ -d "$OVERLAY_INSTALL" ]]; then
    # Each subdir of install/ that contains a package.xml (or a
    # share/<name>/package.xml) is a colcon-emitted prefix.
    for _prefix in "$OVERLAY_INSTALL"/*/; do
        _prefix="${_prefix%/}"
        # Skip non-package dirs colcon drops at the top-level
        # (logs, .colcon_install_layout, COLCON_IGNORE markers, etc).
        [[ -d "$_prefix/share" ]] || continue

        # 1. AMENT_PREFIX_PATH — ament_index resource lookups
        #    (used by ros2 launch's package resolver and many cpp libs).
        _prepend_unique AMENT_PREFIX_PATH "$_prefix"

        # 2. CMAKE_PREFIX_PATH — keeps downstream find_package() happy
        #    even though we don't build at runtime; harmless to inject.
        _prepend_unique CMAKE_PREFIX_PATH "$_prefix"

        # 3. PYTHONPATH — colcon emits python bindings under either
        #    `local/lib/python3.X/dist-packages` (Ubuntu) or
        #    `lib/python3.X/{site,dist}-packages` (others).
        for _site in \
            "$_prefix"/local/lib/python*/dist-packages \
            "$_prefix"/lib/python*/site-packages \
            "$_prefix"/lib/python*/dist-packages
        do
            [[ -d "$_site" ]] && _prepend_unique PYTHONPATH "$_site"
        done

        # 4. LD_LIBRARY_PATH — rclpy.create_{publisher,subscription,
        #    service,client} dlopens libfoo__rosidl_typesupport_*.so
        #    at runtime; cpp executables need their .so deps.
        for _libdir in \
            "$_prefix"/lib \
            "$_prefix"/local/lib
        do
            [[ -d "$_libdir" ]] && _prepend_unique LD_LIBRARY_PATH "$_libdir"
        done
    done
    unset _prefix _site _libdir
fi

# ── External rbnx-package overlays (cross-package dependencies) ─────────────
# piper_with_gripper_moveit's xacro `<xacro:include
# filename="$(find piper_description)/urdf/piper_description.xacro" />`
# means the moveit launch tree NEEDS the `piper_description` ament
# package on AMENT_PREFIX_PATH at runtime. But piper_description lives
# in a SIBLING rbnx package (com.robonix.piper_grasp.piper_description),
# whose colcon install root is independent of ours. We pull it in
# explicitly here.
#
# Resolution order:
#   1. PIPER_DESCRIPTION_RBNX_PREFIX env override
#   2. ros2 pkg prefix (already on ament index — operator pre-sourced)
#   3. sibling boot cache directory (rbnx boot's standard layout puts
#      every package under <boot>/cache/<pkg>/, so ../piper_description
#      from our $PKG hits the sibling)
#   4. dev layout — ../piper_description_rbnx alongside our source tree
_inject_prefix() {
    local prefix="$1"
    [[ -d "$prefix/share" ]] || return 1
    _prepend_unique AMENT_PREFIX_PATH "$prefix"
    _prepend_unique CMAKE_PREFIX_PATH "$prefix"
    for _site in \
        "$prefix"/local/lib/python*/dist-packages \
        "$prefix"/lib/python*/site-packages \
        "$prefix"/lib/python*/dist-packages
    do
        [[ -d "$_site" ]] && _prepend_unique PYTHONPATH "$_site"
    done
    for _libdir in "$prefix"/lib "$prefix"/local/lib; do
        [[ -d "$_libdir" ]] && _prepend_unique LD_LIBRARY_PATH "$_libdir"
    done
    unset _site _libdir
    return 0
}

_find_piper_description_prefix() {
    # 1. explicit env override
    if [[ -n "${PIPER_DESCRIPTION_RBNX_PREFIX:-}" ]]; then
        echo "${PIPER_DESCRIPTION_RBNX_PREFIX}"
        return 0
    fi
    # 2. already on ament index (e.g. operator pre-sourced)
    local p
    if p=$(ros2 pkg prefix piper_description 2>/dev/null); then
        echo "$p"
        return 0
    fi
    # 3. sibling boot cache: rbnx-boot puts each package at
    #    <boot>/cache/<short_name>/; we're piper_moveit, the
    #    sibling is piper_description.
    local sibling="$PKG/../piper_description/rbnx-build/ws/install/piper_description"
    if [[ -d "$sibling/share" ]]; then
        ( cd "$sibling" && pwd )
        return 0
    fi
    # 4. dev layout: $PKG = .../packages/piper_moveit_rbnx, sibling =
    #    .../packages/piper_description_rbnx
    local dev="$PKG/../piper_description_rbnx/rbnx-build/ws/install/piper_description"
    if [[ -d "$dev/share" ]]; then
        ( cd "$dev" && pwd )
        return 0
    fi
    return 1
}

if PIPER_DESC_PREFIX="$(_find_piper_description_prefix)"; then
    if _inject_prefix "$PIPER_DESC_PREFIX"; then
        echo "[piper_moveit/start] using piper_description prefix: $PIPER_DESC_PREFIX" >&2
    else
        echo "[piper_moveit/start] WARN: piper_description prefix candidate \
$PIPER_DESC_PREFIX has no share/ — skipping" >&2
    fi
    unset PIPER_DESC_PREFIX
else
    echo "[piper_moveit/start] WARN: piper_description not found on any \
search path; piper_with_gripper_moveit's xacro will fail to resolve \
'\$(find piper_description)'. Set PIPER_DESCRIPTION_RBNX_PREFIX or \
ensure com.robonix.piper_grasp.piper_description is built and live." >&2
fi

# Quick smoke check: the two msg packages our rclpy bridge imports.
if ! python3 -c "import graspnet_msgs.msg, piper_msgs.msg" 2>/dev/null; then
    echo "[piper_moveit/start] FATAL: graspnet_msgs.msg or piper_msgs.msg not importable" >&2
    echo "[piper_moveit/start] AMENT_PREFIX_PATH:" >&2
    printf '  %s\n' ${AMENT_PREFIX_PATH//:/ } >&2
    echo "[piper_moveit/start] PYTHONPATH:" >&2
    printf '  %s\n' ${PYTHONPATH//:/ } >&2
    echo "[piper_moveit/start] vendored install tree:" >&2
    ls -1 "$OVERLAY_INSTALL" >&2 || true
    exit 3
fi

# Also verify the moveit launch package + its xacro dep
# (piper_description) are on AMENT_PREFIX_PATH — without these,
# `ros2 launch piper_moveit_rbnx.launch.py` blows up with
# PackageNotFoundError before we ever see a useful log.
for _need in piper_with_gripper_moveit piper_description; do
    if ! ros2 pkg prefix "$_need" >/dev/null 2>&1; then
        echo "[piper_moveit/start] FATAL: $_need not on ament index" >&2
        echo "[piper_moveit/start] AMENT_PREFIX_PATH:" >&2
        printf '  %s\n' ${AMENT_PREFIX_PATH//:/ } >&2
        exit 4
    fi
done
unset _need

CODEGEN_PROTO="$PKG/rbnx-build/codegen/proto_gen"
CODEGEN_MCP="$PKG/rbnx-build/codegen/robonix_mcp_types"
if [[ ! -d "$CODEGEN_PROTO" || ! -d "$CODEGEN_MCP" ]]; then
    echo "[piper_moveit/start] ERR: codegen output missing — run scripts/build.sh" >&2
    exit 2
fi
export PYTHONPATH="$CODEGEN_PROTO:$CODEGEN_MCP:$PKG:${PYTHONPATH:-}"
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PYTHONPATH"
fi

exec python3 -u -m piper_moveit.main
