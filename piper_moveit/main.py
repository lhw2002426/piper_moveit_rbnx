#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""piper_moveit_rbnx — manipulation service.

Wraps:
  * piper_with_gripper_moveit (MoveIt config + URDF/SRDF)
  * piper_moveit_control (C++ moveit_control_node_yolo — the actual
    grasp executor: subscribes /graspnet/grasps, plans + runs
    trajectory via MoveGroupInterface)

into one robonix Service. Owns ``robonix/service/manipulation/*``.

Architectural choice (the safest one): we do NOT reimplement the
C++ moveit_control logic in Python, and we do NOT modify the C++
source. Instead the MCP `execute_grasp` handler:

   1. Publishes a graspnet_msgs/msg/GraspPose to /graspnet/grasps,
      which wakes up the vendored C++ moveit_control_node_yolo
      subscriber (it transforms the pose into arm/base_link, plans
      with MoveIt, then runs the trajectory through the gripper +
      arm controllers).

   2. Polls /arm/arm_status (piper_msgs/PiperStatusMsg) for the
      arm_status == 0 (idle) condition — same signal pick.py uses
      upstream. We wait for a brief busy → idle transition (so we
      don't return immediately on a spurious idle reading captured
      before the C++ node had a chance to start moving).

   3. Returns success/failure + elapsed time.

This reduces Stage 5 to "wrap launch, wrap topic, wrap status poll"
— no MoveIt API surface in Python, no graspnet planning logic
duplication. If MoveIt fails (planning or execution), the C++ node
logs the error but does NOT publish a failure topic — we detect it
through timeout on the busy/idle transition.

Lifecycle (per Robonix developer guide §5):
    on_init  — heavy:
                 * resolve atlas deps (informational: arm_status,
                   grasp_pose; we connect via raw ROS for simplicity)
                 * spawn `ros2 launch piper_moveit_rbnx.launch.py`
                   (which starts move_group + RSP + control_node_yolo)
                 * sentinel-wait for /arm/arm_status to publish
                   (proves the cpp graph is up + piper_ctl alive)
                 * spawn rclpy thread (publisher + subscriber for
                   the MCP handler)

    on_deactivate — kill the launch subprocess group, stop rclpy thread.
"""
from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Optional

from robonix_api import ATLAS, Service, Ok, Err  # noqa: E402

logging.basicConfig(
    level=os.environ.get("PIPER_MOVEIT_LOG_LEVEL", "INFO"),
    format="[piper_moveit] %(message)s",
)
log = logging.getLogger("piper_moveit")

piper_moveit = Service(
    id=os.environ.get("ROBONIX_CAPABILITY_ID", "piper_moveit"),
    namespace="robonix/service/manipulation",
)

# ── shared state ────────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_initialized = False
_resolved_cfg: Optional[dict[str, Any]] = None

_launch_proc: Optional[subprocess.Popen] = None

_ros_node = None
_ros_thread: Optional[threading.Thread] = None
_ros_stop_evt = threading.Event()
_grasps_pub = None              # /graspnet/grasps publisher
_arm_status_lock = threading.Lock()
_last_arm_status_value: Optional[int] = None      # PiperStatusMsg.arm_status
_last_arm_status_stamp: float = 0.0               # monotonic ts

# arm_status semantic (from upstream pick.py + piper_msgs/PiperStatusMsg):
#   0 == "normal / idle" — arm finished any in-progress motion.
#   non-zero == various busy / error states.
ARM_STATUS_IDLE = 0


# ── atlas dep resolve (informational only) ──────────────────────────────────
def _log_atlas_deps() -> None:
    """Log which providers own the topics we'll subscribe / publish.
    We don't actually use the atlas-resolved endpoints — for Stage 5 we
    just go directly via the well-known ROS topic names that
    piper_ctl_rbnx + yolo_grasp_rbnx hardcode (they're the same as
    what the legacy pipeline uses, so no remap drama).
    Stage 6 (or a future polish pass) can swap to atlas-resolved
    endpoints if topic names ever start to vary across deploys.
    """
    for cid in (
        "robonix/primitive/arm/arm_status",
        "robonix/service/perception/grasp_pose/grasps",
    ):
        try:
            caps = ATLAS.find_capability(contract_id=cid, transport="ros2")
        except Exception as e:  # noqa: BLE001
            log.warning("atlas query %s failed: %s", cid, e)
            continue
        if not caps:
            log.warning("atlas: no provider for %s yet (will use default topic)", cid)
        else:
            log.info("atlas: %s provided by %s", cid, caps[0].provider_id)


# ── launch subprocess management ────────────────────────────────────────────
def _spawn_launch(cfg: dict) -> subprocess.Popen:
    pkg_root = Path(os.environ.get(
        "RBNX_PACKAGE_ROOT",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
    ))
    launch_file = pkg_root / "launch" / "piper_moveit_rbnx.launch.py"
    if not launch_file.is_file():
        raise FileNotFoundError(f"launch file missing: {launch_file}")

    args = ["ros2", "launch", str(launch_file)]
    # Forward config knobs to launch args.
    for cfg_key, launch_arg in (
        ("arm_group_name",      "arm_group_name"),
        ("end_effector_link",   "end_effector_link"),
        ("gripper_action_name", "gripper_action_name"),
    ):
        v = (cfg.get(cfg_key) or "").strip()
        if v:
            args.append(f"{launch_arg}:={v}")

    log.info("spawning: %s", " ".join(args))
    return subprocess.Popen(args, start_new_session=True)


def _kill_launch() -> None:
    global _launch_proc
    if _launch_proc is None:
        return
    try:
        os.killpg(os.getpgid(_launch_proc.pid), signal.SIGTERM)
        try:
            _launch_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            log.warning("launch group did not exit in 5s; SIGKILL")
            os.killpg(os.getpgid(_launch_proc.pid), signal.SIGKILL)
            _launch_proc.wait(timeout=3.0)
    except ProcessLookupError:
        pass
    except Exception as e:  # noqa: BLE001
        log.warning("kill launch failed: %s", e)
    finally:
        _launch_proc = None


# ── ROS subscribers + publisher (bridge to vendored cpp node) ───────────────
def _ros_thread_main() -> None:
    """rclpy node hosting:
      * publisher  /graspnet/grasps      (graspnet_msgs/msg/GraspPose)
      * subscriber /arm/arm_status       (piper_msgs/msg/PiperStatusMsg)
    """
    global _ros_node, _grasps_pub, _last_arm_status_value, _last_arm_status_stamp

    import rclpy                                              # noqa: E402
    from rclpy.node import Node                               # noqa: E402
    from graspnet_msgs.msg import GraspPose                   # noqa: E402
    from piper_msgs.msg import PiperStatusMsg                 # noqa: E402

    rclpy.init(args=None)
    node = Node("piper_moveit_bridge")
    _ros_node = node

    _grasps_pub = node.create_publisher(GraspPose, "/graspnet/grasps", 10)

    def _arm_status_cb(msg):
        global _last_arm_status_value, _last_arm_status_stamp
        with _arm_status_lock:
            _last_arm_status_value = int(msg.arm_status)
            _last_arm_status_stamp = time.monotonic()
    node.create_subscription(
        PiperStatusMsg, "/arm/arm_status", _arm_status_cb, 10)

    log.info("rclpy bridge up: /graspnet/grasps publisher, /arm/arm_status subscriber")

    while not _ros_stop_evt.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    log.info("rclpy bridge thread exited")


def _wait_for_arm_status(timeout_s: float) -> bool:
    """Sentinel: wait for the FIRST /arm/arm_status message. Confirms
    piper_ctl_rbnx is publishing (i.e. piper_ctl is fully ACTIVE) AND
    our subscriber is wired."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with _arm_status_lock:
            if _last_arm_status_value is not None:
                return True
        time.sleep(0.1)
    return False


def _wait_for_motion_complete(busy_timeout_s: float, idle_timeout_s: float
                              ) -> tuple[bool, str]:
    """After publishing a grasp pose to /graspnet/grasps, wait for:
      Phase 1: arm_status transitions to busy (non-zero) within
               busy_timeout_s — proves the C++ node consumed our
               GraspPose and started a motion. Without this check,
               an arm that's already idle would have its "still 0"
               read mistaken for "motion completed".
      Phase 2: arm_status returns to ARM_STATUS_IDLE (0) within
               idle_timeout_s — motion done, gripper closed, etc.

    Returns (success, reason).
    """
    pub_t = time.monotonic()
    busy_deadline = pub_t + busy_timeout_s

    # Phase 1: wait for busy.
    while time.monotonic() < busy_deadline:
        with _arm_status_lock:
            v = _last_arm_status_value
        if v is not None and v != ARM_STATUS_IDLE:
            log.info("arm went busy (status=%d) after %.2fs",
                     v, time.monotonic() - pub_t)
            break
        time.sleep(0.05)
    else:
        # Never went busy. Two interpretations:
        #   a) C++ node didn't get our publish (subscriber not up yet)
        #   b) The arm was already physically at the goal pose (no
        #      motion needed) — MoveIt's "trivially solved" path.
        # Conservative: fail. Caller can retry.
        return False, (
            f"timeout: /arm/arm_status never transitioned to busy "
            f"within {busy_timeout_s:.1f}s of publishing GraspPose "
            f"(C++ moveit_control_node_yolo did not consume the message, "
            f"or the goal was already satisfied)")

    # Phase 2: wait for return to idle.
    idle_deadline = time.monotonic() + idle_timeout_s
    while time.monotonic() < idle_deadline:
        with _arm_status_lock:
            v = _last_arm_status_value
        if v == ARM_STATUS_IDLE:
            return True, "ok"
        time.sleep(0.05)
    return False, (
        f"timeout: /arm/arm_status did not return to idle (=0) "
        f"within {idle_timeout_s:.1f}s of busy transition")


# ── lifecycle ───────────────────────────────────────────────────────────────
@piper_moveit.on_init
def init(cfg):
    """Driver(CMD_INIT). Heavy:
      1. parse cfg
      2. log atlas deps (informational)
      3. spawn rclpy bridge thread (publisher + status subscriber)
      4. spawn ros2 launch (move_group + RSP + cpp executor)
      5. sentinel-wait for first /arm/arm_status sample
    """
    global _initialized, _resolved_cfg, _launch_proc

    with _state_lock:
        if _initialized:
            return Ok()

    cfg = cfg or {}
    if isinstance(cfg, str):
        try:
            cfg = json.loads(cfg) if cfg else {}
        except json.JSONDecodeError as e:
            return Err(f"bad config_json: {e}")
    _resolved_cfg = cfg

    _log_atlas_deps()

    # 1. Bring up rclpy bridge BEFORE the launch — so the moment cpp
    #    starts publishing /arm/arm_status etc., we already have
    #    the subscriber wired (no race).
    global _ros_thread
    _ros_stop_evt.clear()
    _ros_thread = threading.Thread(
        target=_ros_thread_main, name="piper_moveit-ros", daemon=True)
    _ros_thread.start()
    time.sleep(0.5)  # let rclpy.init + create_publisher land

    # 2. Spawn the ros2 launch.
    try:
        _launch_proc = _spawn_launch(cfg)
    except Exception as e:  # noqa: BLE001
        _ros_stop_evt.set()
        return Err(f"spawn launch failed: {e}")

    # 3. Sentinel wait. /arm/arm_status comes from piper_ctl_rbnx, so
    #    this also acts as a "piper_ctl is ACTIVE" check. The launch
    #    itself takes ~5-10s to bring up move_group; we're patient.
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 60.0))
    if not _wait_for_arm_status(sentinel_timeout):
        _kill_launch()
        _ros_stop_evt.set()
        return Err(
            f"sentinel: no /arm/arm_status sample within {sentinel_timeout:.1f}s "
            "(is piper_ctl_rbnx ACTIVE? Check `rbnx caps | grep piper_ctl`)")

    with _state_lock:
        _initialized = True
    log.info("init complete: move_group + cpp executor live, "
             "/arm/arm_status flowing, execute_grasp MCP exposed")
    return Ok()


@piper_moveit.on_deactivate
def deactivate():
    """ACTIVE → INACTIVE. Tear down the launch group + rclpy thread."""
    log.info("CMD_DEACTIVATE: killing launch and rclpy thread")
    _kill_launch()
    _ros_stop_evt.set()
    if _ros_thread is not None:
        _ros_thread.join(timeout=5.0)
    with _state_lock:
        global _initialized
        _initialized = False
    return Ok()


# ── atlas-routed MCP handler (Pilot's view) ─────────────────────────────────
from manipulation_mcp import (  # noqa: E402  pylint: disable=wrong-import-position
    ExecuteGrasp_Request, ExecuteGrasp_Response,
)


@piper_moveit.mcp("robonix/service/manipulation/execute_grasp")
def execute_grasp(req: ExecuteGrasp_Request) -> ExecuteGrasp_Response:
    """Plan + execute a grasp motion on the Piper arm.

    Use this tool when an upstream service (yolo_grasp_rbnx) has
    produced a target grasp pose and you want the arm to actually
    move there + close the gripper.

    Inputs:
      target_pose    — geometry_msgs/PoseStamped, frame_id typically
                       camera_color_optical_frame.
      gripper_width  — meters, 0..0.07. Caller (yolo_grasp) chooses
                       based on object dimensions.
      timeout_s      — total budget for plan+execute. Default 20s.

    Returns success=True iff the arm moved through the planned
    trajectory and PiperStatusMsg.arm_status returned to 0 (idle).
    """
    with _state_lock:
        if not _initialized:
            return ExecuteGrasp_Response(
                success=False, message="piper_moveit not initialized",
                elapsed_s=0.0)
        if _grasps_pub is None or _ros_node is None:
            return ExecuteGrasp_Response(
                success=False,
                message="rclpy bridge not ready (race? retry after 1s)",
                elapsed_s=0.0)

    # Publish the grasp pose to wake up the C++ moveit_control_node_yolo.
    try:
        from graspnet_msgs.msg import GraspPose  # noqa: E402
        from geometry_msgs.msg import (
            PoseStamped, Pose, Point, Quaternion)  # noqa: E402

        # The MCP request carries a typed PoseStamped (codegen
        # dataclass). Re-pack into the ROS msg type.
        ts_in  = req.target_pose
        ros_ps = PoseStamped()
        ros_ps.header.stamp    = _ros_node.get_clock().now().to_msg()
        ros_ps.header.frame_id = ts_in.header.frame_id
        ros_ps.pose = Pose(
            position=Point(
                x=float(ts_in.pose.position.x),
                y=float(ts_in.pose.position.y),
                z=float(ts_in.pose.position.z)),
            orientation=Quaternion(
                x=float(ts_in.pose.orientation.x),
                y=float(ts_in.pose.orientation.y),
                z=float(ts_in.pose.orientation.z),
                w=float(ts_in.pose.orientation.w)),
        )
        gp = GraspPose()
        gp.target_pose   = ros_ps
        gp.gripper_width = float(req.gripper_width)
    except Exception as e:  # noqa: BLE001
        return ExecuteGrasp_Response(
            success=False, message=f"build GraspPose failed: {e}",
            elapsed_s=0.0)

    t0 = time.monotonic()
    _grasps_pub.publish(gp)
    log.info("published GraspPose (frame=%s, gripper_width=%.3f); "
             "waiting for arm motion to complete",
             ros_ps.header.frame_id, gp.gripper_width)

    # Split the caller-provided timeout into phase budgets:
    #   busy: at most ⅓ — usually << 1s for the cpp to plan.
    #   idle: the rest — execution.
    total_to = float(req.timeout_s) if req.timeout_s > 0 else 20.0
    busy_to  = min(5.0, total_to * 0.33)
    idle_to  = max(1.0, total_to - busy_to)

    success, reason = _wait_for_motion_complete(busy_to, idle_to)
    elapsed = time.monotonic() - t0

    log.info("execute_grasp result: success=%s reason=%s elapsed=%.2fs",
             success, reason, elapsed)
    return ExecuteGrasp_Response(
        success=bool(success),
        message=reason,
        elapsed_s=float(elapsed),
    )


def main() -> int:
    def _on_signal(sig, _frame):
        log.info("signal %d — shutting down", sig)
        _kill_launch()
        _ros_stop_evt.set()
        if _ros_thread is not None:
            _ros_thread.join(timeout=3.0)
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT,  _on_signal)
    try:
        piper_moveit.run()
    finally:
        _kill_launch()
        _ros_stop_evt.set()
        if _ros_thread is not None:
            _ros_thread.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
