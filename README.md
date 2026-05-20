# piper_moveit_rbnx

Robonix package for arm motion planning + execution on the Piper +
Orbbec Dabai DCW grasp pipeline. Stage 5 of the migration.

## What it does

Owns `robonix/service/manipulation/*`. Spawns the upstream MoveIt
move_group + the C++ `moveit_control_node_yolo` grasp executor, and
puts a typed atlas-routed `execute_grasp` MCP tool on top.

```
                 Pilot LLM (or pick_skill_rbnx, Stage 6)
                          │
                          ▼
          MCP   manipulation/execute_grasp  (PoseStamped + width)
                          │
                          ▼
              ┌─ publishes /graspnet/grasps  (graspnet_msgs/GraspPose)
              │
              ▼
       C++ moveit_control_node_yolo  (vendored)
              │
              ├─ tf2 transform → arm/base_link
              ├─ MoveGroupInterface plan
              └─ execute via FollowJointTrajectory action
                          │
                          ▼
              piper_ctl_rbnx executes on real hardware
                          │
                          ▼
              piper_ctl publishes /arm/arm_status (PiperStatusMsg)
                          │
                          ▼
   ◄─── execute_grasp polls busy → idle, returns success/elapsed
```

## Why this design (most-conservative path)

The C++ `moveit_control_node_yolo` is battle-tested upstream. The
MoveIt `MoveGroupInterface` Python API is less stable than its C++
counterpart. Re-implementing the planning + execution logic in
Python carries real risk for zero gain.

So we **vendor the C++ source verbatim** and put a typed front door
on it via ROS topic + status poll:

| component | provenance | what we did |
|---|---|---|
| MoveIt config (URDF/SRDF/...) | upstream `piper_with_gripper_moveit` | vendor verbatim, colcon-build |
| `moveit_control_node_yolo` | upstream `piper_moveit_control` | vendor verbatim, colcon-build |
| Top-level launch | upstream `piper_moveit.launch.py` | **fork** (see below) |
| atlas Service wrapper | new (`piper_moveit/main.py`) | only Python we own |

### Launch fork (`launch/piper_moveit_rbnx.launch.py`)

Three deltas from upstream `piper_moveit.launch.py`:

1. **Removed fake `ros2_control_node`** — upstream's launch spawns
   a simulated joint driver publishing `/arm/joint_states`. With
   piper_ctl_rbnx already publishing real hardware joint states,
   running both = two publishers fighting on the topic = MoveIt
   plans against garbage.

2. **Removed RViz** — robonix-managed deploy is headless. Bring up
   RViz manually in another shell if needed:
   ```
   ros2 launch piper_with_gripper_moveit moveit_rviz.launch.py
   ```

3. **`move_group joint_states` remap target switched** from
   `/arm/joint_states` (which had no real publisher after we
   removed the fake controller) to `/arm/joint_states_single`
   (what piper_ctl_rbnx actually publishes — verified
   `piper_ctrl_single_node.py:43`).

4. **Added `link6 → arm/link6` static TF bridge** to glue the
   unprefixed TF tree (Stage 3A piper_description_rbnx publishes,
   easy_handeye2_rbnx publishes its `link6 → camera_color_optical_frame`
   into) onto the prefixed one (this launch publishes via
   `frame_prefix="arm/"` for MoveIt's planning frame). Without
   the bridge, the C++ node's
   `tf_buffer_->transform(grasp_pose_in_camera_frame, "arm/base_link")`
   call fails because the camera frame and `arm/base_link` are in
   different subtrees.

The rest (`rsp.launch.py`, `static_virtual_joint_tfs.launch.py`,
`spawn_controllers` is intentionally NOT included — see below) we
include verbatim from the upstream package via
`IncludeLaunchDescription`, so future upstream tweaks don't need
us to re-fork.

### Why no `spawn_controllers.launch.py`?

Upstream uses `spawn_controllers` to start `joint_trajectory_controller`
+ `gripper_controller` ros2_control instances. Those drive the **fake**
controller_manager we removed. The C++ node doesn't talk to
controller_manager — it submits FollowJointTrajectory action goals
directly to whoever advertises the action server. **piper_ctl_rbnx
must advertise that action server** for the real-hardware path to
work. (TODO: verify in piper_ctl_rbnx after Stage 5 boots — if it
doesn't, we need a thin "FollowJointTrajectory action server bridge"
inside piper_ctl_rbnx that converts action goals into `pos_cmd`
publishes. That's a piper_ctl_rbnx change, not a piper_moveit_rbnx
issue.)

## Frame tree (post-Stage-5 boot)

Two parallel subtrees, joined at `link6 ↔ arm/link6`:

```
unprefixed (Stage 3A + 3B):                    prefixed (this package):
    base_link                                       arm/world  ──┐
       └── link1 ── ... ── link6                    arm/base_link │ static
                              ├── camera_*          arm/link1     │ joints
                              │                     ...           │ from URDF
                              └── arm/link6 ◄══════ arm/link6 ────┘
                                  (identity static TF — the bridge)
```

Both subtrees describe the **same physical robot**, just under
different frame-name conventions. The bridge means
`tf_buffer_->transform(camera_color_optical_frame_pose, arm/base_link)`
walks: camera_* → link6 → arm/link6 → arm/base_link, and works.

## Architecture

```
piper_moveit_rbnx/
├── package_manifest.yaml
├── capabilities/
│   ├── service/manipulation/{driver,execute_grasp}.v1.toml
│   └── lib/manipulation/srv/ExecuteGrasp.srv
├── piper_moveit/
│   ├── __init__.py
│   └── main.py                       # robonix Service + rclpy bridge
├── launch/
│   └── piper_moveit_rbnx.launch.py   # fork of upstream launch
├── scripts/
│   ├── build.sh                      # colcon + rbnx codegen --mcp
│   └── start.sh
└── src/                              # vendored ROS packages
    ├── piper_moveit/                 # piper_with_gripper_moveit + piper_no_gripper_moveit
    ├── piper_moveit_control/         # C++ moveit_control_node_yolo
    ├── piper_humble/                 # empty shell (referenced by cmakelists)
    ├── piper_msgs/                   # PiperStatusMsg etc.
    └── graspnet_msgs/                # GraspPose etc. (full version)
```

## Lifecycle

```
on_init  ──► parse cfg
       ──► log atlas deps (informational)
       ──► spawn rclpy bridge thread
            (publisher /graspnet/grasps,
             subscriber /arm/arm_status)
       ──► spawn `ros2 launch piper_moveit_rbnx.launch.py`
            (RSP + move_group + cpp executor)
       ──► sentinel-wait for first /arm/arm_status sample
            (proves piper_ctl_rbnx is alive)

on_deactivate ──► kill launch group + stop rclpy thread
```

`on_init` is heavy — sentinel timeout default 60 s because launch
takes ~5–10 s to bring up move_group, and we need real piper_ctl
joint feedback to flow before we declare ourselves ACTIVE.

## Config

```yaml
service:
  - name: piper_moveit
    config:
      arm_group_name:       arm                              # MoveIt SRDF group
      end_effector_link:    arm/link6                        # in prefixed tree
      gripper_action_name:  /gripper_controller/follow_joint_trajectory
      sentinel_timeout_s:   60.0                             # wait for /arm/arm_status
```

All four are optional (defaults match upstream). `end_effector_link`
intentionally uses the prefixed `arm/link6` — MoveIt's planning
frame is `arm/world`, so the EE link lives in the prefixed subtree.

## execute_grasp semantics

Two-phase wait:

1. **busy phase** (≤ ⅓ × `timeout_s`): wait for `/arm/arm_status`
   to transition to non-zero (i.e. arm started moving). If never:
   either the C++ node didn't pick up our publish, or the goal was
   already physically satisfied → fail conservatively.

2. **idle phase** (remaining budget): wait for return to 0 (idle).

`success=True` iff both phases complete in budget.

## Build / run

```bash
# Standalone
cd /Users/howenliu/lab/packages/piper_moveit_rbnx
bash scripts/build.sh   # colcon build the cpp executor (~30–60 s)

# Integrated
cd /Users/howenliu/lab/piper_grasp_deploy
rbnx boot
```

## Verification (in order)

```bash
# 1. atlas-side
rbnx caps | grep manipulation
# expect: piper_moveit ACTIVE, contracts:
#   robonix/service/manipulation/driver, robonix/service/manipulation/execute_grasp

# 2. ROS-side (the cpp bridge is up)
ros2 topic info /graspnet/grasps                 # expect 1+ subscribers (cpp node)
ros2 node list | grep moveit_control_lgw_node    # the cpp node

# 3. TF bridge
ros2 run tf2_ros tf2_echo arm/base_link camera_color_optical_frame
# Expect a transform composed of:
#   arm/base_link ← arm/joint{1..6} ← arm/link6  (RSP, prefixed)
#   arm/link6 ←IDENTITY← link6                   (the bridge)
#   link6 ← <calib> ← camera_color_optical_frame (easy_handeye2)

# 4. Direct execute_grasp test (CAREFUL — this moves the real arm).
#    Park the arm in safe position first.
#    Use a fake target ~30cm above current EE pose, no rotation:
rbnx ask "move the arm 30cm up, no rotation"
# (with manipulation/execute_grasp ACTIVE, pilot can call it
#  directly with a synthetic PoseStamped)
```

## Failure modes

| symptom | cause | fix |
|---|---|---|
| `sentinel: no /arm/arm_status sample within 60s` | piper_ctl_rbnx not ACTIVE, or `auto_can_setup` failed | `rbnx caps \| grep piper_ctl` to confirm; bring up CAN manually if needed |
| `execute_grasp: timeout: arm_status never went busy` | C++ node didn't subscribe `/graspnet/grasps`; or move_group rejected the goal silently | check `ros2 topic info /graspnet/grasps` for subs; check `move_group` logs for plan failure |
| MoveIt plans against wrong joint values | `/arm/joint_states_single` remap not taking effect | check launch fork section in `launch/piper_moveit_rbnx.launch.py` — verify the `joint_states_topic` in move_group_configuration |
| TF lookup error in cpp `arm/base_link → camera_color_optical_frame` | Static bridge `link6 ↔ arm/link6` not running, OR easy_handeye2 calib not staged | check `ros2 run tf2_ros tf2_echo link6 arm/link6` is identity; check easy_handeye2_rbnx is ACTIVE |
| `FollowJointTrajectory action server not advertised` | piper_ctl_rbnx doesn't expose this action (TODO above) | bridge work in piper_ctl_rbnx — out of scope for Stage 5 |

## Coupling with neighbors

* **Upstream** piper_ctl_rbnx (Stage 2) — provides `/arm/arm_status`
  + `/arm/joint_states_single` + (TODO) FollowJointTrajectory action
  servers for arm and gripper.
* **Upstream** piper_description_rbnx (Stage 3A) — provides
  unprefixed RSP TF tree.
* **Upstream** easy_handeye2_rbnx (Stage 3B) — provides
  `link6 → camera_color_optical_frame` static TF.
* **Upstream** yolo_grasp_rbnx (Stage 4B) — when fully wired,
  publishes `/graspnet/grasps` on its own (parallel publisher to us)
  for the legacy direct path. This package is independent of
  yolo_grasp; both can coexist publishing to the same topic.
* **Downstream** pick_skill_rbnx (Stage 6) — will call
  `manipulation/execute_grasp` over MCP.

So deploy ordering:
```
piper_ctl ── piper_description ── easy_handeye2 ── yolo_world ── yolo_grasp ── piper_moveit ── pick_skill
```

`piper_moveit` MUST come AFTER `piper_ctl` (sentinel) and
`easy_handeye2` (TF bridge dep). Order vs `yolo_world` /
`yolo_grasp` doesn't matter — manipulation has no perception
dependency at boot time, only at runtime.
