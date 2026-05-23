"""piper_moveit_rbnx.launch.py — robonix-aware fork of upstream
piper_with_gripper_moveit/launch/piper_moveit.launch.py.

Differences from upstream (`grasp/driver/piper_ros/.../piper_moveit.launch.py`,
which IS the version known to work):

  * NO `rviz2` — robonix-managed deploy is headless. Operator can
    `ros2 launch piper_with_gripper_moveit moveit_rviz.launch.py`
    in another shell if visualisation needed.

  * `move_group` `joint_states` remap target switched from
    `/arm/joint_states` → `/arm/joint_states_single`. This is what
    piper_ctl_rbnx actually publishes for FEEDBACK
    (piper_ctrl_single_node.py:43). The default upstream target
    `/arm/joint_states` is the COMMAND topic that piper_ctl_rbnx
    subscribes to to drive the hardware (see file's other comments).
    Mixing them up makes move_group plan against the fake controller's
    cmd echo instead of the real joint feedback.

  * Adds an additional static_transform_publisher
    `link6 → arm/link6` (identity), to bridge our two TF subtrees:
      ┌─ unprefixed  (Stage 3A piper_description_rbnx publishes):
      │   base_link → link1 → ... → link6
      │   link6 → camera_color_optical_frame  (easy_handeye2)
      └─ prefixed    (this launch publishes via frame_prefix="arm/"):
          arm/world → arm/base_link → arm/link1 → ... → arm/link6

    Without the bridge, the C++ moveit_control_node_yolo's
    `tf_buffer_->transform(grasp_pose_in_camera_frame, "arm/base_link")`
    fails: the camera frame is reachable in the unprefixed tree but
    arm/base_link only exists in the prefixed tree. The identity
    static TF connects the two at the `link6 / arm/link6` join point
    (which, by URDF + MoveIt+frame_prefix, are physically the same
    rigid body).

CRITICAL — KEEP the fake `ros2_control_node` + spawn_controllers
=====================================================================
Earlier iterations dropped these on the theory that they would clash
with piper_ctl_rbnx. That's wrong twice over:

  (a) MoveIt SimpleControllerManager (config/moveit_controllers.yaml)
      requires arm_controller's follow_joint_trajectory action server
      to exist; without it MoveGroupInterface::execute() bails with
      'Action client not connected'.

  (b) MORE IMPORTANTLY: the fake controller is the ONLY data path
      that actually drives the arm. Topology:

        moveit_control_node_yolo
            └─ MoveGroupInterface::execute(plan)
                 └─ SimpleControllerManager → arm_controller's
                    follow_joint_trajectory action
                        └─ joint_trajectory_controller writes
                           commanded joint angles into the mock
                           GenericSystem hardware
                                └─ joint_state_broadcaster publishes
                                   those angles on /joint_states
                                        └─ REMAPPED to /arm/joint_states
                                           └─ piper_ctl_rbnx's
                                              joint_callback subscribes
                                              to /arm/joint_states,
                                              converts to motor
                                              commands over CAN
                                                  └─ THE ARM MOVES.

      So the fake controller is NOT decorative. It IS the bridge from
      MoveIt to piper_ctl_rbnx.

  Topic legend:
      /arm/joint_states         — COMMAND topic. fake controller writes
                                  cmd; piper_ctl reads + drives hardware.
      /arm/joint_states_single  — FEEDBACK topic. piper_ctl writes live
                                  joint angles; move_group reads as the
                                  starting state for planning.
      Two distinct topics, different roles, no clash.

  Symptoms of getting this wrong (observed in the field):
   * Drop fake controller entirely
       → MoveGroupInterface::execute() fails: 'Action client not
         connected to action server: arm_controller/follow_joint_trajectory'.
   * Keep fake controller but remap joint_states to /piper_moveit_rbnx/
     fake_joint_states (an isolated sink)
       → execute() reports SUCCESS but the arm never physically moves
         (no one's listening to the sink topic to drive CAN).
   * Keep fake controller, remap to /arm/joint_states (current setup)
       → arm physically moves on every plan + execute. Correct.

  This mirrors EXACTLY what upstream piper_moveit.launch.py does
  (grasp/driver/piper_ros/.../piper_moveit.launch.py:56-61), with
  RViz removed and the planning-state subscription target swapped
  to the feedback topic.

Default launch args mirror upstream where possible.
"""
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "piper", package_name="piper_with_gripper_moveit"
    ).to_moveit_configs()
    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    # ── 1. virtual joints (upstream-provided; arm/world ←→ arm/base_link) ───
    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(virtual_joints_launch))))

    # ── 2. RSP (with frame_prefix="arm/") + world ↔ arm/world bridge ────────
    # Upstream's rsp.launch.py publishes both. Sourcing it via
    # IncludeLaunchDescription picks up future upstream tweaks for free.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_package_path / "launch/rsp.launch.py"))))

    # ── 3. extra TF bridge link6 → arm/link6 (see file header) ──────────────
    # Identity transform. The two link6 bodies are physically the same
    # rigid body — Stage 3A's URDF and our prefixed RSP both describe
    # it, just under different frame names.
    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="piper_moveit_link6_bridge",
        arguments=["0", "0", "0", "0", "0", "0", "link6", "arm/link6"],
        output="screen",
    ))

    # ── 4. move_group ───────────────────────────────────────────────────────
    _generate_move_group_launch(ld, moveit_config)

    # ── 5. fake ros2_control_node + spawn_controllers ───────────────────────
    # See file header for the full rationale. Short version: this is THE
    # bridge that actually drives the real arm. Topology:
    #
    #   moveit_control_node_yolo
    #       └─ MoveGroupInterface::execute(plan)
    #            └─ SimpleControllerManager → arm_controller's
    #               follow_joint_trajectory action
    #                   └─ joint_trajectory_controller writes commanded
    #                      joint angles into mock_components/GenericSystem
    #                          └─ joint_state_broadcaster publishes
    #                             those angles on /joint_states
    #                                └─ REMAPPED here to /arm/joint_states
    #                                   └─ piper_ctl_rbnx subscribes to
    #                                      /arm/joint_states (its
    #                                      joint_callback), converts to
    #                                      motor commands over CAN
    #                                          └─ THE ARM MOVES.
    #
    # i.e. this remap is NOT a cosmetic isolation. It IS the data path.
    # Earlier revs of this file remapped joint_states to a private
    # sink topic to "isolate" the fake controller from the real
    # publisher; that was wrong and is what made the arm look like
    # it executed but never physically move.
    #
    # piper_ctl_rbnx publishes its OWN topic /arm/joint_states_single
    # for live joint feedback (move_group subscribes to that one;
    # see _generate_move_group_launch below). The two topics are
    # different — no clash:
    #
    #     /arm/joint_states         ← fake controller writes (cmd)
    #                                 piper_ctl reads (drives hardware)
    #     /arm/joint_states_single  ← piper_ctl writes (feedback)
    #                                 move_group reads (current state)
    #
    # Kept exactly as upstream piper_moveit.launch.py does it
    # (grasp/driver/piper_ros/.../piper_moveit.launch.py:56-61).
    ld.add_action(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="screen",
        remappings=[
            ("joint_states", "/arm/joint_states"),
        ],
    ))

    # spawn_controllers reads `controller_names` from
    # moveit_controllers.yaml (arm_controller + gripper_controller)
    # and spawns each via `controller_manager/spawner`.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_package_path / "launch/spawn_controllers.launch.py"))))

    # ── 6. moveit_control_node_yolo (the C++ grasp executor) ────────────────
    # Vendored from piper_moveit_control. Subscribes /graspnet/grasps,
    # plans + executes via MoveGroupInterface.
    ld.add_action(DeclareLaunchArgument(
        "arm_group_name", default_value="arm",
        description="MoveIt planning group for the arm"))
    ld.add_action(DeclareLaunchArgument(
        "gripper_action_name",
        default_value="/gripper_controller/follow_joint_trajectory",
        description="Action name for gripper FollowJointTrajectory"))
    ld.add_action(DeclareLaunchArgument(
        "end_effector_link", default_value="link6",
        description=(
            "End effector link name as it appears in the URDF/SRDF "
            "robot model. MUST match the URDF link name (e.g. 'link6'), "
            "NOT the prefixed TF frame name (e.g. 'arm/link6'). MoveIt's "
            "PositionConstraint / OrientationConstraint look this up in "
            "the kinematic model — using the prefixed TF name causes "
            "'Link arm/link6 not found in model piper' + 'Unable to "
            "construct goal representation' + 'Catastrophic failure' "
            "during plan(). Upstream piper_ros's "
            "piper_moveit_control/launch/moveit_control.launch.py also "
            "uses 'link6' (verified against the working version)."),))

    moveit_control_node_yolo = Node(
        package="piper_moveit_control",
        executable="moveit_control_node_yolo",
        name="moveit_control_node_yolo",
        output="screen",
        parameters=[{
            "arm_group_name":     LaunchConfiguration("arm_group_name"),
            "gripper_action_name": LaunchConfiguration("gripper_action_name"),
            "end_effector_link":  LaunchConfiguration("end_effector_link"),
        }],
    )
    ld.add_action(moveit_control_node_yolo)

    return ld


def _generate_move_group_launch(ld, moveit_config):
    """Mirror of upstream's `my_generate_move_group_launch`, with the
    `joint_states` remap target switched from /arm/joint_states to
    /arm/joint_states_single (what piper_ctl_rbnx actually publishes).
    """
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg(
        "allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg(
        "publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        # Subscribe to joint_states from the real piper_ctl_rbnx
        # publisher — see file header.
        "joint_states_topic": "/arm/joint_states_single",
        "planning_frame": "arm/world",
        "planning_scene_monitor": {"planning_frame": "arm/world"},
    }

    # Inject planning_frame override into the moveit_config dict
    # itself (some MoveIt versions read it from there).
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["planning_frame"] = "arm/world"
    if not isinstance(moveit_config_dict.get("planning_scene_monitor"), dict):
        moveit_config_dict["planning_scene_monitor"] = {}
    moveit_config_dict["planning_scene_monitor"]["planning_frame"] = "arm/world"

    move_group_params = [
        moveit_config_dict,
        move_group_configuration,
        {"use_sim_time": False},
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=move_group_params,
        remappings=[
            ("joint_states", "/arm/joint_states_single"),
        ],
    )
    ld.add_action(move_group_node)
