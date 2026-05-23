"""piper_moveit_rbnx.launch.py — robonix-aware fork of upstream
piper_with_gripper_moveit/launch/piper_moveit.launch.py.

Differences from upstream (`grasp/driver/piper_ros/.../piper_moveit.launch.py`,
which IS the version known to work):

  * NO `rviz2` — robonix-managed deploy is headless. Operator can
    `ros2 launch piper_with_gripper_moveit moveit_rviz.launch.py`
    in another shell if visualisation needed.

  * `move_group` `joint_states` remap target switched from
    `/arm/joint_states` → `/arm/joint_states_single`. This is what
    piper_ctl_rbnx actually publishes (verified
    piper_ctrl_single_node.py:43); the upstream's `/arm/joint_states`
    is what the fake `ros2_control_node` writes back, but we want
    move_group to listen to the REAL hardware feedback.

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

KEY DECISION — KEEP the fake `ros2_control_node` + spawn_controllers
=====================================================================
Earlier iterations of this file tried to drop these two on the theory
that they would clash with piper_ctl_rbnx's real hardware publisher.
That's wrong, and it broke trajectory execution with:

    [move_group] Action client not connected to action server:
                 arm_controller/follow_joint_trajectory
    [move_group] Failed to send trajectory part 1 of 1 to controller
                 arm_controller

Why we need them:

  * MoveIt's SimpleControllerManager (configured in
    config/moveit_controllers.yaml) expects two FollowJointTrajectory
    action servers named `arm_controller/follow_joint_trajectory` and
    `gripper_controller/follow_joint_trajectory`. These are advertised
    by ros2_control's joint_trajectory_controller (spawned by
    spawn_controllers), running on top of the
    `mock_components/GenericSystem` plugin defined in
    `piper.ros2_control.xacro`.

  * The C++ moveit_control_node_yolo calls
    MoveGroupInterface::execute() — that internally goes through
    SimpleControllerManager → arm_controller's follow_joint_trajectory
    action. Without the fake controllers, execute() fails immediately.

  * The fake controllers do NOT touch the real hardware. The flow is:
      1. moveit_control_node_yolo plans + executes via MoveGroupInterface.
      2. MoveGroupInterface sends the trajectory to arm_controller's
         FollowJointTrajectory action (the fake one).
      3. The fake controller "runs" the trajectory in its mock joint
         model and writes joint feedback to /joint_states.
      4. (Separately) moveit_control_node_yolo's pose-publishing logic
         pushes commands to piper_ctl through the existing pos_cmd /
         joint_ctrl pathway, which is what actually moves the arm.

  * Topic isolation: the fake controller writes joint state to a
    dedicated topic `/piper_moveit_rbnx/fake_joint_states` (remap
    below). It is NOT remapped to /arm/joint_states_single — that
    would let fake (zero) joint feedback overwrite real hardware
    feedback. move_group is configured to subscribe to
    /arm/joint_states_single (the real hardware publisher), so the
    fake's writes never reach planning.

This mirrors EXACTLY what upstream piper_moveit.launch.py does
(piper_ros/src/piper_moveit/piper_with_gripper_moveit/launch/
piper_moveit.launch.py), with the joint_states subscription target
swapped to the real-hardware topic.

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
    # See file header for the full rationale. Short version: MoveIt's
    # SimpleControllerManager NEEDS arm_controller's follow_joint_trajectory
    # action server to exist or execute() fails. The mock GenericSystem
    # provides it. Topic remap keeps the fake's joint feedback from
    # leaking onto the real hardware's /arm/joint_states_single topic.
    ld.add_action(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="screen",
        remappings=[
            # The fake controller's joint_state_broadcaster writes
            # /joint_states by default. We isolate it to a private topic
            # to avoid clashing with piper_ctl_rbnx's real publisher.
            # move_group subscribes to /arm/joint_states_single (the
            # REAL hardware feedback), not this one.
            ("joint_states", "/piper_moveit_rbnx/fake_joint_states"),
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
        "end_effector_link", default_value="arm/link6",
        description="EE link in the prefixed TF tree"))

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
