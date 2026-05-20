from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()
    launch_package_path = moveit_config.package_path
    ld = LaunchDescription()
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    # Start move_group
    my_generate_move_group_launch(ld, moveit_config)
    # Start rviz
    my_generate_moveit_rviz_launch(ld, moveit_config)

    # Add ros2_control node (Fake joint driver for simulation/demo)
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            output="screen",
            remappings=[
                # Remap joint_states from controllers into the namespaced topic
                ("joint_states", "/arm/joint_states"),
            ],
        )
    )

    # Spawn controllers
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    return ld


def my_generate_move_group_launch(ld, moveit_config):

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        # Subscribe to joint_states with /arm/ prefix
        "joint_states_topic": "/arm/joint_states",
    }

    # Get moveit config dict and override planning_frame
    moveit_config_dict = moveit_config.to_dict()
    # Override planning_frame to use arm/world since we use frame_prefix
    # MoveIt uses planning_scene_monitor.planning_frame parameter
    if "planning_scene_monitor" in moveit_config_dict:
        if isinstance(moveit_config_dict["planning_scene_monitor"], dict):
            moveit_config_dict["planning_scene_monitor"]["planning_frame"] = "arm/world"
        else:
            # If it's not a dict, create it
            moveit_config_dict["planning_scene_monitor"] = {"planning_frame": "arm/world"}
    else:
        moveit_config_dict["planning_scene_monitor"] = {"planning_frame": "arm/world"}
    # Also set at top level (some MoveIt versions use this)
    moveit_config_dict["planning_frame"] = "arm/world"
    
    # Add planning_frame to move_group_configuration as well
    move_group_configuration["planning_frame"] = "arm/world"
    move_group_configuration["planning_scene_monitor"] = {"planning_frame": "arm/world"}
    
    move_group_params = [
        moveit_config_dict,
        move_group_configuration,
    ]
    move_group_params.append({"use_sim_time": False})

    # Use remap to add /arm/ prefix to move_group topics
    # Keep node and services in global namespace so rviz can connect
    # Only remap topics that need /arm/ prefix
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=move_group_params,
        remappings=[
            # Subscribe joint_states from /arm/joint_states
            ("joint_states", "/arm/joint_states"),
            # Keep other topics in global namespace for rviz compatibility
            # Only joint_states needs /arm/ prefix to match the driver
        ],
    )
    ld.add_action(move_group_node)
    return ld

def my_generate_moveit_rviz_launch(ld, moveit_config):
    """Launch file for rviz"""

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]
    rviz_parameters.append({"use_sim_time": False})

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return ld
