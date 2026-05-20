from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()
    
    ld = LaunchDescription()
    
    # Robot State Publisher with frame_prefix
    # Note: frame_prefix should be "arm/" (without leading slash, with trailing slash)
    # This will make "world" -> "arm/world", "base_link" -> "arm/base_link"
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"frame_prefix": "arm/"},
        ],
        remappings=[
            # Subscribe joint_states from /arm/joint_states
            ("joint_states", "/arm/joint_states"),
        ],
    )
    ld.add_action(robot_state_publisher)

    # Provide a global "world" frame so MoveIt lookups succeed when all robot TFs are prefixed with "arm/"
    static_world_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="arm_world_to_world_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "world", "arm/world"],
        output="screen",
    )
    ld.add_action(static_world_bridge)
    
    return ld
