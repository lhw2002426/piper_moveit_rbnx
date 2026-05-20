"""
Launch file for MoveIt control node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for MoveIt control node
    """
    
    # Declare arguments
    arm_group_name_arg = DeclareLaunchArgument(
        'arm_group_name',
        default_value='arm',
        description='Name of the MoveIt planning group for the arm'
    )
    
    gripper_action_name_arg = DeclareLaunchArgument(
        'gripper_action_name',
        default_value='/gripper_controller/follow_joint_trajectory',
        description='Action name for gripper controller'
    )
    
    end_effector_link_arg = DeclareLaunchArgument(
        'end_effector_link',
        default_value='link6',
        description='Name of the end effector link'
    )
    
    # Create node
    moveit_control_node = Node(
        package='piper_moveit_control',
        executable='moveit_control_node',
        name='moveit_control_node',
        output='screen',
        parameters=[{
            'arm_group_name': LaunchConfiguration('arm_group_name'),
            'gripper_action_name': LaunchConfiguration('gripper_action_name'),
            'end_effector_link': LaunchConfiguration('end_effector_link'),
        }]
    )

    moveit_control_node_yolo = Node(
        package='piper_moveit_control',
        executable='moveit_control_node_yolo',
        name='moveit_control_node_yolo',
        output='screen',
        parameters=[{
            'arm_group_name': LaunchConfiguration('arm_group_name'),
            'gripper_action_name': LaunchConfiguration('gripper_action_name'),
            'end_effector_link': LaunchConfiguration('end_effector_link'),
        }]
    )
    
    return LaunchDescription([
        arm_group_name_arg,
        gripper_action_name_arg,
        end_effector_link_arg,
        # moveit_control_node,
        moveit_control_node_yolo,
    ])

