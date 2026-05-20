"""
Launch file for test control node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for test control node
    """
    
    # Declare arguments
    target_x_arg = DeclareLaunchArgument(
        'target_x',
        default_value='0.3',
        description='Target X position in meters'
    )
    
    target_y_arg = DeclareLaunchArgument(
        'target_y',
        default_value='0.0',
        description='Target Y position in meters'
    )
    
    target_z_arg = DeclareLaunchArgument(
        'target_z',
        default_value='0.3',
        description='Target Z position in meters'
    )
    
    gripper_width_arg = DeclareLaunchArgument(
        'gripper_width',
        default_value='0.05',
        description='Gripper opening width in meters (0.0 to 0.07)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='Reference frame for target pose'
    )
    
    # Create node
    test_control_node = Node(
        package='piper_moveit_control',
        executable='test_control_node',
        name='test_control_node',
        output='screen',
        parameters=[{
            'target_x': LaunchConfiguration('target_x'),
            'target_y': LaunchConfiguration('target_y'),
            'target_z': LaunchConfiguration('target_z'),
            'gripper_width': LaunchConfiguration('gripper_width'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )
    
    return LaunchDescription([
        target_x_arg,
        target_y_arg,
        target_z_arg,
        gripper_width_arg,
        frame_id_arg,
        test_control_node,
    ])


