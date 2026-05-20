# Piper MoveIt Control (C++)

C++ ROS2 node for controlling Piper robot arm with gripper using MoveIt.

## Features

- Subscribes to target pose and gripper width commands
- Uses MoveIt MoveGroupInterface for motion planning and execution
- Controls gripper via action interface
- Simple and efficient C++ implementation

## Building

```bash
cd /home/syswonder/lgw/arm
colcon build --packages-select piper_moveit_control
source install/setup.bash
```

## Usage

### 1. Launch MoveIt

```bash
ros2 launch piper_with_gripper_moveit piper_moveit.launch.py
```

### 2. Launch Control Node

```bash
ros2 launch piper_moveit_control moveit_control.launch.py
```

Or run directly:

```bash
ros2 run piper_moveit_control moveit_control_node
```

### 3. Send Commands

#### Option A: Use Test Node (Recommended)

Run the test node to move to a specific position:

```bash
# Move to default position (0.3, 0.0, 0.3)
ros2 run piper_moveit_control test_control_node

# Move to custom position
ros2 run piper_moveit_control test_control_node --ros-args \
  -p target_x:=0.35 \
  -p target_y:=0.1 \
  -p target_z:=0.25 \
  -p gripper_width:=0.06
```

Or use the launch file:

```bash
# Default position
ros2 launch piper_moveit_control test_control.launch.py

# Custom position
ros2 launch piper_moveit_control test_control.launch.py \
  target_x:=0.35 target_y:=0.1 target_z:=0.25 gripper_width:=0.06
```

#### Option B: Use Command Line

```bash
ros2 topic pub --once /target_pose piper_moveit_control/msg/TargetPose "
target_pose:
  header:
    frame_id: 'base_link'
  pose:
    position: {x: 0.3, y: 0.0, z: 0.3}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
gripper_width: 0.05
"
```

### C++ Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include "piper_moveit_control/msg/target_pose.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("example_publisher");
  
  auto publisher = node->create_publisher<piper_moveit_control::msg::TargetPose>(
      "target_pose", 10);
  
  auto msg = piper_moveit_control::msg::TargetPose();
  msg.target_pose.header.frame_id = "base_link";
  msg.target_pose.pose.position.x = 0.3;
  msg.target_pose.pose.position.y = 0.0;
  msg.target_pose.pose.position.z = 0.3;
  msg.target_pose.pose.orientation.w = 1.0;
  msg.gripper_width = 0.05;
  
  publisher->publish(msg);
  
  rclcpp::shutdown();
  return 0;
}
```

### Python Example

```python
import rclpy
from rclpy.node import Node
from piper_moveit_control.msg import TargetPose

def main():
    rclpy.init()
    node = Node('example_publisher')
    pub = node.create_publisher(TargetPose, 'target_pose', 10)
    
    msg = TargetPose()
    msg.target_pose.header.frame_id = 'base_link'
    msg.target_pose.pose.position.x = 0.3
    msg.target_pose.pose.position.y = 0.0
    msg.target_pose.pose.position.z = 0.3
    msg.target_pose.pose.orientation.w = 1.0
    msg.gripper_width = 0.05
    
    pub.publish(msg)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Message Definition

### TargetPose.msg

```
geometry_msgs/PoseStamped target_pose
float32 gripper_width  # 0.0 (closed) to 0.07 (open) meters
```

## Parameters

- `arm_group_name` (default: "arm"): MoveIt planning group name
- `gripper_action_name` (default: "/gripper_controller/follow_joint_trajectory"): Gripper action server
- `end_effector_link` (default: "link6"): End effector link name

## Topics

### Subscribed

- `/target_pose` (piper_moveit_control/msg/TargetPose): Target pose and gripper width

## Dependencies

- rclcpp
- rclcpp_action
- moveit_ros_planning_interface
- geometry_msgs
- control_msgs
- trajectory_msgs

## Notes

- Gripper width range: 0.0 (closed) to 0.07 (fully open) meters
- The node will ignore new commands while executing a previous command
- Make sure MoveIt is running before starting this node

