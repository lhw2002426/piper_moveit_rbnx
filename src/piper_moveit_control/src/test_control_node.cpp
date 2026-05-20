#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "graspnet_msgs/msg/grasp_pose.hpp"

using namespace std::chrono_literals;

class TestControlNode : public rclcpp::Node
{
public:
  TestControlNode() : Node("test_control_node")
  {
    // Declare parameters for target position
    this->declare_parameter("target_x", 0.040);
    this->declare_parameter("target_y", 0.0);
    this->declare_parameter("target_z", 0.394);
    this->declare_parameter("gripper_width", 0.05);
    this->declare_parameter("frame_id", "base_link");

    // Get parameters
    target_x_ = this->get_parameter("target_x").as_double();
    target_y_ = this->get_parameter("target_y").as_double();
    target_z_ = this->get_parameter("target_z").as_double();
    gripper_width_ = this->get_parameter("gripper_width").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // Create publisher
    publisher_ = this->create_publisher<graspnet_msgs::msg::GraspPose>(
        "/graspnet/grasps", 10);

    RCLCPP_INFO(this->get_logger(), "Test Control Node initialized");
    RCLCPP_INFO(this->get_logger(), "Target position: [%.3f, %.3f, %.3f]", 
                target_x_, target_y_, target_z_);
    RCLCPP_INFO(this->get_logger(), "Gripper width: %.3f m", gripper_width_);
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());

    // Create a timer to send command after a short delay
    timer_ = this->create_wall_timer(
        2s, std::bind(&TestControlNode::sendTargetPose, this));
  }

private:
  void sendTargetPose()
  {
    // Cancel timer after first execution
    timer_->cancel();

    auto msg = graspnet_msgs::msg::GraspPose();
    
    // Set header
    msg.target_pose.header.frame_id = frame_id_;
    msg.target_pose.header.stamp = this->get_clock()->now();
    
    // Set position
    msg.target_pose.pose.position.x = target_x_;
    msg.target_pose.pose.position.y = target_y_;
    msg.target_pose.pose.position.z = target_z_;
    
    // Set orientation (gripper pointing in +X direction)
    // This is a 90-degree rotation around Z axis
    msg.target_pose.pose.orientation.x = 0.0;
    msg.target_pose.pose.orientation.y = 0.676;
    msg.target_pose.pose.orientation.z = 0.0;  // sin(90°/2)
    msg.target_pose.pose.orientation.w = 0.737;  // cos(90°/2)
    
    // Set gripper width
    msg.gripper_width = gripper_width_;

    RCLCPP_INFO(this->get_logger(), "Publishing target pose...");
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Command sent successfully!");
    RCLCPP_INFO(this->get_logger(), "You can close this node with Ctrl+C");
  }

  // Parameters
  double target_x_;
  double target_y_;
  double target_z_;
  double gripper_width_;
  std::string frame_id_;

  // Publisher
  rclcpp::Publisher<graspnet_msgs::msg::GraspPose>::SharedPtr publisher_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TestControlNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}


