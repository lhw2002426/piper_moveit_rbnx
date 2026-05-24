#include <memory>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "graspnet_msgs/msg/grasp_pose.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class MoveItControlNode : public rclcpp::Node
{
public:
  MoveItControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
    : Node("moveit_control_lgw_node", options), is_busy_(false)
  {
    // Declare parameters
    this->declare_parameter("arm_group_name", "arm");
    this->declare_parameter("gripper_action_name", "/gripper_controller/follow_joint_trajectory");
    this->declare_parameter("end_effector_link", "arm/link6");

    // Get parameters
    arm_group_name_ = this->get_parameter("arm_group_name").as_string();
    gripper_action_name_ = this->get_parameter("gripper_action_name").as_string();
    end_effector_link_ = this->get_parameter("end_effector_link").as_string();

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt Control Node...");
    RCLCPP_INFO(this->get_logger(), "Arm group: %s", arm_group_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gripper action: %s", gripper_action_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", end_effector_link_.c_str());
  }

  void initialize()
  {
    // Initialize MoveGroupInterface after node is fully constructed
    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
     
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), arm_group_name_);
    move_group_interface_->setEndEffectorLink(end_effector_link_);
    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setGoalPositionTolerance(0.02);     // 2cm
    move_group_interface_->setGoalOrientationTolerance(0.3);  // ~11deg
    // move_group_interface_->setGoalJointTolerance(0.05);        // rad

    RCLCPP_INFO(this->get_logger(), "goal tolerance: %.3f m, %.3f rad",
                move_group_interface_->getGoalPositionTolerance(),
                move_group_interface_->getGoalOrientationTolerance());
    
    RCLCPP_INFO(this->get_logger(), "MoveIt MoveGroupInterface initialized");

    // Initialize gripper action client
    gripper_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, gripper_action_name_);

    RCLCPP_INFO(this->get_logger(), "Waiting for gripper action server...");
    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN(this->get_logger(), "Gripper action server not available");
    } else {
      RCLCPP_INFO(this->get_logger(), "Gripper action server connected");
    }

    // Create subscriber with QoS to keep only the latest message
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();  // Use best effort delivery
    qos.durability_volatile();  // Don't keep messages for late joiners
    
    subscription_ = this->create_subscription<graspnet_msgs::msg::GraspPose>(
        "/graspnet/grasps", qos,
        std::bind(&MoveItControlNode::targetPoseCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/arm/joint_states_single", qos,
      std::bind(&MoveItControlNode::jointStateCallback, this, std::placeholders::_1));

    stop_signal_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/arm/stop_signal", qos,
      std::bind(&MoveItControlNode::stopSignalCallback, this, std::placeholders::_1));

    // /moveit_control/reset — std_srvs/srv/Trigger.
    //
    // The grasp state machine in this node has three sticky flags
    // (is_busy_, need_to_adjust_gripper_, need_to_return_init_pose_)
    // that are intentionally NOT reset on completion of a successful
    // grasp — upstream's design assumed one grasp per process
    // lifetime, with the caller restarting the node between picks.
    //
    // Robonix doesn't restart packages between MCP calls, so we
    // expose an explicit reset RPC. piper_moveit_rbnx's MCP
    // `reset` capability calls this service; pick_skill_rbnx in turn
    // calls reset at the start of every pick() so the state machine
    // is guaranteed clean even if a previous pick crashed mid-grasp.
    //
    // Reset semantics:
    //   1. Drop all sticky flags (is_busy_, need_to_adjust_gripper_,
    //      need_to_return_init_pose_).
    //   2. Open the gripper to a wide neutral width.
    //   3. Move the arm back to the joint-space init pose
    //      (moveArmtoInit), so the next grasp starts from a known
    //      configuration. This blocks while planning + execution
    //      finish, so the Trigger response only succeeds once the
    //      arm is actually parked.
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/moveit_control/reset",
      std::bind(&MoveItControlNode::resetCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Reset service ready: /moveit_control/reset");

    RCLCPP_INFO(this->get_logger(), "Subscribed to /graspnet/grasps topic");
    RCLCPP_INFO(this->get_logger(), "MoveIt Control Node initialized and ready");
    this->controlGripper(0.12);
    this->moveArmtoInit();
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt Control Node completed");
  }

private:
  void stopSignalCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "Stop signal received");
      is_busy_ = true;
      moveArmtoInit();
    }
  }
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = msg;
    last_joint_state_stamp_ = msg->header.stamp;
    if (need_to_adjust_gripper_) {
      sleep(1);
      // adjust_grapper();
      need_to_adjust_gripper_ = false;
      need_to_return_init_pose_ = true;
    }
    // for (size_t i = 0; i < msg->name.size(); ++i) {
    //   RCLCPP_DEBUG(this->get_logger(), "Joint State listened- %s: %f", msg->name[i].c_str(), msg->position[i]);
    // }
  }

  void targetPoseCallback(const graspnet_msgs::msg::GraspPose::SharedPtr msg)
  {
    if (is_busy_) {
      RCLCPP_WARN(this->get_logger(), "Robot is busy, ignoring new command");
      if (need_to_return_init_pose_) {
        controlGripper(0.025);
        sleep(1);
        RCLCPP_INFO(this->get_logger(), "Returning to initial pose");
        moveArmtoInit();
        need_to_return_init_pose_ = false;
      }
      return;
    }

    is_busy_ = true;
    
    // Start timing: message received
    auto time_received = std::chrono::high_resolution_clock::now();
    
    RCLCPP_INFO(this->get_logger(), "Received target pose command");
    RCLCPP_INFO(this->get_logger(), "Position: [%.3f, %.3f, %.3f]",
                msg->target_pose.pose.position.x,
                msg->target_pose.pose.position.y,
                msg->target_pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Orientation: [%.3f, %.3f, %.3f, %.3f]",
            msg->target_pose.pose.orientation.x,
            msg->target_pose.pose.orientation.y,
            msg->target_pose.pose.orientation.z,
            msg->target_pose.pose.orientation.w);
    RCLCPP_INFO(this->get_logger(), "Gripper width: %.3f m", msg->gripper_width);

    // Control gripper
    controlGripper(msg->gripper_width);
    // Move arm to target pose
    bool success = moveArmToPose(msg->target_pose, time_received);

    if (success) {
      need_to_adjust_gripper_ = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan or execute arm motion");
      is_busy_ = false;
    }
  }

  // /moveit_control/reset handler.
  //
  // Drops all sticky flags + parks the arm at the init pose. Returns
  // success=true once the arm has actually finished moving. If the
  // current state has a grasp mid-flight (busy + active execution),
  // we still set busy=false at the END so the next caller can fire
  // immediately, but we ALSO let the in-flight motion complete first
  // by waiting on the moveArmtoInit() blocking call.
  //
  // NOTE: we don't preempt an in-flight MoveGroup execution here —
  // MoveGroupInterface's cancel API is racy and the upstream design
  // never tested it. If a caller really wants to abort mid-grasp,
  // publish to /arm/stop_signal first (which already triggers
  // moveArmtoInit() through stopSignalCallback) and THEN call reset.
  void resetCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    RCLCPP_INFO(this->get_logger(),
                "Reset requested: clearing state machine + parking arm");
    // Drop all flags up front. Even if moveArmtoInit() fails below,
    // the next grasp command should be acceptable — it's safer than
    // staying stuck in is_busy_=true forever.
    is_busy_ = false;
    need_to_adjust_gripper_ = false;
    need_to_return_init_pose_ = false;

    // Open gripper to a neutral wide width before parking, in case
    // the arm is currently holding something (gripper close on init
    // would otherwise crush whatever's in the jaws).
    try {
      controlGripper(0.025f);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "controlGripper(0.025) threw: %s", e.what());
    }

    bool init_ok = false;
    try {
      init_ok = moveArmtoInit();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "moveArmtoInit() threw: %s", e.what());
    }

    if (init_ok) {
      resp->success = true;
      resp->message = "reset complete; arm parked at init";
    } else {
      // Even on failure we keep is_busy_=false so the caller is at
      // least not LOCKED OUT forever — but tell them the arm isn't
      // parked.
      resp->success = false;
      resp->message = "state flags cleared but moveArmtoInit() failed";
    }
    RCLCPP_INFO(this->get_logger(), "Reset done: %s",
                resp->message.c_str());
  }

  bool adjust_grapper()
  {
    RCLCPP_INFO(this->get_logger(), "Adjusting gripper...");
    try {
      if (!last_joint_state_) {
        RCLCPP_WARN(this->get_logger(), "No joint_states_single received yet");
        return false;
      }

      const auto &names = last_joint_state_->name;
      const auto &pos   = last_joint_state_->position;

      auto index_of = [&](const std::string &name) -> int {
        auto it = std::find(names.begin(), names.end(), name);
        if (it == names.end()) {
          RCLCPP_ERROR(this->get_logger(), "Joint '%s' not found in joint_states_single", name.c_str());
          return -1;
        }
        return static_cast<int>(std::distance(names.begin(), it));
      };

      int idx1 = index_of("joint1");
      int idx2 = index_of("joint2");
      int idx3 = index_of("joint3");
      int idx4 = index_of("joint4");
      int idx5 = index_of("joint5");
      int idx6 = index_of("joint6");
      // int idx7 = index_of("gripper");

      if (idx1 < 0 || idx2 < 0 || idx3 < 0 || idx4 < 0 || idx5 < 0 || idx6 < 0) {
        return false;  // 有 joint 没找到，直接退出
      }

      for (auto n : names) {
        RCLCPP_INFO(this->get_logger(), "Joint in state: %s %lf", n.c_str(), pos[index_of(n)]);
      }

      std::vector<double> joints(6);
      joints[0] = pos[idx1];
      joints[1] = pos[idx2];
      joints[2] = pos[idx3];
      joints[3] = pos[idx4];
      joints[4] = pos[idx5];
      joints[5] = pos[idx6];
      // joints[6] = pos[idx7]; // gripper

      // 1. 给关节加一个偏移
      joints[1] += 0.1208396746;  // 你原来的偏移

      // 2. 先 wrap 到 [-pi, pi]，避免跳太远
      auto wrapToPi = [](double a) {
        const double TWO_PI = 2.0 * M_PI;
        while (a > M_PI)  a -= TWO_PI;
        while (a < -M_PI) a += TWO_PI;
        return a;
      };
      joints[5] = wrapToPi(joints[5]);

      // 3. 再按 piper 的物理限制做限幅（这里用你 URDF 里的 [-2.0944, 2.0944]）
      const double lower = -2.0944;
      const double upper =  2.0944;
      joints[5] = std::clamp(joints[5], lower, upper);

      // 4. 做一个简单的数值“取整”（防止太多小数误差）
      auto roundTo = [](double v) {
        return std::round(v * 1e6) / 1e6;  // 保留 6 位小数
      };
      joints[5] = roundTo(joints[5]);

      // 5. 把目标关节角送给 MoveIts
      return moveArmJoint(joints);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error during arm motion: %s", e.what());
      move_group_interface_->clearPoseTargets();
      return false;
    }
  }

  bool moveArmJoint(const std::vector<double>& joints)
  {
    try{
      // Pin the planner's start state to the arm's CURRENT joint
      // position. Without this, MoveGroupInterface uses whatever
      // start state is in the planning scene's robot model — which
      // can be stale (e.g. the default all-zero state at startup,
      // or the previous goal if /joint_states hasn't refreshed yet).
      // The visible symptom of a stale start state: a "joint" plan
      // that mysteriously detours through joint=0 before reaching
      // the actual goal, because that's the assumed starting point.
      move_group_interface_->setStartStateToCurrentState();
      move_group_interface_->setJointValueTarget(joints);

      // Plan
      auto time_start_planning = std::chrono::high_resolution_clock::now();
      RCLCPP_INFO(this->get_logger(), "moveArmJoint Planning trajectory...");
      
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto plan_res = move_group_interface_->plan(plan);
      bool success = (plan_res == moveit::core::MoveItErrorCode::SUCCESS);

      auto time_planning_done = std::chrono::high_resolution_clock::now();
      auto planning_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          time_planning_done - time_start_planning);
      
      RCLCPP_INFO(this->get_logger(), "Planning completed in %.3f ms", 
                  planning_duration.count() / 1000.0);

      if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Planning failed");
        const auto msg = moveit::core::error_code_to_string(plan_res);
        RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", msg.c_str());
        move_group_interface_->clearPoseTargets();
        return false;
      }

      // Execute
      RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
      auto time_start_execution = std::chrono::high_resolution_clock::now();
      
      success = (move_group_interface_->execute(plan) == 
                 moveit::core::MoveItErrorCode::SUCCESS);

      auto time_execution_started = std::chrono::high_resolution_clock::now();
      
      auto execution_delay = std::chrono::duration_cast<std::chrono::milliseconds>(
          time_execution_started - time_start_execution);

      RCLCPP_INFO(this->get_logger(), "=== Timing Statistics ===");
      RCLCPP_INFO(this->get_logger(), "  - Planning time: %.3f ms", 
                  planning_duration.count() / 1000.0);
      RCLCPP_INFO(this->get_logger(), "  - Execution setup time: %.3f ms", 
                  execution_delay.count() / 1000.0);
      RCLCPP_INFO(this->get_logger(), "========================");

      move_group_interface_->clearPoseTargets();

      if (success) {
        RCLCPP_INFO(this->get_logger(), "Arm motion completed successfully");
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Execution failed");
        return false;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error during arm motion: %s", e.what());
      move_group_interface_->clearPoseTargets();
      return false;
    }
  }

  bool moveArmToPose(const geometry_msgs::msg::PoseStamped& target_pose,
                     const std::chrono::high_resolution_clock::time_point& time_received)
  {
    // Transform target_pose to base_link frame and print

    geometry_msgs::msg::PoseStamped pose_in_base_link, pose_in_link6;
    try {
      // Transform the pose to base_link frame
      // pose_in_link6 = tf_buffer_->transform(target_pose, "arm/link6", tf2::durationFromSec(1.0));
      // RCLCPP_INFO(this->get_logger(), "Target pose in link6 frame:");
      // RCLCPP_INFO(this->get_logger(), "  Position: [%.4f, %.4f, %.4f]",
      //             pose_in_link6.pose.position.x,
      //             pose_in_link6.pose.position.y,
      //             pose_in_link6.pose.position.z);
      // RCLCPP_INFO(this->get_logger(), "  Orientation (quaternion): [%.4f, %.4f, %.4f, %.4f]",
      //             pose_in_link6.pose.orientation.x,
      //             pose_in_link6.pose.orientation.y,
      //             pose_in_link6.pose.orientation.z,
      //             pose_in_link6.pose.orientation.w);
      // pose_in_link6.pose.position.z -= 0.10; // raise 10cm above the grasp point
      // // 记得把 frame_id 改成 link6，时间戳也可以更新
      // pose_in_link6.header.frame_id = "arm/link6";
      // pose_in_link6.header.stamp = this->now();
      // pose_in_base_link = tf_buffer_->transform(pose_in_link6, "arm/base_link", tf2::durationFromSec(1.0));
      pose_in_base_link = tf_buffer_->transform(target_pose, "arm/base_link", tf2::durationFromSec(1.0));

      // Convert quaternion to euler angles (RPY)
      tf2::Quaternion quat(
        pose_in_base_link.pose.orientation.x,
        pose_in_base_link.pose.orientation.y,
        pose_in_base_link.pose.orientation.z,
        pose_in_base_link.pose.orientation.w
      );
      double roll, pitch, yaw;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      
      // Convert radians to degrees
      double roll_deg = roll * 180.0 / M_PI;
      double pitch_deg = pitch * 180.0 / M_PI;
      double yaw_deg = yaw * 180.0 / M_PI;
      
      RCLCPP_INFO(this->get_logger(), "Target pose in base_link frame:");
      RCLCPP_INFO(this->get_logger(), "  Position: [%.4f, %.4f, %.4f]",
                  pose_in_base_link.pose.position.x,
                  pose_in_base_link.pose.position.y,
                  pose_in_base_link.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "  Orientation (RPY degrees): [%.4f, %.4f, %.4f]",
                  roll_deg, pitch_deg, yaw_deg);
      RCLCPP_INFO(this->get_logger(), "  Orientation (quaternion): [%.4f, %.4f, %.4f, %.4f]",
                  pose_in_base_link.pose.orientation.x,
                  pose_in_base_link.pose.orientation.y,
                  pose_in_base_link.pose.orientation.z,
                  pose_in_base_link.pose.orientation.w);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform pose to base_link: %s", ex.what());
      return false;
    }
    // return true;//debug only
    try {
      // Pin the planner's start state to the arm's current joint
      // position; same rationale as moveArmJoint(). Without this
      // the cartesian planner can pick a wildly suboptimal IK
      // solution that "starts" from a stale assumed pose.
      move_group_interface_->setStartStateToCurrentState();
      // Set target pose
      move_group_interface_->setPoseTarget(pose_in_base_link);

      // Plan
      auto time_start_planning = std::chrono::high_resolution_clock::now();
      RCLCPP_INFO(this->get_logger(), "moveArmToPose Planning trajectory...");
      
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto plan_res = move_group_interface_->plan(plan);
      bool success = (plan_res == moveit::core::MoveItErrorCode::SUCCESS);

      auto time_planning_done = std::chrono::high_resolution_clock::now();
      auto planning_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          time_planning_done - time_start_planning);
      
      RCLCPP_INFO(this->get_logger(), "Planning completed in %.3f ms", 
                  planning_duration.count() / 1000.0);

      if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Planning failed");
        const auto msg = moveit::core::error_code_to_string(plan_res);
        RCLCPP_ERROR(this->get_logger(), "Planning failed: %s", msg.c_str());
        move_group_interface_->clearPoseTargets();
        return false;
      }

      // return true;//debug , only plan not execute
      // Execute
      RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
      auto time_start_execution = std::chrono::high_resolution_clock::now();
      
      success = (move_group_interface_->execute(plan) == 
                 moveit::core::MoveItErrorCode::SUCCESS);

      auto time_execution_started = std::chrono::high_resolution_clock::now();
      
      // Calculate time from message received to motion start
      auto total_delay = std::chrono::duration_cast<std::chrono::milliseconds>(
          time_execution_started - time_received);
      
      auto execution_delay = std::chrono::duration_cast<std::chrono::milliseconds>(
          time_execution_started - time_start_execution);

      RCLCPP_INFO(this->get_logger(), "=== Timing Statistics ===");
      RCLCPP_INFO(this->get_logger(), "Total delay (received to motion start): %.3f ms", 
                  total_delay.count() / 1000.0);
      RCLCPP_INFO(this->get_logger(), "  - Planning time: %.3f ms", 
                  planning_duration.count() / 1000.0);
      RCLCPP_INFO(this->get_logger(), "  - Execution setup time: %.3f ms", 
                  execution_delay.count() / 1000.0);
      RCLCPP_INFO(this->get_logger(), "========================");

      move_group_interface_->clearPoseTargets();

      if (success) {
        RCLCPP_INFO(this->get_logger(), "Arm motion completed successfully");
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Execution failed");
        return false;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error during arm motion: %s", e.what());
      move_group_interface_->clearPoseTargets();
      return false;
    }
  }

  bool moveArmtoInit()
  {
    double degrees[] = {90.0, 104.0, -82.0, 0.0, 70.0, 0.0};
    // double degrees[] = {0.0, 90.0, -90.0, 0.0, 70.0, 0.0};
    const int size = sizeof(degrees) / sizeof(degrees[0]);
    
    // 创建vector存储弧度值
    std::vector<double> radians;
    
    // 预留空间以提高性能
    radians.reserve(size);
    
    // 角度转弧度公式：弧度 = 角度 × π / 180
    const double pi = M_PI;  // 使用cmath中定义的π
    
    for (int i = 0; i < size; i++) {
        // 将角度转换为弧度
        double radian = degrees[i] * pi / 180.0;
        
        // 添加到vector
        radians.push_back(radian);
    }

    // auto time_now = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "Moving arm to initial pose...");
    return moveArmJoint(radians);
  }

  void controlGripper(float width)
  {
    // Clamp width to valid range
    width = std::max(0.0f, std::min(0.12f, width));

    // Map input width (0-0.07) to joint value (0-0.035)
    float joint_value = width / 2.0f;

    RCLCPP_INFO(this->get_logger(), "Setting gripper width to %.3f m (joint7 = %.3f)",
                width, joint_value);

    // Create goal message
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"joint7"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {joint_value};
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);

    goal_msg.trajectory.points.push_back(point);

    // Send goal
    RCLCPP_INFO(this->get_logger(), "Sending gripper command...");

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        [this](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Gripper goal rejected");
          } else {
            RCLCPP_INFO(this->get_logger(), "Gripper goal accepted");
          }
        };

    send_goal_options.result_callback =
        [this](const GoalHandleFollowJointTrajectory::WrappedResult& result) {
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(this->get_logger(), "Gripper motion completed");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(this->get_logger(), "Gripper motion aborted");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_WARN(this->get_logger(), "Gripper motion canceled");
              break;
            default:
              RCLCPP_ERROR(this->get_logger(), "Unknown gripper motion result");
              break;
          }
        };

    gripper_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // Parameters
  std::string arm_group_name_;
  std::string gripper_action_name_;
  std::string end_effector_link_;

  // MoveIt interface
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

  // Action client
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_action_client_;

  // Subscriber
  rclcpp::Subscription<graspnet_msgs::msg::GraspPose>::SharedPtr subscription_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_signal_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  bool is_busy_;

  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

  rclcpp::Time last_joint_state_stamp_;

  bool need_to_adjust_gripper_;

  bool need_to_return_init_pose_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<MoveItControlNode>();
  
  // Initialize MoveIt components after node is created
  node->initialize();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
