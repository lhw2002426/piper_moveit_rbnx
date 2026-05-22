cmake_minimum_required(VERSION 3.8)
project(piper_moveit_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(graspnet_msgs REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# MoveIt control node
add_executable(moveit_control_node src/moveit_control_node.cpp)
target_include_directories(moveit_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_control_node
  "rclcpp"
  "rclcpp_action"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "control_msgs"
  "trajectory_msgs"
  "graspnet_msgs"
)

# YOLO MoveIt control node - 新添加的节点
add_executable(moveit_control_node_yolo src/moveit_control_node_yolo.cpp)
target_include_directories(moveit_control_node_yolo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_control_node_yolo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_control_node_yolo
  "rclcpp"
  "rclcpp_action"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "control_msgs"
  "trajectory_msgs"
  "graspnet_msgs"
  "std_msgs"
  "std_srvs"
)

# Test control node
add_executable(test_control_node src/test_control_node.cpp)
target_include_directories(test_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test_control_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  test_control_node
  "rclcpp"
  "geometry_msgs"
  "graspnet_msgs"
)

# 安装所有三个可执行文件
install(TARGETS 
  moveit_control_node 
  moveit_control_node_yolo  # 新添加的节点
  test_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()