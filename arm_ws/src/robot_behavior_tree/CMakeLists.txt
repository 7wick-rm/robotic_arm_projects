cmake_minimum_required(VERSION 3.8)
project(robot_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 2. Add Executable
add_executable(pick_and_place_tree src/pick_and_place_tree.cpp)

target_include_directories(pick_and_place_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(pick_and_place_tree PUBLIC cxx_std_17)

# 3. Link Dependencies
ament_target_dependencies(pick_and_place_tree
  rclcpp
  rclcpp_action
  std_msgs
  behaviortree_cpp
  behaviortree_ros2
  robot_interfaces
  ament_index_cpp
)

# 4. Install Directives
# Install the executable
install(TARGETS pick_and_place_tree
  DESTINATION lib/${PROJECT_NAME})

# Install the config directory containing the XML tree
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()