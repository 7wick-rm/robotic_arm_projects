cmake_minimum_required(VERSION 3.8)
project(robot_mtc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(mtc_node src/mtc_node.cpp)

target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)

# ament_target_dependencies automatically handles the linking for robot_interfaces
ament_target_dependencies(mtc_node
  rclcpp
  rclcpp_action
  moveit_task_constructor_core
  tf2_geometry_msgs
  tf2_eigen
  robot_interfaces
)

install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()