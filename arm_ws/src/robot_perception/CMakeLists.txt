cmake_minimum_required(VERSION 3.8)
project(robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(robot_interfaces REQUIRED)

# Find System PCL
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search kdtree features)

# Include PCL headers
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add Executable
add_executable(obj_perception src/obj_perception.cpp)

# Link PCL libraries
target_link_libraries(obj_perception ${PCL_LIBRARIES})

# Link ROS 2 dependencies
ament_target_dependencies(obj_perception
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  robot_interfaces
)

install(TARGETS obj_perception
  DESTINATION lib/${PROJECT_NAME})

ament_package()
