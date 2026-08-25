cmake_minimum_required(VERSION 3.5)
project(laser_line_extraction)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineSegment.msg"
  "msg/LineSegmentList.msg"
  DEPENDENCIES sensor_msgs geometry_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Build libraries
add_library(line src/line.cpp)
ament_target_dependencies(line rclcpp)

add_library(line_extraction src/line_extraction.cpp)
ament_target_dependencies(line_extraction rclcpp)

add_library(line_extraction_ros src/line_extraction_ros.cpp)
ament_target_dependencies(line_extraction_ros 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  visualization_msgs
)
target_link_libraries(line_extraction_ros line line_extraction)
rosidl_target_interfaces(line_extraction_ros ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Build executable
add_executable(line_extraction_node src/line_extraction_node.cpp)
target_link_libraries(line_extraction_node line_extraction_ros)
ament_target_dependencies(line_extraction_node rclcpp)

# Install
install(TARGETS line_extraction_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS line_extraction_ros line_extraction line
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
ament_package()