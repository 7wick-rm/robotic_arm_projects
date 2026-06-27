cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

set(Python_FIND_STRATEGY LOCATION)
set(Python_FIND_IMPLEMENTATIONS CPython)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
set(Python_EXECUTABLE ${Python3_EXECUTABLE})
set(Python_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
set(Python_LIBRARIES ${Python3_LIBRARIES})
set(Python_NumPy_INCLUDE_DIRS ${Python3_NumPy_INCLUDE_DIRS})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BlockPose.msg"
  "action/PickAndPlace.action"
  "srv/GetPlan.srv"
  DEPENDENCIES geometry_msgs action_msgs
)

ament_package()