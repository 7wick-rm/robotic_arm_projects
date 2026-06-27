cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(perception REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Generate action interfaces FIRST ─────────────────────────────────────────
# Must come before any target that #includes the generated headers.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickAndPlace.action"
  DEPENDENCIES geometry_msgs
)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(mtc_node src/mtc_node2.cpp)

target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(mtc_node
  rclcpp
  rclcpp_action
  moveit_task_constructor_core
  perception
)

# ── Link generated action typesupport to the executable ──────────────────────
# This is what makes the generated headers actually findable at compile time.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mtc_node "${cpp_typesupport_target}")

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/pick_and_place_client.py
  DESTINATION lib/${PROJECT_NAME})

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()