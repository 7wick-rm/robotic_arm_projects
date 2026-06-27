cmake_minimum_required(VERSION 3.10.2)
project(piper_mujoco)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY the_piper DESTINATION share/${PROJECT_NAME})
