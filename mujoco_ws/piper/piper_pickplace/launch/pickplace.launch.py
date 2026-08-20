from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():

    BASE             = "/home/sathwik/robotic_arm_projects/mujoco_ws/piper/piper_resources"
    piper_desc_dir   = os.path.join(BASE, "piper_description")
    piper_moveit_dir = os.path.join(BASE, "piper_moveit_config")

    urdf_xacro      = os.path.join(piper_desc_dir,   "urdf",   "piper.urdf")
    srdf_file       = os.path.join(piper_moveit_dir, "config", "piper_description.srdf")
    kinematics_yaml = os.path.join(piper_moveit_dir, "config", "kinematics.yaml")
    moveit_ctrl_yaml= os.path.join(piper_moveit_dir, "config", "moveit_controllers.yaml")
    joint_limits_yaml = os.path.join(piper_moveit_dir, "config", "joint_limits.yaml")

    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_moveit_config")
        .robot_description(
            file_path=urdf_xacro,
            mappings={
                "mujoco":          "true",
                "mock_hardware":   "false",
                "include_gripper": "true",
            },
        )
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_yaml)
        .joint_limits(file_path=joint_limits_yaml)
        .trajectory_execution(file_path=moveit_ctrl_yaml)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    pick_place_demo = Node(
        package="piper_pickplace",
        executable="mtc",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_place_demo])