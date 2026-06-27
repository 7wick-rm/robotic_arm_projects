"""
mujoco_piper.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
from launch.actions import ExecuteProcess

def generate_launch_description():

    # ── Paths ──────────────────────────────────────────────────────────────
    BASE             = "/home/sathwik/mujoco_ws/piper/piper_resources"
    piper_desc_dir   = os.path.join(BASE, "piper_description")
    piper_moveit_dir = os.path.join(BASE, "piper_moveit_config")

    urdf_xacro       = os.path.join(piper_desc_dir,   "urdf",   "piper.urdf.xacro")
    srdf_file        = os.path.join(piper_moveit_dir, "config", "piper.srdf.xacro")
    kinematics_yaml  = os.path.join(piper_moveit_dir, "config", "kinematics.yaml")
    controllers_yaml = os.path.join(piper_moveit_dir, "config", "ros2_controllers.yaml")
    moveit_ctrl_yaml = os.path.join(piper_moveit_dir, "config", "moveit_controllers.yaml")
    rviz_config      = os.path.join(piper_moveit_dir, "config", "moveit.rviz")
    mujoco_scene     = os.path.join(BASE, "piper_mujoco", "the_piper", "scene.xml")

    # ── Robot description ─────────────────────────────────────────────────
    robot_description_content = xacro.process_file(
        urdf_xacro,
        mappings={
            "mujoco":          "true",
            "mock_hardware":   "false",
            "include_gripper": "true",
        },
    ).toxml()
    robot_description = {"robot_description": robot_description_content}

    # ── MoveIt config ──────────────────────────────────────────────────────
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
        .joint_limits(file_path=os.path.join(piper_moveit_dir, "config", "joint_limits.yaml"))
        .trajectory_execution(file_path=moveit_ctrl_yaml)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ══════════════════════════════════════════════════════════════════════
    # NODES
    # ══════════════════════════════════════════════════════════════════════

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    world_to_base_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_tf",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        parameters=[{"use_sim_time": True}],
    )

    # ── MuJoCo ros2_control node ──────────────────────────────────────────
    mujoco_control_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            robot_description,
            controllers_yaml,
            {
                "use_sim_time":      True,
                "mujoco_model_path": mujoco_scene,
            },
        ],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"use_sim_time": True},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    # ── Controller spawners ────────────────────────────────────────────────
    # All pointing to /mujoco_ros2_control where this fork's CM lives.

    def make_spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )
    jsb_spawner  = make_spawner("joint_state_broadcaster")
    jtc_spawner  = make_spawner("arm_controller")
    grip_spawner = make_spawner("gripper_controller")
    # admittance_spawner = make_spawner("admittance_controller")

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=mujoco_control_node,
                    on_start=[jsb_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=jsb_spawner,
                    on_exit=[jtc_spawner], #admittance_spawner
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=jtc_spawner,
                    on_exit=[grip_spawner],
                )
            ),
            rviz_node,
            world_to_base_tf_node,
            robot_state_publisher_node,
            move_group_node,
            mujoco_control_node
        ]
    )