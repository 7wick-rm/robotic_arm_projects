"""
mujoco_piper.launch.py

Brings up:
  - MuJoCo physics as the ros2_control hardware backend
  - move_group (MoveIt planning)
  - RViz with the MotionPlanning plugin (drag-able interactive markers)
  - controller spawners, sequenced to start only once MuJoCo's
    controller_manager is actually up

Prereqs before running:
  1. Confirm the real executable name for your installed mujoco_ros2_control:
       ros2 pkg executables mujoco_ros2_control
     Update MUJOCO_CONTROL_EXECUTABLE below if it isn't "ros2_control_node".

  2. Confirm the include filename in piper.urdf.xacro matches the actual
     file on disk exactly (piper_description.ros2_control.xacro vs whatever
     you actually named it):
       find <BASE> -iname "*ros2_control*.xacro"

  3. mujoco_model_path below must point at your CONVERTED MJCF
     (piper.xml / scene.xml from urdf-to-mjcf) -- NOT the raw piper.urdf.
"""

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

MUJOCO_CONTROL_EXECUTABLE = "mujoco_ros2_control"  # confirmed via: ros2 pkg executables mujoco_ros2_control


def generate_launch_description():

    # ── Paths ──────────────────────────────────────────────────────────────
    BASE             = "/home/sathwik/robotic_arm_projects/mujoco_ws/piper/piper_resources"
    piper_desc_dir   = os.path.join(BASE, "piper_description")
    piper_moveit_dir = os.path.join(BASE, "piper_moveit_config")

    urdf_xacro        = os.path.join(piper_moveit_dir,   "config",   "piper_description.urdf.xacro")
    srdf_file         = os.path.join(piper_moveit_dir, "config", "piper_description.srdf")
    kinematics_yaml   = os.path.join(piper_moveit_dir, "config", "kinematics.yaml")
    controllers_yaml  = os.path.join(piper_moveit_dir, "config", "ros2_controllers.yaml")
    moveit_ctrl_yaml  = os.path.join(piper_moveit_dir, "config", "moveit_controllers.yaml")
    joint_limits_yaml = os.path.join(piper_moveit_dir, "config", "joint_limits.yaml")
    rviz_config       = os.path.join(piper_moveit_dir, "config", "moveit.rviz")

    # Converted MJCF, NOT the raw piper.urdf -- see module docstring point 3.
    mujoco_scene = os.path.join(BASE, "piper_mujoco", "xml", "scene.xml")

    # ── Xacro mappings ────────────────────────────────────────────────────
    # Keys here MUST exactly match <xacro:arg name="..."> in piper.urdf.xacro.
    # Your current piper.urdf.xacro declares: initial_positions_file, mujoco,
    # mujoco_model_path. Add more here only if you add more xacro:args.
    xacro_mappings = {
        "use_mujoco":            "true",
        "mujoco_model_path": mujoco_scene,
    }

    # ── Robot description ─────────────────────────────────────────────────
    robot_description_content = xacro.process_file(
        urdf_xacro,
        mappings=xacro_mappings,
    ).toxml()
    robot_description = {"robot_description": robot_description_content}

    # ── MoveIt config ──────────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_moveit_config")
        .robot_description(file_path=urdf_xacro, mappings=xacro_mappings)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_yaml)
        .joint_limits(file_path=joint_limits_yaml)
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

    # MuJoCo IS the controller_manager here -- it hosts ros2_control and
    # steps physics each control cycle.
    mujoco_control_node = Node(
        package="mujoco_ros2_control",
        executable=MUJOCO_CONTROL_EXECUTABLE,
        output="screen",
        parameters=[
            robot_description,
            controllers_yaml,
            {
                "use_sim_time": True,
                "mujoco_model_path": mujoco_scene,
            },
        ],
    )

    # move_group_node: powers planning AND the interactive-marker service
    # that RViz's MotionPlanning display talks to.
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

    # rviz_node: loads moveit.rviz, which must already contain the
    # "MotionPlanning" display (this is what gives you the draggable
    # interactive marker at the end effector). If it's missing, add the
    # MotionPlanning display in RViz manually once and re-save moveit.rviz.
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

    return LaunchDescription(
        [
            mujoco_control_node,
            robot_state_publisher_node,
            world_to_base_tf_node,
            move_group_node,
            rviz_node,
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=mujoco_control_node,
                    on_start=[jsb_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=jsb_spawner,
                    on_exit=[jtc_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=jtc_spawner,
                    on_exit=[grip_spawner],
                )
            ),
        ]
    )