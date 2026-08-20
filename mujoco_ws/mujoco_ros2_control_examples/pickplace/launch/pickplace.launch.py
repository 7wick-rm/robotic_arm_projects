from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
    MoveItConfigsBuilder("moveit_resources_panda")
    .robot_description(
        file_path="config/panda.urdf.xacro",
        mappings={"ros2_control_hardware_type": "mujoco"},
    )
    .robot_description_semantic(file_path="config/panda.srdf")
    .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
    .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
    .to_dict()
)
    # MTC Demo node
    pick_place_demo = Node(
        package="pickplace",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_place_demo])