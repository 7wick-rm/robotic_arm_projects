from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mecanum_arm_robot", package_name="robot_moveit_config")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # Override fake controller manager with real one
            {"moveit_controller_manager": 
                "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"use_sim_time": True},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])