# import os
# from moveit_configs_utils import MoveItConfigsBuilder
# from launch import LaunchDescription
# from launch_ros.actions import Node


# def make_moveit_nodes(robot_name, is_ignition):
#     prefix = robot_name + "_"

#     moveit_config = (
#         MoveItConfigsBuilder("wb_arm", package_name="wb_moveit_config")
#         .robot_description(mappings={
#             "is_ignition": is_ignition,
#             "prefix": prefix,
#         })
#         .to_moveit_configs()
#     )

#     move_group = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         namespace=robot_name,
#         output="screen",
#         parameters=[
#             moveit_config.to_dict(),
#             {"use_sim_time": True},
#         ],
#     )

#     return [move_group]


# def generate_launch_description():
#     ros_distro = os.environ.get("ROS_DISTRO", "humble")
#     is_ignition = "True" if ros_distro == "humble" else "False"

#     robot1_nodes = make_moveit_nodes("robot1", is_ignition)
#     robot2_nodes = make_moveit_nodes("robot2", is_ignition)

#     return LaunchDescription([
#         *robot1_nodes,
#         *robot2_nodes,
#     ])




import os
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def make_moveit_nodes(robot_name, is_ignition):
    prefix = robot_name + "_"
    pkg_moveit = get_package_share_directory("wb_moveit_config")

    # Load SRDF via xacro so prefix is applied
    srdf_path = os.path.join(pkg_moveit, "config", "wb_arm.srdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("wb_arm", package_name="wb_moveit_config")
        .robot_description(mappings={
            "is_ignition": is_ignition,
            "prefix": prefix,
        })
        .robot_description_semantic(
            file_path=srdf_path,
            mappings={"prefix": prefix}
        )
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution(
            file_path=os.path.join(pkg_moveit, "config", "moveit_controllers.yaml")
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=robot_name,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            # Tell move_group which controller manager to talk to
            {"controller_manager_name": f"/{robot_name}/controller_manager"},
        ],
        remappings=[
            # Remap joint_states to the namespaced topic from Gazebo
            ("/joint_states", f"/{robot_name}/joint_states"),
            ("/robot_description", f"/{robot_name}/robot_description"),
        ],
    )

    return [move_group]


def generate_launch_description():
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot1_nodes = make_moveit_nodes("robot1", is_ignition)
    robot2_nodes = make_moveit_nodes("robot2", is_ignition)

    return LaunchDescription([
        *robot1_nodes,
        *robot2_nodes,
    ])