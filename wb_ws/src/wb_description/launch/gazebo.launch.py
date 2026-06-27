import os
from pathlib import Path
from os import pathsep
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction, GroupAction)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, \
    PathJoinSubstitution, PythonExpression


def make_robot(name, x, y, yaw, pkg_description, is_ignition):
    """Returns all nodes/actions needed for one robot instance."""
    roboo_description = get_package_share_directory("moveit_config")


    urdf_path = os.path.join(pkg_description, "urdf", "wb_arm.urdf.xacro")
    controllers_file = os.path.join(roboo_description, "config",
                                    f"ros2_controllers_{name}.yaml")

    robot_description = ParameterValue(
        Command([
            "xacro ", urdf_path,
            " is_ignition:=", is_ignition,
            " prefix:=", name, "_",
            " robot_name:=", name,           # ADD THIS
            " controllers_file:=", controllers_file,
        ]),
        value_type=str
    )

    # robot_state_publisher under the robot's namespace
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=name,
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
            "frame_prefix": name + "_",
        }]
    )

    # Spawn into Gazebo
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", f"/{name}/robot_description",
            "-name", name,
            "-x", str(x),
            "-y", str(y),
            "-z", "0.109",
            "-Y", str(yaw),
        ],
    )

    # # Controller spawners — scoped to the robot's namespace
    jsb = TimerAction(period=3.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", f"/{name}/controller_manager"],
    )])

    arm = TimerAction(period=10.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                   "--controller-manager", f"/{name}/controller_manager"],
    )])

    gripper = TimerAction(period=10.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller",
                   "--controller-manager", f"/{name}/controller_manager"],
    )])

    return [rsp, spawn, jsb, arm, gripper]


def generate_launch_description():
    robo_description = get_package_share_directory("wb_description")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                robo_description, "urdf", "wb_arm.urdf.xacro"
            ),
        description="Path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
            robo_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(robo_description).parent.resolve())
    model_path += pathsep + os.path.join(robo_description, "models")

    gazebo_resource_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH", model_path
    )
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
            launch_arguments={
                "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
            }.items()
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    robot1_nodes = make_robot("robot1", x=0.0,  y=-1.2, yaw=0.0,
                              pkg_description=robo_description,
                              is_ignition=is_ignition)

    robot2_nodes = make_robot("robot2", x=0.0,  y=1.2,  yaw=3.14159,
                              pkg_description=robo_description,
                              is_ignition=is_ignition)

    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,   # MUST be before gazebo
        gazebo,
        clock_bridge,
        *robot1_nodes,
        *robot2_nodes,
])