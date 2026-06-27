# <?xml version="1.0" encoding="UTF-8"?>
# <launch>
#     <let name="robot_description"
#          value="$(command 'xacro $(find-pkg-share piper_description)/urdf/piper.urdf.xacro')" />

#     <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" output="both"/>

#     <node pkg="robot_state_publisher" exec="robot_state_publisher" output="both">
#         <param name="robot_description" value="$(var robot_description)" />
#     </node>

#     <node pkg="rviz2" exec="rviz2" args="-d $(find-pkg-share piper_description)/rviz/robot_urdf.rviz" output="both"/>
# </launch>




import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_description_dir = get_package_share_directory("piper_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        arm_description_dir, "urdf", "piper.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", "/home/sathwik/piper_ros2/src/piper_description/rviz/robot_urdf.rviz"],
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
