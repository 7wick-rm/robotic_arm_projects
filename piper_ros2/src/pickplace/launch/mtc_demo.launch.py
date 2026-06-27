import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mtc_node = Node(
        package="pickplace",
        executable="mtc",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("piper_moveit_config"),
                "config",
                "kinematics.yaml"
            ),
            os.path.join(
                get_package_share_directory("piper_moveit_config"),
                "config",
                "ompl_planning.yaml"
            ),
            os.path.join(
                get_package_share_directory("piper_moveit_config"),
                "config",
                "joint_limits.yaml"
            ),
        ],
    )

    return LaunchDescription([mtc_node])