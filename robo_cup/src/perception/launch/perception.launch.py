# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():

#     perception_node = Node(
#         package="perception",
#         executable="obj_perception"
#     )

#     return LaunchDescription(
#         [
#             perception_node
#         ]
#     )








import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    # ── obj_perception node ────────────────────────────────────────────────────
    # Starts immediately — begins publishing /detected_blocks right away.
    perception_node = Node(
        package="perception",
        executable="obj_perception",
        output="screen",
    )

    # ── mtc_node ───────────────────────────────────────────────────────────────
    # Delayed by 3 s so that obj_perception has time to start up and publish
    # at least a few detections before mtc_node tries to read them.
    # (mtc_node also has its own internal wait loop, but the delay helps.)
    mtc_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="mtc_tutorial",   # ← change to your actual package name
                executable="mtc_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([
        perception_node,
        mtc_node,
    ])