from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_description",
                 package_name="robo_moveit_config").planning_pipelines(pipelines=["ompl"]).to_moveit_configs()

    # MTC Demo node
    pick_place_demo = Node(
        package="pickplace",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([pick_place_demo])