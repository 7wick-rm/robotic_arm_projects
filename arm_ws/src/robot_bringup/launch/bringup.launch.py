import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction # <--- NEW: Import the timer
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("assembly_final_done_pakka", package_name="my_robot_moveit_config")
        .to_moveit_configs()
    )

    bt_pkg_dir = get_package_share_directory('robot_behavior_tree')
    tree_xml_path = os.path.join(bt_pkg_dir, 'config', 'pick_and_place.xml')

    return LaunchDescription([

        Node(
            package='robot_planner', 
            executable='planner_server',
            name='planner_server',
            output='screen'
        ),

        Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"moveit_controller_manager": 
                "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"use_sim_time": True},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            ],
        ),

        Node(
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
        ),

        Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ['-0.08','-0.25','0','0','1.57','0','depth_camera_link', 'aligned_depth_camera_link'], 
        ),
        
        Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['0.0','0.0','0.0','0.0','0.0','0.0','world','base_link']
        ),

        Node(
            package='robot_perception',
            executable='obj_perception',
            name='obj_perception',
            output='screen'
        ),

        Node(
        package="robot_mtc_control",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
        ),

        # --- THE FIX: DELAY THE BRAIN BY 5 SECONDS ---
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_behavior_tree',
                    executable='pick_and_place_tree',
                    name='pick_and_place_tree',
                    output='screen',
                    arguments=[tree_xml_path] 
                )
            ]
        )

    ])
