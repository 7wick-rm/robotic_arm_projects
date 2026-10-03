import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # MoveIt config for your robot
    moveit_config = (
        MoveItConfigsBuilder(
            "robot_description",
            package_name="robo_moveit_config"
        )
        .robot_description(file_path="config/robot_description.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
        
    )

    # ExecuteTaskSolutionCapability so MTC solutions can be executed
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # move_group node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        {"use_sim_time": True},
    ],
    )

    # RViz — uses your moveit config's rviz file
    rviz_config_file = os.path.join(
        get_package_share_directory("robo_moveit_config"),
        "launch",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        # arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    # Static TF — world -> base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        parameters=[{"use_sim_time":True}]
    )

    # # Robot State Publisher
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[moveit_config.robot_description],
    # )

    # #ros2_controlnode
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("robot_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[ros2_controllers_path],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="both",
    # )

    # # Spawn controllers
    # load_controllers = []
    # for controller in [
    #     "arm_controller",
    #     "gripper_controller",
    #     "joint_state_broadcaster",
    # ]:
    #     load_controllers += [
    #         ExecuteProcess(
    #             cmd=["ros2 run controller_manager spawner {}".format(controller)],
    #             shell=True,
    #             output="screen",
    #         )
    #     ]

    return LaunchDescription(
        [
            # static_tf,
        #robot_state_publisher,
            run_move_group_node,
            rviz_node,
            #ros2_control_node,
        ]
         #+ load_controllers
    )