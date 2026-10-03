import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml


def load_yaml(package_name, file_path):
    package_share = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_share, file_path)
    with open(absolute_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    robot_description_pkg = get_package_share_directory('robot_description')
    moveit_config_pkg = get_package_share_directory('robo_moveit_config')

    xacro_file = os.path.join(
        robot_description_pkg, 'urdf', 'Arm_assembly_main.urdf.xacro'
    )

    robot_description_content = Command([
        'xacro ', xacro_file,
        ' real_hw_test:=true',
        ' is_ignition:=false',
    ])
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    ros2_controllers_real_hw = os.path.join(
        moveit_config_pkg, 'config', 'ros2_controllers_real_hw.yaml'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, ros2_controllers_real_hw],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_servo_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_servo_controller'],
        output='screen',
    )

    mock_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mock_arm_controller'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    robot_description_semantic = {
        'robot_description_semantic': open(
            os.path.join(moveit_config_pkg, 'config', 'robot_description.srdf')
        ).read()
    }

    kinematics_yaml = load_yaml('robo_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            'robo_moveit_config', 'config/joint_limits.yaml'
        )
    }
    moveit_controllers_real_hw = load_yaml(
        'robo_moveit_config', 'config/moveit_controllers_real_hw.yaml'
    )
    ompl_planning_yaml = load_yaml('robo_moveit_config', 'config/ompl_planning.yaml')

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            joint_limits_yaml,
            moveit_controllers_real_hw,
            ompl_planning_yaml,
            {'use_sim_time': False},
            {'move_group.planning_plugin': 'ompl_interface/OMPLPlanner'},
            {'move_group.request_adapters': ' '.join([
                'default_planner_request_adapters/ResolveConstraintFrames',
                'default_planner_request_adapters/FixWorkspaceBounds',
                'default_planner_request_adapters/FixStartStateBounds',
                'default_planner_request_adapters/FixStartStateCollision',
                'default_planner_request_adapters/FixStartStatePathConstraints',
                'default_planner_request_adapters/AddTimeOptimalParameterization',
            ])},
            {"moveit_controller_manager":
                "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

    rviz_config = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_servo_controller_spawner,
        mock_arm_controller_spawner,
        gripper_controller_spawner,
        move_group_node,
        rviz_node,
    ])