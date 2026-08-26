from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gpu_offload = SetEnvironmentVariable(name="__NV_PRIME_RENDER_OFFLOAD", value="1")
    glx_vendor = SetEnvironmentVariable(name="__GLX_VENDOR_LIBRARY_NAME", value="nvidia")
    
    append_models_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([FindPackageShare('gazebo_perception'), 'models']),
    )
    append_brick_models_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([FindPackageShare('gazebo_perception'), 'models', 'brick_models']),
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare('gazebo_perception'), 'worlds', 'brick_world.sdf']
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        # Using a strict list of tuples instead of dict.items()
        launch_arguments=[('gz_args', ['-r ', world_path])],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rgbd_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock', 
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
    )

    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--x", "0.0",
            "--y", "-0.9505",
            "--z", "0.9505",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "-2.35619",
            "--frame-id", "map",
            "--child-frame-id", "camera_rig/camera_link/rgbd_camera",
        ],
    )

    point_cloud_processor = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    ('rgb/image_rect_color', '/rgbd_camera/image'),
                    ('depth_registered/image_rect', '/rgbd_camera/depth_image'),
                    ('rgb/camera_info', '/rgbd_camera/camera_info'),
                    ('points', '/rgbd_camera/points')
                ],
                parameters=[{'use_sim_time': True}]
            ),
        ],
        output='screen',
    )
    
    inference_node = Node(
        package="gazebo_perception",
        executable="inference_node",
        name="inference_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    server_node = Node(
            package="gazebo_perception",
            executable="perception_server",
            name="perception_server",
            output="screen"
        )
    

    return LaunchDescription([
        gpu_offload,
        glx_vendor,
        append_models_path,
        append_brick_models_path,
        gz_sim,
        bridge_node,
        tf_node,
        point_cloud_processor,
        inference_node
    ])
