import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command
)
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path')
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.0',
                 description='z component of the robot pose.'))


def launch_setup(context, *args, **kwargs):
    # Packages
    pkg_clearpath_nav2_test = get_package_share_directory('clearpath_nav2_test')
    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')
    
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'fake_params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    map_yaml_file = LaunchConfiguration('map')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace

    tf_broadcast = Node(
        package='clearpath_nav2_test',
        executable='tf_broadcast.py',
        name='tf_broadcast',
        output='screen'
    )

    # Bridge node for Gazebo to ROS TF
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[
            {'config_file': PathJoinSubstitution([pkg_clearpath_nav2_test, 'config', 'j100', 'tf_bridge.yaml'])}
        ]
    )

    # Group
    group = GroupAction([
        # PushRosNamespace(namespace),
        params_declare,
        bridge_node,
        tf_broadcast,
        # TF relay is now handled by tf_broadcast.py
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        #     parameters=[parameter_file],
        #     output='screen'
        #     ),
        # Robot state publisher not needed - robot already publishes its own TF
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{
        #         'robot_description': Command(['xacro', ' ', xacro_path])
        #     }]
        # ),
        # Node(
        #     package='lio_sam',
        #     executable='lio_sam_imuPreintegration',
        #     name='lio_sam_imuPreintegration',
        #     parameters=[parameter_file],
        #     remappings=[
        #         ('/tf', '/j100_0000/tf'),
        #         ('/tf_static', '/j100_0000/tf_static'),
        #     ],
        #     output='screen'
        # ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            remappings=[
                ('/tf', '/j100_0000/tf'),
                ('/tf_static', '/j100_0000/tf_static'),
            ],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            remappings=[
                ('/tf', '/j100_0000/tf'),
                ('/tf_static', '/j100_0000/tf_static'),
            ],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            remappings=[
                ('/tf', '/j100_0000/tf'),
                ('/tf_static', '/j100_0000/tf_static'),
            ],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])

    return [group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld