"""
LIO-SAM Ground Truth SLAM Launch File

This launch file runs LIO-SAM's mapping pipeline but uses Gazebo ground truth
for localization instead of IMU preintegration. This is useful for:
- Testing mapping without localization drift
- Evaluating pure LiDAR mapping quality
- Building maps with perfect localization in simulation

The ground truth pose from Gazebo replaces the IMU odometry, so:
- Loop closure still works (uses LiDAR matching)
- Map building works with perfect poses
- No IMU drift to worry about
"""

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
    PathJoinSubstitution
)
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path'),
    DeclareLaunchArgument('publish_tf', default_value='true',
                          choices=['true', 'false'],
                          description='Publish map->base_link TF from ground truth'),
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
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')
    
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'ground_truth_params.yaml'),
        description='Path to the ROS2 parameters file to use.')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    publish_tf = LaunchConfiguration('publish_tf')

    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)
    namespace = clearpath_config.system.namespace

    # Ground truth odometry publisher - replaces IMU preintegration
    ground_truth_odom = Node(
        package='lio_sam',
        executable='ground_truth_odom_publisher.py',
        name='ground_truth_odom_publisher',
        output='screen',
        parameters=[{
            'robot_frame': 'j100_0000/robot',
            'odom_topic': 'odometry/imu',
            'publish_tf': True,
        }]
    )

    # Bridge node for Gazebo to ROS TF (to get ground truth)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[
            {'config_file': PathJoinSubstitution([pkg_clearpath_nav2_test, 'config', 'j100', 'tf_bridge.yaml'])}
        ]
    )

    # LIO-SAM nodes (without imuPreintegration - using ground truth instead)
    image_projection = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[parameter_file],
        remappings=[
            ('/tf', '/j100_0000/tf'),
            ('/tf_static', '/j100_0000/tf_static'),
        ],
        output='screen'
    )

    feature_extraction = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[parameter_file],
        remappings=[
            ('/tf', '/j100_0000/tf'),
            ('/tf_static', '/j100_0000/tf_static'),
        ],
        output='screen'
    )

    map_optimization = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[parameter_file],
        remappings=[
            ('/tf', '/j100_0000/tf'),
            ('/tf_static', '/j100_0000/tf_static'),
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Group all nodes
    group = GroupAction([
        params_declare,
        bridge_node,
        ground_truth_odom,
        image_projection,
        feature_extraction,
        map_optimization,
        rviz,
    ])

    return [group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
