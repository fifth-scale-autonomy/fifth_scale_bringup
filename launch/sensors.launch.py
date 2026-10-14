"""라이다 드라이버, 각도 필터, 센서 정적 TF를 띄운다.

    ros2 launch fifth_scale_bringup sensors.launch.py
    ros2 launch fifth_scale_bringup sensors.launch.py rviz:=true
    ros2 launch fifth_scale_bringup sensors.launch.py serial_port:=/dev/ttyUSB1   # udev 규칙 설치 전
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'fifth_scale_bringup'


def static_tf_nodes(context):
    path = LaunchConfiguration('extrinsics').perform(context)
    with open(path) as f:
        frames = yaml.safe_load(f)
    # 외부 파라미터 조정 중에는 lidar_projection 이 base_link -> camera_link 를 발행한다
    tuning = LaunchConfiguration('tune_extrinsics').perform(context).lower() == 'true'
    nodes = []
    for name, t in frames.items():
        if tuning and name == 'camera_link':
            continue
        args = []
        for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            args += [f'--{key}', str(float(t[key]))]
        args += ['--frame-id', t['parent'], '--child-frame-id', t['child']]
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'tf_{name}',
            arguments=args,
            output='log',
        ))
    return nodes


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('extrinsics',
                              default_value=os.path.join(share, 'config', 'extrinsics.yaml')),
        DeclareLaunchArgument('laser_filter',
                              default_value=os.path.join(share, 'config', 'laser_filter.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('tune_extrinsics', default_value='false'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': 1000000,     # S2 전용 속도
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'DenseBoost',      # 32 kHz, 10 Hz 에서 스캔당 약 3200점
            }],
            output='screen',
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter',
            parameters=[LaunchConfiguration('laser_filter')],
            remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
            output='screen',
        ),
        OpaqueFunction(function=static_tf_nodes),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'lidar_check.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
