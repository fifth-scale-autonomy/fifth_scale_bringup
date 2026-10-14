"""카메라만 켜고 RViz 로 각도와 노출을 확인한다 (라이다·검출·차량 없이).

    ros2 launch fifth_scale_bringup camera_view.launch.py                    # extrinsics.yaml 의 카메라 각도
    ros2 launch fifth_scale_bringup camera_view.launch.py camera_pitch:=3.0  # 숙인 각을 바꿔 선이 맞는지 비교
    ros2 launch fifth_scale_bringup camera_view.launch.py exposure_control:=true   # 지면 기준 노출 제어 켜기

RViz 의 "camera (각도·노출 확인)" 화면 (camera_overlay 노드가 그림)
  - 주황 가로선 = 앞쪽 거리 2~8 m, 초록 세로선 = 좌우 y 0, +-1, +-2 m (base_link = 뒷바퀴 축 가운데 아래 땅)
    땅에 재어 둔 표시와 맞으면 각도가 맞다. 먼 선만 어긋나면 pitch, 한쪽으로 기울면 yaw 가 틀린 것.
  - 노랑 점선 = 지평선 (평지에서 먼 지평선과 비교)
  - 보라 = 포화 (Y >= 230). 위 글자 = 지면 밝기 p99 / 포화 비율, 카메라 노출 모드·시간·gain (실제 값)
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PKG = 'fifth_scale_bringup'


def setup(context):
    arg = lambda n: context.launch_configurations[n]  # noqa: E731
    share = get_package_share_directory(PKG)
    with open(os.path.expanduser(arg('extrinsics'))) as f:
        frames = yaml.safe_load(f)
    for key, name in (('pitch', 'camera_pitch'), ('yaw', 'camera_yaw')):
        if arg(name) not in ('', 'none'):
            frames['camera_link'][key] = math.radians(float(arg(name)))
    actions = [GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'camera.launch.py')),
        launch_arguments={'device': arg('device')}.items())])]
    for name, t in frames.items():                            # 카메라 각도 TF (라이다 TF 도 같이, 쓰이지 않음)
        args = []
        for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            args += [f'--{k}', str(float(t[k]))]
        args += ['--frame-id', t['parent'], '--child-frame-id', t['child']]
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher', name=f'tf_{name}',
                            arguments=args, output='log'))
    actions += [
        Node(package='band_track_detection', executable='camera_overlay', name='camera_overlay', output='screen',
             parameters=[{'device': arg('device')}]),
    ]
    if arg('exposure_control').lower() == 'true':
        actions.append(Node(package='band_track_detection', executable='exposure_control', name='exposure_control',
                            output='screen'))
    if arg('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            arguments=['-d', os.path.join(share, 'rviz', 'camera_view.rviz')]))
    return actions


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/brio'),
        DeclareLaunchArgument('extrinsics', default_value=os.path.join(share, 'config', 'extrinsics.yaml')),
        DeclareLaunchArgument('camera_pitch', default_value='',
                              description='카메라 숙인 각 (도). 비우면 extrinsics.yaml 값'),
        DeclareLaunchArgument('camera_yaw', default_value='', description='카메라 yaw (도). 비우면 extrinsics.yaml 값'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('exposure_control', default_value='false',
                              description='지면 기준 노출 제어 (exposure_control). 끄면 카메라 내장 자동 노출'),
        OpaqueFunction(function=setup),
    ])
