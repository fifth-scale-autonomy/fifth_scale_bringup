"""Logitech BRIO 카메라를 띄운다.

    ros2 launch fifth_scale_bringup camera.launch.py                   # 1280x720, 10 fps, v4l2_camera
    ros2 launch fifth_scale_bringup camera.launch.py width:=1920        # 1920x1080
    ros2 launch fifth_scale_bringup camera.launch.py driver:=usb_cam    # 예전 드라이버 (비교용)

v4l2_camera 를 기본으로 쓴다. usb_cam(0.8.1)은 프레임 약 4장을 쌓아 두고 내보내 10 fps 에서
실제 지연이 약 630 ms 였고, stamp 가 영상보다 최대 0.55 s 앞서 정합에 쓸 수 없었다.
v4l2_camera(YUYV)는 실제 지연 약 95 ms(30 fps 측정), stamp 가 영상과 일치했다.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'fifth_scale_bringup'
HEIGHTS = {'1280': '720', '1920': '1080'}


def v4l2_camera(share, device, width, height, fps):
    params = os.path.join(share, 'config', f'v4l2_camera_{width}x{height}.yaml')
    # v4l2_camera 0.7.3 은 프레임레이트 파라미터가 없다. 같은 형식으로 미리 걸어 두면 유지된다.
    preset = ExecuteProcess(
        cmd=['v4l2-ctl', '-d', device,
             f'--set-fmt-video=width={width},height={height},pixelformat=YUYV', f'--set-parm={fps}'],
        output='screen')
    node = Node(
        package='v4l2_camera', executable='v4l2_camera_node', name='camera',
        parameters=[params, {'video_device': device}],
        output='screen')
    check = TimerAction(period=4.0, actions=[
        ExecuteProcess(cmd=['v4l2-ctl', '-d', device, '--get-parm',
                            '-C', 'focus_automatic_continuous,focus_absolute,exposure_dynamic_framerate'],
                       output='screen')])
    return [preset, RegisterEventHandler(OnProcessExit(target_action=preset, on_exit=[node, check]))]


def usb_cam(share, device, width, height):
    params = os.path.join(share, 'config', f'usb_cam_{width}x{height}.yaml')
    with open(os.path.join(share, 'config', 'brio_controls.yaml')) as f:
        controls = yaml.safe_load(f) or {}
    # 한 번에 여러 값을 보내면 하나만 거부돼도 전부 적용되지 않는다
    # (자동 초점이 켜져 있으면 focus_absolute 가 거부됨). 파일 순서대로 하나씩 건다.
    set_cmd = '; '.join(f'v4l2-ctl -d {device} -c {k}={v}' for k, v in controls.items())
    return [
        Node(package='usb_cam', executable='usb_cam_node_exe', name='camera',
             parameters=[params, {'video_device': device}], output='screen'),
        # usb_cam 이 장치를 연 뒤에 건다 (열 때 일부 제어값이 초기화된다)
        TimerAction(period=3.0, actions=[ExecuteProcess(cmd=['sh', '-c', set_cmd], output='screen')]),
        TimerAction(period=4.0, actions=[
            ExecuteProcess(cmd=['v4l2-ctl', '-d', device, '-C', ','.join(controls)], output='screen')]),
    ]


def setup(context):
    share = get_package_share_directory(PKG)
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    width = arg('width')
    if width not in HEIGHTS:
        raise RuntimeError(f'width 는 {list(HEIGHTS)} 중 하나여야 합니다: {width}')
    driver = arg('driver')
    if driver == 'v4l2_camera':
        return v4l2_camera(share, arg('device'), width, HEIGHTS[width], arg('fps'))
    if driver == 'usb_cam':
        return usb_cam(share, arg('device'), width, HEIGHTS[width])
    raise RuntimeError(f'driver 는 v4l2_camera 또는 usb_cam: {driver}')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='1280', description='1280 또는 1920'),
        DeclareLaunchArgument('fps', default_value='10', description='라이다(10 Hz)에 맞춤'),
        DeclareLaunchArgument('driver', default_value='v4l2_camera', description='v4l2_camera 또는 usb_cam'),
        DeclareLaunchArgument('device', default_value='/dev/brio'),
        OpaqueFunction(function=setup),
    ])
