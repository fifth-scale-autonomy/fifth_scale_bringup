"""야외 주행 bag 을 재생하면서 지금 경로 생성(dtr_planner)을 다시 돌려 RViz 로 본다. 차량·센서 없이 실행.

    ros2 launch fifth_scale_bringup replay.launch.py                                  # ~/bags/20261012_outdoor_1456
    ros2 launch fifth_scale_bringup replay.launch.py bag:=~/bags/20261012_outdoor_1458 rate:=0.5
    ros2 launch fifth_scale_bringup replay.launch.py clear_margin:=0.20              # 여유를 바꿔 보기
    ros2 launch fifth_scale_bringup replay.launch.py camera_pitch:=3.1               # 카메라 기울기를 바꿔 정합 보기

fusion:=true (기본): bag 의 YOLO 검출(/cone/detections)·라이다·카메라를 재생하고 정합(cone_fusion)부터 다시 돌린다.
  왼쪽 카메라 화면 "fusion (YOLO + lidar)" 에 정합 모습이 보인다 (초록 박스 = 라이다 군집과 짝, 빨강 = 짝 없음,
  + = 이미지에 투영한 라이다 군집, 박스 아래 = 라이다 거리). 주황 원기둥 = 추적 중인 콘.
fusion:=false: 그때 기록된 콘 위치(/cone/positions)를 그대로 쓰고 경로만 다시 만든다.
어느 쪽이든 그때 기록된 경로(/planning/*)는 재생하지 않는다.
  - 새 경로 (초록 선 + 마커): 지금 설정. 연두 점선 = 뒷바퀴 자국, 하늘색 점선 = 앞바퀴 자국
  - 예전 경로 (빨간 선, compare:=true): 뒤축 경로만 검사하고 콘 바닥 반지름 0.20 인 예전 설정
  - 흰 띠 경계 (track:=true): 3D 화면 파랑(왼쪽)/노랑(오른쪽) 선, 회색 칸 = 판단 불가(포화).
    왼쪽 "track (지면 투영)" = 위에서 내려다본 영상: 초록 = 띠 응답, 빨강 점 = 띠 중심, 파랑/노랑 = 경계, 보라 = 판단 불가
  - 차량 모양: 흰 차체(앞쪽 끝 주황), 검은 바퀴(앞바퀴는 기록된 실제 조향각), 빨간 점 = 뒤축 중심(경로 기준점),
    파랑 = 라이다, 초록 = 카메라, 위 글자 = 기록된 속도·조향각
bag 시간(--clock)으로 돌리므로 노드와 RViz 는 use_sim_time 을 쓴다. 끝나면 처음부터 다시 재생한다 (loop:=false 로 끔).

화면 녹화용 (추론과 RViz 를 따로 돌린다. 둘을 같이 돌리며 화면까지 녹화하면 추론이 밀려 끊겨 보인다):
    ros2 launch fifth_scale_bringup replay.launch.py bag:=~/bags/X save:=true rate:=0.5   # 1) 결과 저장 -> ~/bags/X_view
    ros2 launch fifth_scale_bringup replay.launch.py bag:=~/bags/X_view view:=true        # 2) 저장한 결과만 RViz 로 재생
  save: RViz 없이 추론만 돌리고 RViz 가 그리는 토픽 (VIEW_TOPICS) 을 bag 시간으로 기록한다. 재생이 끝나면 저절로 끝남.
        rate:=0.5 면 영상마다 처리 시간이 두 배라 빠지는 프레임이 없고, 기록은 bag 시간이라 보기 재생은 원래 속도.
        save:=true 면 <bag>_view, 경로를 주면 그 경로 (이미 있으면 멈춤, 지우지 않음).
  view: 노드 없이 결과 bag 과 RViz 만 (지금 extrinsics.yaml 의 고정 TF 는 발행).
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown, TimerAction
from launch_ros.actions import Node

PKG = 'fifth_scale_bringup'
# /tf_static 은 재생하지 않고 지금 extrinsics.yaml (+ camera_pitch / camera_yaw 덮어쓰기) 로 발행한다
TOPICS = ['/scan_filtered', '/image_raw', '/camera_info', '/tf', '/odom',
          '/vehicle/steering_angle', '/vehicle/speed_mps']
# save 모드에서 기록하는 토픽 = replay.rviz 가 그리는 것 (원본 카메라 영상은 RViz 에서 꺼져 있어 뺀다)
VIEW_TOPICS = ['/scan_filtered', '/tf', '/vehicle/markers', '/track/markers', '/cone/markers', '/planning/markers',
               '/planning/path', '/planning_old/path', '/cone/fusion_image', '/track/debug_bev']


def static_tf_nodes(path, pitch_deg, yaw_deg):
    with open(path) as f:
        frames = yaml.safe_load(f)
    if pitch_deg not in ('', 'none'):
        frames['camera_link']['pitch'] = math.radians(float(pitch_deg))
    if yaw_deg not in ('', 'none'):
        frames['camera_link']['yaw'] = math.radians(float(yaw_deg))
    nodes = []
    for name, t in frames.items():
        args = []
        for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            args += [f'--{key}', str(float(t[key]))]
        args += ['--frame-id', t['parent'], '--child-frame-id', t['child']]
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name=f'tf_{name}',
                          arguments=args, output='log'))
    cam = frames['camera_link']
    print(f'[replay] 카메라 pitch {math.degrees(cam["pitch"]):.2f} 도, yaw {math.degrees(cam["yaw"]):.2f} 도')
    return nodes


def rviz_node():
    return Node(package='rviz2', executable='rviz2', name='rviz2', parameters=[{'use_sim_time': True}],
                arguments=['-d', os.path.join(get_package_share_directory(PKG), 'rviz', 'replay.rviz')])


def setup(context):
    arg = lambda n: context.launch_configurations[n]  # noqa: E731
    bag = os.path.expanduser(arg('bag'))
    if not os.path.isdir(bag):
        raise RuntimeError(f'bag 폴더가 없습니다: {bag}')
    tf_nodes = static_tf_nodes(os.path.expanduser(arg('extrinsics')), arg('camera_pitch'), arg('camera_yaw'))
    if arg('view').lower() == 'true':
        # 저장한 결과만 재생 (추론 노드 없음)
        play = ['ros2', 'bag', 'play', bag, '--clock', '--rate', arg('rate')]
        if arg('loop').lower() == 'true':
            play.append('--loop')
        return tf_nodes + [ExecuteProcess(cmd=play, output='screen'), rviz_node()]
    save = arg('save').strip()
    out = ''
    if save.lower() not in ('', 'false'):
        out = bag.rstrip('/') + '_view' if save.lower() == 'true' else os.path.expanduser(save)
        if os.path.exists(out):
            raise RuntimeError(f'저장할 곳이 이미 있습니다 (지우지 않음): {out}')
        print(f'[replay] 결과 저장 -> {out} (RViz 끔, 재생이 끝나면 종료)')
    fusion = arg('fusion').lower() == 'true'
    topics = TOPICS + (['/cone/detections'] if fusion else ['/cone/positions'])
    play = ['ros2', 'bag', 'play', bag, '--clock', '--rate', arg('rate'), '--topics'] + topics
    if arg('loop').lower() == 'true' and not out:
        play.append('--loop')
    params = {'use_sim_time': True, 'lookahead': float(arg('lookahead')), 'wheelbase': float(arg('wheelbase')),
              'front_axle': float(arg('wheelbase')), 'vehicle_width': float(arg('vehicle_width')),
              'center_weight': float(arg('center_weight')), 'cone_clear_radius': float(arg('cone_clear_radius')),
              'clear_margin': float(arg('clear_margin'))}
    # 결과 저장: 재생이 끝나면 처리 중인 마지막 프레임을 기다렸다가 (3 s) 전체 종료 -> 기록기가 bag 을 닫는다
    on_exit = [TimerAction(period=3.0, actions=[Shutdown(reason='재생 끝, 결과 저장 완료')])] if out else None
    actions = tf_nodes + [
        ExecuteProcess(cmd=play, output='screen', on_exit=on_exit),
        Node(package='dtr_controller', executable='dtr_planner', name='dtr_planner', parameters=[params],
             output='screen'),
    ]
    if out:
        actions.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--use-sim-time', '-o', out]
                                      + VIEW_TOPICS, output='screen'))
    if fusion:
        # 실차와 같은 정합 노드. 디버그 이미지는 검출 노드가 아니라 재생하는 원본 영상 위에 그린다
        actions.append(Node(package='lavacone_fusion', executable='cone_fusion', name='cone_fusion',
                            output='screen',
                            parameters=[{'use_sim_time': True, 'publish_image': True, 'image_topic': '/image_raw',
                                         'cone_height': 0.70, 'track_memory': 3.0}]))
    if arg('track').lower() == 'true':
        # 흰 띠 트랙 경계 검출. 지면 투영 디버그 영상(/track/debug_bev)도 낸다
        track_yaml = os.path.join(get_package_share_directory('band_track_detection'), 'config', 'track.yaml')
        actions.append(Node(package='band_track_detection', executable='track_detector', name='track_detector',
                            output='screen', parameters=[track_yaml, {'use_sim_time': True, 'publish_debug': True}]))
    if arg('rviz').lower() == 'true' or out:
        actions.append(Node(package=PKG, executable='vehicle_marker', name='vehicle_marker',
                            parameters=[{'use_sim_time': True, 'wheelbase': float(arg('wheelbase')),
                                         'width': float(arg('vehicle_width'))}]))
    if arg('rviz').lower() == 'true' and not out:
        actions.append(rviz_node())
    if arg('compare').lower() == 'true':
        old = dict(params, front_axle=0.0, cone_clear_radius=0.20, gate_max_ratio=1.5)
        outs = ['path', 'markers', 'steering_angle', 'curvature', 'target', 'track_width', 'speed_cap']
        actions.append(Node(package='dtr_controller', executable='dtr_planner', name='dtr_planner_old',
                            parameters=[old],
                            remappings=[(f'/planning/{t}', f'/planning_old/{t}') for t in outs],
                            arguments=['--ros-args', '--log-level', 'warn']))
    return actions


def generate_launch_description():
    home = os.path.expanduser('~')
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value=os.path.join(home, 'bags', '20261012_outdoor_1456')),
        DeclareLaunchArgument('rate', default_value='1.0', description='재생 속도 (0.5 = 절반)'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('compare', default_value='true', description='예전 설정 경로(빨강)도 같이 그림'),
        DeclareLaunchArgument('fusion', default_value='true',
                              description='true: 정합부터 다시 돌려 카메라 화면에 표시, false: 기록된 콘 위치 사용'),
        DeclareLaunchArgument('track', default_value='true',
                              description='흰 띠 트랙 경계 검출 (RViz 경계선 + 왼쪽 "track (지면 투영)" 화면)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('save', default_value='',
                              description='true 또는 경로: RViz 없이 추론 결과를 bag 으로 저장 (화면 녹화용, 기본 끔)'),
        DeclareLaunchArgument('view', default_value='false',
                              description='true: bag 이 save 로 만든 결과 bag. 노드 없이 RViz 로만 재생'),
        DeclareLaunchArgument('extrinsics', default_value=os.path.join(
            get_package_share_directory(PKG), 'config', 'extrinsics.yaml')),
        DeclareLaunchArgument('camera_pitch', default_value='',
                              description='카메라 숙인 각 (도). 비우면 extrinsics.yaml 값. 정합 화면으로 비교할 때'),
        DeclareLaunchArgument('camera_yaw', default_value='', description='카메라 yaw (도). 비우면 extrinsics.yaml 값'),
        DeclareLaunchArgument('lookahead', default_value='2.5'),
        DeclareLaunchArgument('wheelbase', default_value='0.75'),
        DeclareLaunchArgument('vehicle_width', default_value='0.80'),
        DeclareLaunchArgument('center_weight', default_value='0.05'),
        DeclareLaunchArgument('cone_clear_radius', default_value='0.25'),
        DeclareLaunchArgument('clear_margin', default_value='0.15'),
        OpaqueFunction(function=setup),
    ])
