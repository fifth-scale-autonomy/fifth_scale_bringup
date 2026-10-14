"""라이다, 카메라, TF, 콘 검출, 투영 확인, 박스-라이다 정합, RViz 를 한 번에 띄운다.

    ros2 launch fifth_scale_bringup bringup.launch.py                         # 640 모델, 1280x720
    ros2 launch fifth_scale_bringup bringup.launch.py profile:=960            # 960 모델, 1920x1080
    ros2 launch fifth_scale_bringup bringup.launch.py variant:=aug            # 증강 모델
    ros2 launch fifth_scale_bringup bringup.launch.py size:=s backend:=openvino   # s 모델, OpenVINO
    ros2 launch fifth_scale_bringup bringup.launch.py tune_extrinsics:=true   # 외부 파라미터 조정 모드
    ros2 launch fifth_scale_bringup bringup.launch.py drive:=true vehicle:=true   # 차량 연결 (정지 상태로 시작)

기본값: nano 모델(cone_n_640_base) + ONNX(onnxruntime CPU). 사용자 지정(2026-10-07).
모델 파일 이름: size=n 이면 cone_n_<profile>_<variant>.pt, size=s 이면 cone_<profile>_<variant>.pt
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'fifth_scale_bringup'
# rect: 16:9 카메라에 맞춘 [높이, 너비] 입력. 정사각형 입력은 위아래 44% 가 여백이다
PROFILES = {'640': {'width': '1280', 'imgsz': 640, 'rect': [384, 640]},
            '960': {'width': '1920', 'imgsz': 960, 'rect': [544, 960]}}


def setup(context):
    share = get_package_share_directory(PKG)
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    profile = arg('profile')
    if profile not in PROFILES:
        raise RuntimeError(f'profile 은 {list(PROFILES)} 중 하나: {profile}')
    prof = PROFILES[profile]
    size = arg('size')
    if size not in ('n', 's'):
        raise RuntimeError(f'size 는 n 또는 s: {size}')
    name = f'cone_{"n_" if size == "n" else ""}{profile}_{arg("variant")}.pt'
    model = arg('model') or os.path.join(arg('model_dir'), name)
    if not os.path.exists(os.path.expanduser(model)):
        raise RuntimeError(f'모델 파일이 없습니다: {model} (lavacone_fusion 에는 cone_n_640_base 만 동봉. '
                           f'다른 모델은 model_dir:= 또는 model:= 로 지정)')
    tune = arg('tune_extrinsics')
    venv_python = os.path.join(arg('venv'), 'bin', 'python3')
    cpus = arg('detector_cpus')
    if not os.path.exists(venv_python):
        raise RuntimeError(f'venv 파이썬이 없습니다: {venv_python}')

    launch_dir = os.path.join(share, 'launch')
    # 하위 launch 에 넘긴 인자(rviz:=false 등)가 이 파일의 같은 이름 설정을 덮어쓰지 않도록 범위를 가둔다
    actions = [
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensors.launch.py')),
            launch_arguments={'tune_extrinsics': tune, 'rviz': 'false'}.items())]),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'camera.launch.py')),
            launch_arguments={'width': prof['width'], 'driver': arg('camera_driver')}.items())]),
        Node(
            package='lavacone_fusion', executable='cone_detector', name='cone_detector',
            # ultralytics/OpenVINO 가 있는 venv 로 실행. 검출이 모든 코어를 쓰면 카메라 디코딩이 밀려
            # 이미지 지연이 커지므로 detector_cpus 코어에만 묶는다 (빈 값이면 제한 없음)
            prefix=(f'taskset -c {cpus} ' if cpus not in ('', 'none') else '') + venv_python,
            parameters=[{
                'model': model,
                'imgsz': prof['rect'] if arg('rect').lower() == 'true' else prof['imgsz'],
                'conf': float(arg('conf')),
                'backend': arg('backend'),
                'publish_debug': arg('debug_image').lower() == 'true',
                'timing_csv': arg('timing_csv'),
                'ort_spinning': arg('ort_spinning').lower() == 'true',
            }],
            output='screen'),
        Node(
            package='lavacone_fusion', executable='cone_fusion', name='cone_fusion',
            parameters=[{'publish_image': arg('debug_image').lower() == 'true',   # 디버그 이미지 위에 그린다
                         'cone_radius_at_scan': float(arg('cone_radius_at_scan')),
                         'cone_height': float(arg('cone_height')),
                         'track_memory': float(arg('track_memory'))}],
            output='screen'),
        Node(
            package='dtr_controller', executable='dtr_planner', name='dtr_planner',
            parameters=[{'lookahead': float(arg('lookahead')), 'wheelbase': float(arg('wheelbase')),
                         'vehicle_width': float(arg('vehicle_width')), 'center_weight': float(arg('center_weight')),
                         'front_axle': float(arg('wheelbase')),
                         'cone_clear_radius': float(arg('cone_clear_radius')),
                         'clear_margin': float(arg('clear_margin'))}],
            condition=IfCondition(LaunchConfiguration('planner')),
            output='screen'),
        Node(
            package='dtr_controller', executable='drive_command', name='drive_command',
            parameters=[{'use_speed_cap': arg('speed_cap').lower() == 'true'}],
            condition=IfCondition(LaunchConfiguration('drive')),
            output='screen'),
        Node(
            # 차량 피드백(/vehicle/speed_mps, /vehicle/steering_angle) -> TF odom -> base_link.
            # 피드백이 없으면 아무것도 내지 않고, cone_fusion 은 이동 보정 없이 동작한다
            package=PKG, executable='wheel_odometry', name='wheel_odometry',
            parameters=[{'wheelbase': float(arg('wheelbase')), 'steer_scale': float(arg('odom_steer_scale'))}],
            condition=IfCondition(LaunchConfiguration('odom')),
            output='screen'),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('scale_vehicle_control'), 'launch', 'bringup.launch.py')),
            launch_arguments={'wheel_diameter': arg('wheel_diameter')}.items())],
            condition=IfCondition(LaunchConfiguration('vehicle'))),
        Node(
            package='lavacone_fusion', executable='lidar_projection', name='lidar_projection',
            parameters=[{'tune': tune.lower() == 'true'}],
            # 외부 파라미터 조정 모드는 투영 이미지를 보며 맞추므로 projection 값과 관계없이 켠다
            condition=IfCondition(str(arg('projection').lower() == 'true' or tune.lower() == 'true')),
            output='screen'),
        Node(
            # 지면 기준 노출 제어 (카메라를 수동 노출로 두고 지면 밝기로 노출 시간·gain 을 정함). 기본 꺼짐
            package='band_track_detection', executable='exposure_control', name='exposure_control',
            condition=IfCondition(LaunchConfiguration('exposure_control')),
            output='screen'),
        Node(
            # 흰 띠 트랙 경계 검출 (band_track_detection). 기본 꺼짐
            package='band_track_detection', executable='track_detector', name='track_detector',
            parameters=[os.path.join(get_package_share_directory('band_track_detection'), 'config',
                                     'track.yaml'),
                        {'publish_debug': arg('debug_image').lower() == 'true'}],
            condition=IfCondition(LaunchConfiguration('track')),
            output='screen'),
        Node(
            # RViz 차량 모양 (차체, 바퀴 조향, 센서 위치, 속도·조향 글자)
            package=PKG, executable='vehicle_marker', name='vehicle_marker',
            parameters=[{'wheelbase': float(arg('wheelbase')), 'width': float(arg('vehicle_width'))}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'fusion.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(
            package='rqt_reconfigure', executable='rqt_reconfigure', name='rqt_reconfigure',
            condition=IfCondition(LaunchConfiguration('tune_extrinsics'))),
    ]
    return actions


def generate_launch_description():
    home = os.path.expanduser('~')
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='640', description='640(1280x720) 또는 960(1920x1080)'),
        DeclareLaunchArgument('rect', default_value='true',
                              description='true: 16:9 입력(640 -> 384x640), false: 정사각형 입력'),
        DeclareLaunchArgument('size', default_value='n', description='n(nano) 또는 s(small)'),
        DeclareLaunchArgument('variant', default_value='base', description='base 또는 aug'),
        DeclareLaunchArgument('camera_driver', default_value='v4l2_camera', description='v4l2_camera 또는 usb_cam'),
        DeclareLaunchArgument('model', default_value='', description='모델 경로 직접 지정 (size/profile/variant 무시)'),
        DeclareLaunchArgument('model_dir', default_value=os.path.join(
            get_package_share_directory('lavacone_fusion'), 'models'),
            description='모델 폴더. 기본은 lavacone_fusion 동봉 모델 (nano 640 base 만). 다른 모델은 이 폴더를 바꾼다'),
        DeclareLaunchArgument('backend', default_value='onnx', description='onnx, openvino 또는 pytorch'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        DeclareLaunchArgument('debug_image', default_value='false',
                              description='검출·정합 확인 이미지. 켜면 CPU 약 0.25코어 추가'),
        DeclareLaunchArgument('ort_spinning', default_value='false',
                              description='onnxruntime 대기 중 spin. 켜면 추론 약 9 ms 빠르고 CPU 약 2배'),
        DeclareLaunchArgument('timing_csv', default_value='', description='프레임별 처리 시간 기록 파일'),
        DeclareLaunchArgument('cone_radius_at_scan', default_value='0.0',
                              description='스캔 높이(0.48 m)에서 콘 반지름 m. 0 이면 점 평균'),
        DeclareLaunchArgument('cone_height', default_value='0.70', description='콘 높이 m (마커/출력용)'),
        DeclareLaunchArgument('projection', default_value='false',
                              description='라이다 투영 확인 이미지 (4단계 캘리브레이션용). 켜면 CPU 약 0.2코어 추가'),
        DeclareLaunchArgument('tune_extrinsics', default_value='false'),
        DeclareLaunchArgument('planner', default_value='true', description='DTR 경로 생성 노드'),
        DeclareLaunchArgument('lookahead', default_value='2.5', description='Pure Pursuit 앞보기 거리 m'),
        DeclareLaunchArgument('wheelbase', default_value='0.75', description='축거 m (1/5 차량 실측)'),
        DeclareLaunchArgument('odom_steer_scale', default_value='1.0',
                              description='주행 거리계 조향 피드백 배율 (띠 bag 추정 0.7, 실측 전이라 기본 1.0)'),
        DeclareLaunchArgument('vehicle_width', default_value='0.80',
                              description='차폭 m. 경로가 콘에서 차폭/2 + 콘 반지름 + 여유 만큼 떨어지게 한다 (0 이면 중앙선)'),
        DeclareLaunchArgument('cone_clear_radius', default_value='0.25',
                              description='콘 바닥 반지름 m (차체가 피할 크기). 대형콘 바닥 약 38 cm 정사각형'),
        DeclareLaunchArgument('clear_margin', default_value='0.15',
                              description='차체(네 바퀴 포함) 와 콘 바닥 사이에 남길 여유 m'),
        DeclareLaunchArgument('center_weight', default_value='0.05',
                              description='0 이면 커브에서 차폭 여유를 다 써서 덜 휘게, 클수록 중앙선을 따름'),
        DeclareLaunchArgument('drive', default_value='false',
                              description='조향각 -> /ackermann_cmd 변환 노드. enable 파라미터를 켜기 전에는 정지 명령만 보냄'),
        DeclareLaunchArgument('wheel_diameter', default_value='0.27',
                              description='바퀴 지름 m (실측). 주면 차량 속도를 m/s 로 표시'),
        DeclareLaunchArgument('vehicle', default_value='false',
                              description='차량 제어기 드라이버(scale_vehicle_node, /dev/car)'),
        DeclareLaunchArgument('odom', default_value='true',
                              description='바퀴 주행 거리계 (차량 피드백이 있을 때만 동작, 콘 추적 이동 보정에 씀)'),
        DeclareLaunchArgument('track_memory', default_value='3.0',
                              description='이동 보정 중 안 보이는 확인 콘을 유지하는 시간 s'),
        DeclareLaunchArgument('speed_cap', default_value='true',
                              description='자율 모드에서 경로 곡률로 속도 단계 상한 적용'),
        DeclareLaunchArgument('exposure_control', default_value='false',
                              description='지면 기준 노출 제어 (exposure_control). 끄면 카메라 내장 자동 노출'),
        DeclareLaunchArgument('track', default_value='false',
                              description='흰 띠 트랙 경계 검출 (track_detector, config/track.yaml)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('detector_cpus', default_value='none',
                              description='검출 노드를 묶을 CPU 코어 (taskset 형식, 예: 0-2). none 이면 제한 없음'),
        DeclareLaunchArgument('venv', default_value=os.path.join(home, 'ros2_ws', 'venv')),
        OpaqueFunction(function=setup),
    ])
