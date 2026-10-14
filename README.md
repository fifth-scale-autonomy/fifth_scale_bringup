# fifth_scale_bringup
#### 1/5 HENES T870 자율주행 통합 실행 — 센서·카메라 설정, 센서 장착 위치, 차량·리플레이 launch, RViz, 바퀴 주행 거리계
#### Bringup for the 1/5-scale HENES T870 autonomous car — sensor and camera config, sensor extrinsics, vehicle and bag-replay launches, RViz layouts, wheel odometry.

> **대회 목적**: 1/5 스케일 자율주행 차량 대회 (콘·흰 띠 트랙) 참가를 위해 개발한 패키지입니다.
> 구성: [lavacone_fusion](https://github.com/fifth-scale-autonomy/lavacone_fusion) (라바콘 정합),
> [band_track_detection](https://github.com/fifth-scale-autonomy/band_track_detection) (트랙 검출),
> [dtr_controller](https://github.com/fifth-scale-autonomy/dtr_controller) (경로·조향),
> [fifth_scale_bringup](https://github.com/fifth-scale-autonomy/fifth_scale_bringup) (통합 실행), 차량 드라이버 [henes_t870_driver](https://github.com/fifth-scale-autonomy/henes_t870_driver).

## 구성

| 패키지 | 역할 |
|---|---|
| lavacone_fusion | YOLO 라바콘 검출 + 라이다 정합 |
| band_track_detection | 흰 띠 트랙 경계 검출, 노출 제어 |
| dtr_controller | DTR 경로 생성, Pure Pursuit, 차량 명령 |
| scale_vehicle_control ([henes_t870_driver](https://github.com/fifth-scale-autonomy/henes_t870_driver)) | 차량 드라이버 (아두이노 시리얼) |
| fifth_scale_bringup (이 패키지) | 위를 묶어 실행 |

센서: RPLIDAR S2 (sllidar_ros2), Logitech BRIO (v4l2_camera, 1280x720 10 fps), 차량 제어기 (FTDI, `/dev/car`).
탑재 컴퓨터: AMD Ryzen 3 4300U (GPU 없음), Ubuntu 24.04, ROS 2 Jazzy.

## 설치

```bash
cd ~/ros2_ws/src
git clone https://github.com/fifth-scale-autonomy/lavacone_fusion
git clone https://github.com/fifth-scale-autonomy/band_track_detection
git clone https://github.com/fifth-scale-autonomy/dtr_controller
git clone https://github.com/fifth-scale-autonomy/fifth_scale_bringup
git clone https://github.com/fifth-scale-autonomy/henes_t870_driver
git clone https://github.com/Slamtec/sllidar_ros2
sudo apt install ros-jazzy-v4l2-camera ros-jazzy-laser-filters ros-jazzy-vision-msgs ros-jazzy-ackermann-msgs
cd ~/ros2_ws && colcon build && source install/setup.bash
```

- YOLO 검출은 venv 파이썬으로 돈다: `python3 -m venv ~/ros2_ws/venv` 에 ultralytics, onnxruntime 설치
  (`lavacone_fusion` README). 모델은 `lavacone_fusion/models` 에 동봉되어 있다 (다른 모델은 `model_dir:=`).
- **udev 규칙** (`udev/99-cone-perception.rules`): 파일 안 `<LIDAR_SERIAL>`, `<BRIO_SERIAL>`, `<CAR_SERIAL>` 을 각자 장치의
  일련번호로 바꾼 뒤 (찾는 방법은 파일 주석)
  `sudo cp udev/99-cone-perception.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`.
  `/dev/rplidar`, `/dev/brio`, `/dev/car` 가 생기고, launch 기본값이 이 이름을 쓴다. 사용자를 `dialout` 그룹에 넣는다.

## 사용 전에 채울 값 (차량마다 다름)

좌표계: REP-103 (x 전방, y 좌측, z 위, m / rad). **base_link 원점 = 뒤축 중심 바로 아래 지면.**
주행 거리계, Pure Pursuit, 차체 여유 계산이 모두 뒤축 기준이라 원점을 다른 곳에 두면 안 된다.
설정 파일을 고친 뒤에는 `colcon build --packages-select fifth_scale_bringup` (또는 처음부터 `--symlink-install`).

| 어디 | 값 | 이 차량 값 | 재는 법 |
|---|---|---|---|
| `config/extrinsics.yaml` `laser` | x, y, z: 라이다 회전 중심 위치, z = 스캔 평면 높이 | 0.95, 0, 0.48 | 줄자 |
| 〃 | yaw: 라이다 정면 방향 | π (로고가 차량 전방) | sllidar_ros2 는 laser +x 가 로고 반대쪽이라 로고가 전방이면 π |
| `config/extrinsics.yaml` `camera_link` | x, y, z: 렌즈 중심 위치 | 0.53, 0, 0.92 | 줄자 |
| 〃 | pitch: 아래로 숙인 각 (+ = 아래, rad) | 0.1972 (11.3°) | 각도계는 오차가 커서 맞지 않았다. 아래 "카메라 각도 확인" |
| 〃 | roll, yaw | 0, -0.0201 | yaw 는 `extrinsic_check` 로 추정 |
| `config/extrinsics.yaml` `camera_optical_frame` | 바꾸지 않는다 (광학 축 고정 회전) | | |
| `config/camera_info/brio_1280x720.yaml` | 카메라 내부 파라미터 | 없음 (임시 값) | 체커보드 캘리브레이션 (`camera_calibration`). 없으면 공칭 대각 화각 87° |
| `config/laser_filter.yaml` | 차체에 가려 늘 가까운 값이 찍히는 각도 (laser 프레임 rad) | -0.5498 ~ 0.6632 | `ros2 run lavacone_fusion scan_probe --mode occlusion --topic /scan` |
| `udev/99-cone-perception.rules` | 장치 일련번호 3 개 | | 파일 주석의 `udevadm info` |
| `bringup.launch.py` 인자 | `wheelbase` 축거 / `vehicle_width` 차폭 / `wheel_diameter` 바퀴 지름 | 0.75 / 0.80 / 0.27 m | 줄자 |
| 〃 | `cone_height` 콘 높이 / `cone_clear_radius` 콘 바닥 반지름 | 0.70 / 0.25 m | 대회 콘 규격 |
| `camera.launch.py` 인자 | `device` / `width` | `/dev/brio` / 1280 | |
| 차량 드라이버 (henes_t870_driver) | `port`, `steer_full_lock_deg` (조향 끝 각) | `/dev/car`, 20° | 드라이버 README |
| `band_track_detection/config/track.yaml` | `band_white`, `band_black`, `track_width` | 0.10, 0.05, 1.5 m | 트랙 규격 |
| `bringup.launch.py` 인자 `venv`, `model_dir` | YOLO 실행 venv, 모델 폴더 | `~/ros2_ws/venv`, lavacone_fusion 동봉 | `lavacone_fusion` README |

### 카메라 각도 확인

카메라 pitch 가 1~2° 만 틀려도 지면 거리와 띠 위치가 크게 어긋난다. 카메라를 다시 달면 다음 중 하나로 맞춘다.

1. `ros2 launch fifth_scale_bringup camera_view.launch.py camera_pitch:=<도>`: 영상에 지면 거리선 (x 2~8 m, y ±2 m)
   이 그려진다. 바닥에 표시한 거리 (테이프, 콘) 와 선이 맞는 pitch 를 찾는다.
2. 콘 두 개를 3~6 m 앞에 두고 20 s 녹화한 뒤 `ros2 run lavacone_fusion extrinsic_check <bag> --apply`
   (YOLO 박스와 라이다 거리로 pitch / yaw 추정, extrinsics.yaml 에 기록).
3. 띠 트랙이면 직선 구간에서 `/track/debug_bev` 의 두 띠가 평행하고 좌우 간격 W 가 `track_width` 와 같아지는 pitch.
   (지금 값 11.3° 는 이 방법. 7.0° 에서는 먼 곳 간격이 15% 넓었다.)

## launch

| 파일 | 내용 |
|---|---|
| `sensors.launch.py` | 라이다 드라이버 + 각도 필터 (`laser_filter.yaml`, 차체에 가리는 각도 제거) + 센서 정적 TF (`extrinsics.yaml`) |
| `camera.launch.py` | BRIO (v4l2_camera, `config/v4l2_camera_*.yaml`, 자동 초점 끔). `width:=1920` 가능 |
| `bringup.launch.py` | 센서 + 카메라 + 콘 검출·정합 + 경로 + (선택) 트랙 검출·노출 제어·차량 명령·드라이버 + 주행 거리계 + RViz |
| `replay.launch.py` | 녹화 bag 을 재생하며 정합·트랙 검출·경로를 다시 돌려 RViz 로 보기 |
| `camera_view.launch.py` | 카메라 + 지면 거리선 겹침 (카메라 각도·노출 확인) |

`bringup.launch.py` 주요 인자:

| 인자 | 기본 | 뜻 |
|---|---|---|
| `vehicle` | false | 차량 드라이버 (`/dev/car`) |
| `drive` | false | 조향각 -> 차량 명령 변환 (`drive_go` 가 실행 중일 때만 움직임) |
| `track` / `exposure_control` | false / false | 흰 띠 트랙 검출 / 지면 기준 노출 제어 (직사광에서는 켠다) |
| `debug_image` | false | 정합·트랙 확인 영상 (CPU 약 0.25 코어 추가) |
| `odom` / `odom_steer_scale` | true / 1.0 | 바퀴 주행 거리계 / 조향 피드백 배율 |
| `size` / `backend` / `profile` | n / onnx / 640 | YOLO 모델 크기 / 실행 방식 / 해상도 |
| `detector_cpus` | none | 검출 노드를 묶을 코어 (예 `0-2`) |
| `lookahead`, `vehicle_width`, `cone_clear_radius`, `clear_margin`, `center_weight` | | 경로 매개변수 (`dtr_controller` README) |
| `rviz` | true | RViz (왼쪽 Displays, 오른쪽 정합 화면·트랙 검출 화면) |

## 자주 쓰는 명령 (bash alias 예)

```bash
alias car='ros2 launch fifth_scale_bringup bringup.launch.py drive:=true vehicle:=true'
alias carview='ros2 launch fifth_scale_bringup bringup.launch.py drive:=true vehicle:=true track:=true exposure_control:=true debug_image:=true'
alias drive='ros2 run dtr_controller drive_go'      # 다른 터미널에서. 실행 중에만 주행
alias replay='ros2 launch fifth_scale_bringup replay.launch.py'
alias rsave='ros2 launch fifth_scale_bringup replay.launch.py save:=true rate:=0.5'
alias rview='ros2 launch fifth_scale_bringup replay.launch.py view:=true'
alias camview='ros2 launch fifth_scale_bringup camera_view.launch.py'
```

`carview` 와 녹화를 같이 돌리면 CPU 가 모자라 검출이 4~6 Hz 로 떨어진다. 녹화할 때는 `debug_image` 를 끈다.

## 녹화와 재생

1. 녹화: `ros2 bag record -s mcap -o ~/bags/<이름> /scan_filtered /image_raw /camera_info /cone/detections /tf /tf_static
   /odom /vehicle/speed_mps /vehicle/steering_angle ...` (영상 포함 1 분 약 1 GB).
2. 부하가 큰 상태로 녹화하면 녹화기가 영상을 몰아 받아 기록 시각이 1 초씩 뭉친다. 재생하면 같은 간격으로 나와
   검출이 3~4 Hz 로 보인다. 확인과 복구:
   `ros2 run fifth_scale_bringup bag_retime <bag> --report` (토픽별 기록 지연), `ros2 run fifth_scale_bringup bag_retime <bag>`
   (header 시각 기준 복사본 `<bag>_hdr`).
3. 재생: `ros2 launch fifth_scale_bringup replay.launch.py bag:=<bag>`. 녹화된 YOLO 검출 (`/cone/detections`) 을 그대로
   쓰고 정합·트랙 검출·경로만 다시 돌린다. `camera_pitch:=` 로 카메라 각도를 바꿔 볼 수 있다. `rpause` 대신
   `ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused`.
4. 화면 녹화용: `save:=true rate:=0.5` 로 RViz 없이 결과를 `<bag>_view` 에 저장한 뒤 `bag:=<bag>_view view:=true` 로
   RViz 만 재생한다 (추론과 화면 녹화가 CPU 를 다투지 않는다).

## 바퀴 주행 거리계 (`wheel_odometry`)

`/vehicle/speed_mps` (드라이버가 rpm 과 바퀴 지름 0.27 m 로 계산) 와 `/vehicle/steering_angle` 을 자전거 모델로 적분해
`/odom`, TF odom -> base_link 를 낸다. 콘 추적과 트랙 점 기억이 쓴다. 조향 피드백 ±2000 = ±20° 가정은 측정 전이다.
띠 bag 으로 확인했을 때 회전량이 약 1.4 배 크게 나왔다 (`odom_steer_scale:=0.7` 에서 회전 중 오차 15 -> 3.5 cm/s).
최대 조향으로 회전 지름을 재면 정확한 값을 정할 수 있다 (바퀴 20° 면 뒤축 중심 지름 약 4.1 m).

## 시험

```bash
cd ~/ros2_ws/src/fifth_scale_bringup && python3 -m pytest test -q
```

## 라이선스

GNU Affero General Public License v3.0 (`AGPL-3.0-only`, `LICENSE`). Ultralytics (AGPL-3.0) 를 쓰는 `lavacone_fusion` 을
함께 실행하는 통합 패키지이므로 같은 라이선스로 공개한다. 차량 드라이버 `henes_t870_driver` 는 별도 레포로 Apache-2.0,
라이다 드라이버 `sllidar_ros2` 는 Slamtec 의 BSD 라이선스다. YOLO 모델과 학습 데이터 (FSOCO) 의 출처는
`lavacone_fusion` README.
