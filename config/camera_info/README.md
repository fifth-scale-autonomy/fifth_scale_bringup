2단계 캘리브레이션 결과(brio_1280x720.yaml, brio_1920x1080.yaml)를 여기에 둔다.
파일을 추가한 뒤 `colcon build --packages-select fifth_scale_bringup` 을 한 번 실행해야 설치 경로에 반영된다.
파일이 없으면 카메라 드라이버는 빈 camera_info 를 내고, 투영/정합 노드는 공칭 화각(대각 87도) 임시 값을 쓴다 (`fallback_dfov_deg`).
