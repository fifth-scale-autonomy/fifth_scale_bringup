"""bag 을 header 시각(촬영/측정 시각) 기준으로 다시 쓴다. 재생이 녹화기 수신 시각 대신 촬영 시각대로 나오게 한다.

    ros2 run fifth_scale_bringup bag_retime ~/bags/20261014_outdoor_1321            # -> ~/bags/20261014_outdoor_1321_hdr
    ros2 run fifth_scale_bringup bag_retime ~/bags/20261014_outdoor_13* --report    # 쓰지 않고 토픽별 기록 지연만 출력

왜 (2026-10-14): 이날 bag 은 녹화기가 영상을 몰아서 받았다 (기록 간격 30 ms 미만 64~77 %, 최대 1 s 공백,
촬영부터 기록까지 중앙 0.36 s). header 간격은 100 ms 로 고르다. ros2 bag play 는 기록 시각대로 내보내므로 재생도
1 초씩 몰려 나오고, 최신 프레임만 받는 track_detector 는 3~4 Hz 만 처리한다. Jazzy 의 bag play 에는 header 시각 재생
옵션이 없다.

방법
  - header 가 첫 필드인 메시지 (Image, CameraInfo, LaserScan, Odometry, Path, 검출 배열 ...) 는 기록 시각을 header.stamp 로.
  - /tf 는 안의 transform stamp 중 가장 늦은 값. header 가 없는 메시지 (/vehicle/mode 등) 와 /tf_static 은 그대로.
  - stamp 가 0 이거나 기록 시각과 --max-shift (5 s) 넘게 다르면 그대로 둔다.
  - 원본 bag 은 건드리지 않는다. 출력 폴더가 이미 있으면 멈춘다.
주의: 기록된 처리 결과 (/cone/detections, /track/points, /cone/positions ...) 도 촬영 시각에 나오므로 실차에서 있었던
처리 지연은 재생에 나타나지 않는다. 그 지연은 --report 의 '기록 지연' 으로 본다.
"""
import argparse
import heapq
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage


def header_first(type_name):
    try:
        fields = get_message(type_name).get_fields_and_field_types()
    except Exception:  # noqa: BLE001
        return False
    first = next(iter(fields.items()), (None, None))
    return first == ('header', 'std_msgs/Header')


def stamp_ns(topic, type_name, data, has_header):
    """메시지 안의 시각 (ns). 없으면 None."""
    if has_header:
        # CDR: 4 바이트 캡슐 헤더 뒤에 header.stamp (int32 sec, uint32 nanosec). data[1] 이 1 이면 little endian.
        order = '<' if data[1] == 1 else '>'
        sec, nsec = np.frombuffer(bytes(data[4:12]), dtype=np.dtype(f'{order}i4')).astype(np.int64)
        return int(sec) * 1_000_000_000 + int(nsec) % (1 << 32)
    if type_name == 'tf2_msgs/msg/TFMessage' and topic != '/tf_static':
        m = deserialize_message(data, TFMessage)
        if m.transforms:
            return max(t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec for t in m.transforms)
    return None


def open_reader(path):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return rd


def retime(path, out, max_shift, report_only):
    rd = open_reader(path)
    topics = rd.get_all_topics_and_types()
    types = {t.name: t.type for t in topics}
    has_hdr = {t.name: header_first(t.type) for t in topics}
    wr = None
    if not report_only:
        wr = rosbag2_py.SequentialWriter()
        wr.open(rosbag2_py.StorageOptions(uri=out, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
        for t in topics:
            wr.create_topic(t)
    stats = {t.name: {'lag': [], 'log': [], 'stamp': [], 'kept': 0} for t in topics}
    heap, seq, last_out, clamped = [], 0, -1, 0
    shift_ns = int(max_shift * 1e9)

    def emit(upto):
        nonlocal last_out, clamped
        while heap and heap[0][0] <= upto:
            t, _, topic, data = heapq.heappop(heap)
            if t < last_out:            # max_shift 안에서는 일어나지 않음
                t, clamped = last_out, clamped + 1
            last_out = t
            wr.write(topic, data, t)

    while rd.has_next():
        topic, data, log_ns = rd.read_next()
        st = stats[topic]
        st['log'].append(log_ns)
        s = stamp_ns(topic, types[topic], data, has_hdr[topic])
        new = log_ns
        if s is not None and s > 0 and abs(log_ns - s) <= shift_ns:
            new = s
            st['lag'].append((log_ns - s) * 1e-6)
            st['stamp'].append(s)
        else:
            st['kept'] += 1
        if wr is not None:
            heapq.heappush(heap, (new, seq, topic, data))
            seq += 1
            # 앞으로 올 메시지의 새 시각은 (지금 기록 시각 - max_shift) 이상이므로 그 이전 것은 내보내도 순서가 맞다
            emit(log_ns - shift_ns)
    if wr is not None:
        emit(float('inf'))
        wr.close()                      # metadata.yaml 기록

    print(f'{os.path.basename(path.rstrip("/"))}' + ('' if report_only else f' -> {out}'))
    print(f'  {"토픽":<28}{"수":>6}  {"기록 지연 ms (중앙/p90/최대)":<24}{"최대 공백 ms (기록/header)":<22}그대로')
    for name in sorted(stats):
        st = stats[name]
        n = len(st['log'])
        if n == 0:
            continue
        lag = np.array(st['lag'])
        lag_s = f'{np.median(lag):6.0f} {np.percentile(lag, 90):6.0f} {lag.max():6.0f}' if len(lag) else '     -'
        gap = np.diff(np.array(st['log'])) * 1e-6
        hgap = np.diff(np.sort(np.array(st['stamp']))) * 1e-6
        gap_s = (f'{gap.max():6.0f} ' if len(gap) else '     - ') + (f'{hgap.max():6.0f}' if len(hgap) else '     -')
        print(f'  {name:<28}{n:>6}  {lag_s:<28}{gap_s:<20}{st["kept"]}')
    if clamped:
        print(f'  순서 보정 {clamped} 건')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--suffix', default='_hdr', help='출력 폴더 = 입력 폴더 + suffix')
    ap.add_argument('--max-shift', type=float, default=5.0, help='header 와 기록 시각 차가 이보다 크면 기록 시각 유지 (s)')
    ap.add_argument('--report', action='store_true', help='쓰지 않고 토픽별 기록 지연만 출력')
    a = ap.parse_args()
    for b in a.bags:
        b = os.path.expanduser(b).rstrip('/')
        out = b + a.suffix
        if not a.report and os.path.exists(out):
            print(f'{out} 이 이미 있어 건너뜀 (지우지 않음)')
            continue
        retime(b, out, a.max_shift, a.report)


if __name__ == '__main__':
    main()
