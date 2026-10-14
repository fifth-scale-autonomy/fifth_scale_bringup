"""실행 중인 ROS 노드의 CPU 사용량을 /proc 에서 직접 재서 노드별·스레드별로 나눈다.

    python3 -m fifth_scale_bringup.cpu_profile --seconds 30 --threads cone_detector

CPU% 는 코어 1개를 100% 로 본 값이다 (4코어 기계의 최대는 400%).
"""
import argparse
import os
import re
import time

HZ = os.sysconf('SC_CLK_TCK')
NCPU = os.cpu_count()

# (표시 이름, 명령줄 정규식). 위에서부터 처음 맞는 것으로 분류한다
GROUPS = [
    ('camera (v4l2_camera)', r'v4l2_camera_node|usb_cam_node_exe'),
    ('lidar (sllidar_node)', r'sllidar_node'),
    ('scan_filter', r'scan_to_scan_filter_chain'),
    ('static TF x3', r'static_transform_publisher'),
    ('cone_detector', r'lavacone_fusion/cone_detector'),
    ('cone_fusion', r'lavacone_fusion/cone_fusion'),
    ('lidar_projection', r'lavacone_fusion/lidar_projection'),
    ('rviz2', r'rviz2'),
    ('ros2 launch / daemon', r'ros2 launch|ros2cli.daemon'),
]


def proc_ticks(pid):
    with open(f'/proc/{pid}/stat') as f:
        parts = f.read().rsplit(')', 1)[1].split()
    return int(parts[11]) + int(parts[12])      # utime + stime


def cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return f.read().replace(b'\0', b' ').decode(errors='replace')
    except OSError:
        return ''


def thread_ticks(pid):
    out = {}
    for tid in os.listdir(f'/proc/{pid}/task'):
        try:
            with open(f'/proc/{pid}/task/{tid}/stat') as f:
                s = f.read()
            name = s[s.index('(') + 1:s.rindex(')')]
            parts = s.rsplit(')', 1)[1].split()
            out[tid] = (name, int(parts[11]) + int(parts[12]))
        except (OSError, ValueError):
            pass
    return out


def system_ticks():
    with open('/proc/stat') as f:
        v = [int(x) for x in f.readline().split()[1:]]
    idle = v[3] + v[4]
    return sum(v), idle


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--threads', default='cone_detector', help='스레드별로 나눠 볼 그룹 이름 (빈 값이면 생략)')
    args = ap.parse_args()

    me = os.getpid()
    pids = {}
    for p in os.listdir('/proc'):
        if p.isdigit() and int(p) != me:
            cl = cmdline(p)
            for name, pat in GROUPS:
                if re.search(pat, cl):
                    pids[p] = name
                    break
    other_pids = [p for p in os.listdir('/proc') if p.isdigit() and p not in pids and int(p) != me]

    def snap():
        a = {}
        for p in list(pids) + other_pids:
            try:
                a[p] = proc_ticks(p)
            except OSError:
                pass
        return a

    tpid = next((p for p, n in pids.items() if n == args.threads), None)
    s0, t0, th0, (tot0, idle0) = snap(), time.monotonic(), thread_ticks(tpid) if tpid else {}, system_ticks()
    time.sleep(args.seconds)
    s1, t1, th1, (tot1, idle1) = snap(), time.monotonic(), thread_ticks(tpid) if tpid else {}, system_ticks()
    dt = t1 - t0
    pct = lambda d: 100.0 * d / HZ / dt  # noqa: E731

    groups = {}
    for p, name in pids.items():
        if p in s0 and p in s1:
            groups[name] = groups.get(name, 0) + pct(s1[p] - s0[p])
    others = {}
    for p in other_pids:
        if p in s0 and p in s1 and s1[p] > s0[p]:
            comm = cmdline(p).split(' ')[0].rsplit('/', 1)[-1][:24] or p
            others[comm] = others.get(comm, 0) + pct(s1[p] - s0[p])

    busy = 100.0 * (1 - (idle1 - idle0) / max(tot1 - tot0, 1))
    print(f'측정 {dt:.0f}s, 시스템 전체 CPU 사용률 {busy:.1f}% (코어 {NCPU}개 = {busy * NCPU:.0f}%/400%)')
    print('\n[ROS 노드별] CPU% (코어 1개 = 100%)')
    total_ros = 0.0
    for name, v in sorted(groups.items(), key=lambda kv: -kv[1]):
        total_ros += v
        print(f'  {name:26s} {v:6.1f}%  {"#" * int(v / 5)}')
    print(f'  {"(ROS 합계)":26s} {total_ros:6.1f}%')
    print('\n[그 밖의 프로세스] 상위 5개')
    for name, v in sorted(others.items(), key=lambda kv: -kv[1])[:5]:
        print(f'  {name:26s} {v:6.1f}%')
    if tpid:
        print(f'\n[{args.threads} 스레드별] CPU%')
        rows = {}
        for tid, (name, ticks) in th1.items():
            if tid in th0:
                rows.setdefault(name, []).append(pct(ticks - th0[tid][1]))
        for name, vals in sorted(rows.items(), key=lambda kv: -sum(kv[1])):
            if sum(vals) >= 0.5:
                print(f'  {name:20s} x{len(vals):<3d} 합계 {sum(vals):6.1f}%  (각 {", ".join(f"{v:.0f}" for v in sorted(vals, reverse=True)[:6])})')


if __name__ == '__main__':
    main()
