"""RViz 에 차량 모양을 그린다 (base_link 기준: 원점 = 뒤축 중심 아래 지면, x 전방).

출력: /vehicle/markers (MarkerArray)
  - 차체 바닥 윤곽(흰 반투명 상자 + 테두리, 앞쪽 끝은 주황), 네 바퀴(앞바퀴는 실제 조향각만큼 돌아감)
  - 라이다(x 0.95, 높이 0.48)와 카메라(x 0.53, 높이 0.92) 위치
  - 차 위 글자: 속도(m/s)와 조향각
조향각은 차량 피드백(/vehicle/steering_angle, 제어기 부호라 steer_invert 로 뒤집음)을 쓰고,
없으면 경로 조향 명령(/planning/steering_angle)을 쓴다.
차체 앞뒤 끝(body_front, body_rear)은 실측 전 값이다. 재서 바꾼다.
"""
import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float32
from visualization_msgs.msg import Marker, MarkerArray


class VehicleMarker(Node):
    def __init__(self):
        super().__init__('vehicle_marker')
        p = self.declare_parameter
        p('frame', 'base_link')
        p('wheelbase', 0.75)
        p('width', 0.80)                  # 차폭 (바퀴 바깥면까지)
        p('body_front', 1.05)             # 뒤축에서 차체 앞끝까지 (m, 실측 전. 라이다가 0.95 에 있음)
        p('body_rear', -0.25)             # 뒤축에서 차체 뒤끝까지 (m, 실측 전)
        p('wheel_diameter', 0.27)
        p('wheel_width', 0.10)
        p('steer_invert', True)           # 제어기 피드백 + 가 ROS 우회전 (drive_command invert 와 같은 이유)
        p('rate', 10.0)
        self.g = lambda n: self.get_parameter(n).value
        self.steer_fb = self.steer_cmd = self.speed = None
        self.t_fb = self.t_cmd = self.t_speed = None
        self.create_subscription(Float32, '/vehicle/steering_angle', self.on_fb, 10)
        self.create_subscription(Float32, '/planning/steering_angle', self.on_cmd, 10)
        self.create_subscription(Float32, '/vehicle/speed_mps', self.on_speed, 10)
        self.pub = self.create_publisher(MarkerArray, '/vehicle/markers', 2)
        self.create_timer(1.0 / self.g('rate'), self.publish)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_fb(self, m):
        self.steer_fb, self.t_fb = (-m.data if self.g('steer_invert') else m.data), self.now()

    def on_cmd(self, m):
        self.steer_cmd, self.t_cmd = m.data, self.now()

    def on_speed(self, m):
        self.speed, self.t_speed = m.data, self.now()

    def fresh(self, t):
        return t is not None and abs(self.now() - t) < 1.0

    def publish(self):
        L, W = self.g('wheelbase'), self.g('width')
        xf, xr = self.g('body_front'), self.g('body_rear')
        wd, ww = self.g('wheel_diameter'), self.g('wheel_width')
        steer, src = 0.0, ''
        if self.fresh(self.t_fb):
            steer, src = self.steer_fb, 'fb'
        elif self.fresh(self.t_cmd):
            steer, src = self.steer_cmd, 'cmd'
        arr = MarkerArray()
        mid = [0]

        def mk(mtype, x, y, z, sx, sy, sz, rgba, yaw=0.0, roll=0.0):
            m = Marker()
            m.header.frame_id = self.g('frame')          # stamp 0: RViz 가 최신 TF 로 그림 (실차·재생 모두)
            m.ns, m.id, m.type, m.action = 'vehicle', mid[0], mtype, Marker.ADD
            mid[0] += 1
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
            cr, sr, cy, sy = math.cos(roll / 2), math.sin(roll / 2), math.cos(yaw / 2), math.sin(yaw / 2)
            m.pose.orientation.w, m.pose.orientation.x = cr * cy, sr * cy        # yaw(z) * roll(x)
            m.pose.orientation.y, m.pose.orientation.z = sr * sy, cr * sy
            m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
            m.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
            arr.markers.append(m)
            return m

        # 차체: 반투명 상자 (지면 위 0.12~0.45 m) + 바닥 윤곽선
        mk(Marker.CUBE, (xf + xr) / 2, 0, 0.29, xf - xr, W - 0.04, 0.33, (0.9, 0.9, 0.95, 0.35))
        out = mk(Marker.LINE_STRIP, 0, 0, 0.02, 0.04, 0, 0, (1.0, 1.0, 1.0, 0.95))
        out.points = [Point(x=float(x), y=float(y)) for x, y in
                      ((xr, W / 2), (xf, W / 2), (xf, -W / 2), (xr, -W / 2), (xr, W / 2))]
        mk(Marker.CUBE, xf - 0.03, 0, 0.29, 0.06, W - 0.04, 0.34, (1.0, 0.55, 0.0, 0.9))   # 앞쪽 끝 (주황)
        # 바퀴: 원기둥 축을 y 로 (x 축 90도), 앞바퀴는 조향각만큼 돌림
        yw = W / 2 - ww / 2
        for x, y, yaw in ((0, yw, 0), (0, -yw, 0), (L, yw, steer), (L, -yw, steer)):
            mk(Marker.CYLINDER, x, y, wd / 2, wd, wd, ww, (0.1, 0.1, 0.1, 1.0), yaw=yaw, roll=math.pi / 2)
        # 뒤축 중심 (base_link 원점, 경로가 지나는 점)
        mk(Marker.SPHERE, 0, 0, 0.05, 0.12, 0.12, 0.12, (1.0, 0.2, 0.2, 1.0))
        # 센서
        mk(Marker.CYLINDER, 0.95, 0, 0.48, 0.10, 0.10, 0.06, (0.2, 0.6, 1.0, 1.0))      # 라이다
        mk(Marker.CUBE, 0.53, 0, 0.92, 0.04, 0.12, 0.04, (0.2, 1.0, 0.4, 1.0))         # 카메라
        txt = mk(Marker.TEXT_VIEW_FACING, (xf + xr) / 2, 0, 0.9, 0, 0, 0.18, (1, 1, 1, 1))
        v = f'{self.speed:+.2f} m/s' if self.fresh(self.t_speed) else 'v -'
        st = f'steer {math.degrees(steer):+.1f} deg ({src})' if src else 'steer -'
        txt.text = f'{v}  {st}'
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = VehicleMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
