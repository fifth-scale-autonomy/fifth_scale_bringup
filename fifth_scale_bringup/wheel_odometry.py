"""바퀴 속도·조향 피드백으로 차량 이동량을 적분하는 주행 거리계 (자전거 모델, 뒤축 중심 = base_link).

입력: /vehicle/speed_mps      (Float32, m/s, 후진 음수. scale_vehicle_node 가 rpm 피드백과 바퀴 지름으로 계산)
      /vehicle/steering_angle (Float32, rad. 제어기 조향 피드백. 부호는 제어기 기준이라 steer_invert 로 뒤집는다)
출력: /odom                   (nav_msgs/Odometry, odom -> base_link)
      TF odom -> base_link

피드백이 timeout 넘게 끊기면 적분과 TF 발행을 멈춘다. 그동안 cone_fusion 은 이동 보정 없이 동작한다.
오차 요인 (실측 전): 바퀴 지름 0.27 m, rpm 배율 100, 조향 피드백 ±2000 = ±20도 가정, 슬립.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


def integrate(x, y, yaw, v, steer, dt, wheelbase):
    """뒤축 중심 자전거 모델을 dt 동안 정확히(원호로) 적분한다. steer 는 ROS 기준 (좌 +)."""
    w = v * math.tan(steer) / wheelbase
    if abs(w) < 1e-9:
        return x + v * dt * math.cos(yaw), y + v * dt * math.sin(yaw), yaw
    r = v / w
    yaw2 = yaw + w * dt
    return (x + r * (math.sin(yaw2) - math.sin(yaw)),
            y - r * (math.cos(yaw2) - math.cos(yaw)),
            math.atan2(math.sin(yaw2), math.cos(yaw2)))


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        p = self.declare_parameter
        p('wheelbase', 0.75)
        p('speed_topic', '/vehicle/speed_mps')
        p('steer_topic', '/vehicle/steering_angle')
        p('steer_invert', True)       # drive_command invert=True 와 같은 이유: 제어기 + 가 ROS 우회전
        p('steer_offset', 0.0)        # ROS 기준 조향 영점 보정 (rad)
        # 조향 피드백 배율 (1.0 = 피드백 그대로). 2026-10-14 띠 트랙 bag 3 개에서 1 s 전 띠를 odom 으로 옮겨 지금 띠와
        # 맞춰 보니 회전량을 0.7 배로 할 때 회전 중 오차가 15 cm -> 3.5 cm 로 가장 작았다 (실제 바퀴 각 측정 전).
        p('steer_scale', 1.0)
        p('rate', 50.0)
        p('timeout', 0.3)             # 속도 피드백이 이만큼 끊기면 적분·발행 중지 (s)
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('publish_tf', True)
        self.g = lambda n: self.get_parameter(n).value

        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.steer = 0.0
        self.t_speed = None           # 마지막 속도 피드백 시각 (s)
        self.t_last = None            # 마지막 적분 시각 (s)
        self.stale = True

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Float32, self.g('speed_topic'), self.on_speed, 20)
        self.create_subscription(Float32, self.g('steer_topic'), self.on_steer, 20)
        self.create_timer(1.0 / self.g('rate'), self.step)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_speed(self, msg):
        self.v = float(msg.data)
        self.t_speed = self.now()

    def on_steer(self, msg):
        s = -float(msg.data) if self.g('steer_invert') else float(msg.data)
        self.steer = self.g('steer_scale') * s + self.g('steer_offset')

    def step(self):
        t = self.now()
        fresh = self.t_speed is not None and t - self.t_speed <= self.g('timeout')
        if not fresh:
            if not self.stale:
                self.get_logger().warn('속도 피드백 끊김 -> 주행 거리계 정지 (cone_fusion 이동 보정 꺼짐)')
            self.stale, self.t_last = True, None
            return
        if self.stale:
            self.get_logger().info('속도 피드백 수신 -> 주행 거리계 동작')
            self.stale = False
        if self.t_last is not None:
            dt = min(t - self.t_last, 0.1)
            self.x, self.y, self.yaw = integrate(self.x, self.y, self.yaw, self.v, self.steer, dt,
                                                 self.g('wheelbase'))
        self.t_last = t
        self.publish(t)

    def publish(self, t):
        stamp = Time(seconds=t).to_msg()
        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.g('odom_frame')
        o.child_frame_id = self.g('base_frame')
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.v * math.tan(self.steer) / self.g('wheelbase')
        self.pub.publish(o)
        if self.g('publish_tf'):
            tf = TransformStamped()
            tf.header = o.header
            tf.child_frame_id = o.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
