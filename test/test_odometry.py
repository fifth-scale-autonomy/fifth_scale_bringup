"""주행 거리계 적분 단위 시험."""
import math

from fifth_scale_bringup.wheel_odometry import integrate


def run(v, steer, seconds, dt=0.02, wheelbase=0.75):
    x = y = yaw = 0.0
    for _ in range(int(round(seconds / dt))):
        x, y, yaw = integrate(x, y, yaw, v, steer, dt, wheelbase)
    return x, y, yaw


def test_straight():
    x, y, yaw = run(1.0, 0.0, 2.0)
    assert math.isclose(x, 2.0, abs_tol=1e-9) and abs(y) < 1e-9 and abs(yaw) < 1e-9


def test_left_turn_follows_circle():
    steer = math.radians(15)
    r = 0.75 / math.tan(steer)                       # 약 2.8 m
    t = (math.pi / 2) * r / 1.0                      # 1/4 바퀴
    x, y, yaw = run(1.0, steer, t, dt=t / 200)
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-6)
    assert math.isclose(x, r, abs_tol=1e-6) and math.isclose(y, r, abs_tol=1e-6)


def test_right_turn_is_negative_and_reverse_goes_back():
    _, y, yaw = run(1.0, math.radians(-10), 1.0)
    assert y < 0 and yaw < 0
    x, _, _ = run(-0.5, 0.0, 1.0)
    assert math.isclose(x, -0.5, abs_tol=1e-9)


def test_step_size_does_not_matter():
    a = run(2.0, math.radians(12), 1.5, dt=0.02)
    b = run(2.0, math.radians(12), 1.5, dt=0.1)
    assert all(math.isclose(p, q, abs_tol=1e-9) for p, q in zip(a, b))
