"""Motion control block. Owner: Antoine.

Drive the robot towards one waypoint.

The idea, in three points:
    1. Heading error = angle between where the robot faces and where the
       waypoint is, brought back between -pi and pi.
    2. The robot turns at a rate proportional to this error.
    3. It drives forward at a constant cruise speed, multiplied by the cosine
       of the error: full speed when it faces the waypoint, slower as the
       error grows, and no forward speed at all beyond 90 degrees (it then
       turns in place).
    4. Very close to the waypoint, the direction to it is not reliable any
       more (a position error of 3 mm seen from 2 cm away is a 9 degree
       error), so the robot steers more gently there.

Nothing switches on or off at a threshold, so the wheel speeds change smoothly
with the heading error. The forward speed does not depend on the distance, so
the robot does not brake at every waypoint.
"""

import math

from src import common

# Provisional values, to tune on the real robot (keep the recordings in data/).
CRUISE_SPEED = 0.045  # forward speed when facing the waypoint, in m/s (about
                      # 150 Thymio units; at 200 the wheels lag the speed asked)
TURN_GAIN = 2.0       # turn rate per radian of heading error, in 1/s
MAX_TURN = 1.5        # largest turn rate, in rad/s
CLOSE = 0.06          # closer than this to the waypoint, steer more gently, in m
MAX_WHEEL = 500       # largest speed a wheel accepts, in Thymio units
MAX_SPEED_UP = 30     # largest rise of the forward speed per loop turn (0.1 s),
                      # in Thymio units: 0 to cruise speed in 0.5 s


def wrap(angle):
    """Bring an angle back between -pi (excluded) and pi (included)."""
    return math.pi - (math.pi - angle) % (2 * math.pi)


def go_to(waypoint, pose):
    """Wheel speeds that bring the robot towards one waypoint.

    waypoint  (x, y) to reach, in metres.
    pose      (x, y, theta) of the robot, in metres and radians.

    Gives back three values:
        left     speed of the left wheel, in Thymio units.
        right    speed of the right wheel, in Thymio units.
        reached  True when the robot is within common.WAYPOINT_TOL of the
                 waypoint.
    """
    if pose is None:
        return 0, 0, False   # unknown position: stop

    x, y, theta = pose
    dx = waypoint[0] - x
    dy = waypoint[1] - y
    distance = math.hypot(dx, dy)
    reached = distance < common.WAYPOINT_TOL

    # 1. Heading error: positive when the waypoint is on the left.
    error = wrap(math.atan2(dy, dx) - theta)

    # 2. Turn rate, in rad/s, limited to MAX_TURN.
    # 4. It is reduced in proportion to the distance once closer than CLOSE.
    turn = TURN_GAIN * error * min(1.0, distance / CLOSE)
    turn = max(-MAX_TURN, min(MAX_TURN, turn))

    # 3. Forward speed, in m/s.
    forward = CRUISE_SPEED * max(0.0, math.cos(error))

    # From (forward, turn) to the two wheels. To turn left, the right wheel
    # goes faster than the left one.
    half_base = common.WHEEL_BASE / 2
    left = (forward - turn * half_base) / common.THYMIO_TO_MS
    right = (forward + turn * half_base) / common.THYMIO_TO_MS

    left = max(-MAX_WHEEL, min(MAX_WHEEL, round(left)))
    right = max(-MAX_WHEEL, min(MAX_WHEEL, round(right)))
    return left, right, reached


def soften(previous, target):
    """Soft start: the target wheel speeds, with the forward speed rising slowly.

    previous  (left, right) sent to the wheels at the last loop turn.
    target    (left, right) wanted now, for example given by go_to.

    The forward speed (mean of the two wheels) rises by at most MAX_SPEED_UP
    per turn, but drops at once. The turning part (half the difference of the
    two wheels) is never limited. Why: a jump from 0 to 150 makes the wheels
    slip and the Thymio misread their speed (measured on 2026-11-17: the right
    wheel reads up to 30 units too fast for 0.5 s), so the pose computed from
    the wheel speeds turns left when the robot does not. Limiting the turning
    or the braking too makes the robot overshoot each corner (simulation:
    4.5 cm from the path instead of 1.6 cm).
    """
    forward_before = (previous[0] + previous[1]) / 2
    forward = (target[0] + target[1]) / 2
    turning = (target[1] - target[0]) / 2
    forward = min(forward, forward_before + MAX_SPEED_UP)
    return round(forward - turning), round(forward + turning)
