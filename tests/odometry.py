"""Movement equations of a two-wheeled robot. Used by the motion tests.

Given a pose and the speeds of the two wheels, where is the robot a short
time later? The simulation uses it to move a fake robot, and the test on the
real Thymio uses it to follow the pose when there is no camera.
"""

import math

from src import common
from src.motion import wrap


def move(pose, left, right, dt):
    """Pose after driving dt seconds at wheel speeds left, right (Thymio units).

    pose  (x, y, theta), in metres and radians.

    Gives back the new (x, y, theta).
    """
    x, y, theta = pose
    forward = (left + right) / 2 * common.THYMIO_TO_MS               # m/s
    turn = (right - left) * common.THYMIO_TO_MS / common.WHEEL_BASE  # rad/s

    # The robot moves along its heading at the middle of the time step.
    middle = theta + turn * dt / 2
    x = x + forward * math.cos(middle) * dt
    y = y + forward * math.sin(middle) * dt
    theta = wrap(theta + turn * dt)
    return x, y, theta
