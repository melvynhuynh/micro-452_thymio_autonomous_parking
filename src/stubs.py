"""Fake version of every block. Belongs to all four of us.

Each function has the same name, the same inputs and the same outputs as the
real one, and gives back the shared example of common.py.

To test your block without the camera or the robot, swap one import:

    from src import stubs as vision     # instead of: from src import vision
"""

import numpy as np

from src import common


# --- vision ------------------------------------------------------------------

def read_frame():
    """Fake camera image: a black picture."""
    return np.zeros((480, 640, 3), dtype=np.uint8)


def detect(frame):
    """Fake detection: the grid, the robot and the target of the example."""
    if frame is None:
        return None, None, None
    return common.EXAMPLE_GRID.copy(), common.EXAMPLE_POSE, common.EXAMPLE_GOAL


def to_pixels(x, y):
    """Fake conversion: as if the lot filled the whole 640 x 480 image."""
    u = round(x / (common.COLS * common.CELL) * 640)
    v = round(480 - y / (common.ROWS * common.CELL) * 480)
    return u, v


# --- localization ------------------------------------------------------------

def update(cam_pose, wheel_speeds, dt):
    """Fake filter: trusts the camera, no uncertainty."""
    return cam_pose, None


# --- global_nav --------------------------------------------------------------

def plan(grid, pose, goal):
    """Fake planner: the path of the example."""
    return list(common.EXAMPLE_PATH)


# --- motion ------------------------------------------------------------------

def go_to(waypoint, pose):
    """Fake controller: does not move and says the waypoint is reached."""
    return 0, 0, True


# --- robot -------------------------------------------------------------------

def connect():
    """Fake connection: nothing to do."""


def read():
    """Fake sensors: nothing around, white paper below, robot not moving."""
    return (list(common.EXAMPLE_PROX), list(common.EXAMPLE_GROUND),
            common.EXAMPLE_WHEEL_SPEEDS)


def set_speed(left, right):
    """Fake motors: nothing happens."""


# --- local_nav ---------------------------------------------------------------

def avoid(prox, ground):
    """Fake avoidance: nothing is ever in the way."""
    return None


# --- visualization -----------------------------------------------------------

def show(frame, grid, goal, path, cam_pose, pose, cov):
    """Fake display: nothing is drawn."""
