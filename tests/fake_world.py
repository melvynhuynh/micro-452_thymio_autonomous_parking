"""A fake robot and a fake camera that agree with each other. For the tests.

The stubs always give back the same pose, so nothing ever moves. Here the
wheel speeds really move a simulated robot, and the fake camera sees it where
it is. It replaces robot.py and vision.py in the test of main.py.

It also has a small planner and a small obstacle avoidance, only to test the
states of main.py before the real blocks of Melvyn and Laurent exist.
"""

import math
import random

import numpy as np

from src import common
from src.motion import wrap
from tests.odometry import move

DT = 0.1                         # the world moves by this much at each read()
NOISE_XY = 0.002                 # error of the fake camera on the position, in m
NOISE_THETA = math.radians(1)    # error of the fake camera on the heading

# The state of the fake world. reset() puts it back to the shared example.
pose = common.EXAMPLE_POSE       # true pose of the fake robot
wheels = (0, 0)                  # wheel speeds asked last
grid = common.EXAMPLE_GRID.copy()
goal = common.EXAMPLE_GOAL
obstacle = None                  # cell (row, col) of an added obstacle, or None
obstacle_at = None               # or (x, y) of its centre, anywhere, in m
camera_on = True                 # False = the camera is hidden
turn = 0                         # number of read() so far
events = {}                      # turn number -> function called at that turn
track = []                       # every (x, y, left, right) of the run
passing = 0                      # turns the fake avoidance still drives on
side = 1                         # side the fake avoidance turns to: +1 left


def reset():
    """Put the world back to the shared example."""
    global pose, wheels, grid, goal, obstacle, obstacle_at, camera_on, turn
    global events, track, passing, side
    random.seed(1)
    pose = common.EXAMPLE_POSE
    wheels = (0, 0)
    grid = common.EXAMPLE_GRID.copy()
    goal = common.EXAMPLE_GOAL
    obstacle = None
    obstacle_at = None
    camera_on = True
    turn = 0
    events = {}
    track = []
    passing = 0
    side = 1


# --- fake robot.py -----------------------------------------------------------

def connect():
    """Nothing to connect."""


def read():
    """Move the world by one loop turn, then give the sensors of the robot."""
    global pose, turn
    turn += 1
    if turn in events:
        events[turn]()
    pose = move(pose, wheels[0], wheels[1], DT)
    track.append((pose[0], pose[1], wheels[0], wheels[1]))

    # Each of the 5 front sensors sits on the round front of the robot,
    # 5.5 cm from the wheel axis, and looks outwards (front left first). Its
    # beam, 20 degrees wide, measures the distance to the nearest box.
    prox = [0, 0, 0, 0, 0, 0, 0]
    for sensor, angle in enumerate(SENSOR_ANGLES):
        nearest = math.inf
        for spread in (-10, 0, 10):
            direction = pose[2] + math.radians(angle + spread)
            sx = pose[0] + ROBOT_FRONT * math.cos(direction)
            sy = pose[1] + ROBOT_FRONT * math.sin(direction)
            for centre in box_centres():
                nearest = min(nearest, ray_to_box(sx, sy, direction, centre))
        prox[sensor] = reading(nearest)
    return prox, list(common.EXAMPLE_GROUND), wheels


SENSOR_ANGLES = [40, 20, 0, -20, -40]   # degrees, left positive (approximate)
ROBOT_FRONT = 0.055                     # m from the wheel axis to the front
BOX_HALF = 0.075                        # half the side of a box, in m


def reading(distance):
    """Sensor value for a box at distance (m), fitted on our recordings:
    about 4800 touching, 3000 at 3 cm, 1200 at 6 cm, 0 beyond 8 cm."""
    if distance > 0.08:
        return 0
    return round(4800 - 60000 * max(distance, 0.0))


def ray_to_box(x, y, direction, centre):
    """Distance along a ray from (x, y) to a square box, or inf if it misses."""
    dx, dy = math.cos(direction), math.sin(direction)
    enter, leave = 0.0, math.inf
    for start, step, low, high in [
            (x, dx, centre[0] - BOX_HALF, centre[0] + BOX_HALF),
            (y, dy, centre[1] - BOX_HALF, centre[1] + BOX_HALF)]:
        if abs(step) < 1e-9:
            if not low <= start <= high:
                return math.inf
            continue
        t1, t2 = (low - start) / step, (high - start) / step
        enter = max(enter, min(t1, t2))
        leave = min(leave, max(t1, t2))
    return enter if enter <= leave else math.inf


def box_centres():
    """(x, y) of every box: the cars of the grid and the added obstacle."""
    centres = [common.cell_to_xy(row, col) for row in range(common.ROWS)
               for col in range(common.COLS) if grid[row, col] == 1]
    if obstacle is not None:
        centres.append(common.cell_to_xy(obstacle[0], obstacle[1]))
    if obstacle_at is not None:
        centres.append(obstacle_at)
    return centres


def set_speed(left, right):
    """Remember the wheel speeds; read() uses them to move the robot."""
    global wheels
    wheels = (left, right)


# --- fake vision.py ----------------------------------------------------------

def read_frame():
    """A tiny black image, or None when the camera is hidden."""
    if not camera_on:
        return None
    return np.zeros((2, 2, 3), dtype=np.uint8)


def detect(frame):
    """The grid, the pose of the fake robot (a bit noisy) and the target."""
    if frame is None:
        return None, None, None
    cam_pose = (pose[0] + random.gauss(0, NOISE_XY),
                pose[1] + random.gauss(0, NOISE_XY),
                wrap(pose[2] + random.gauss(0, NOISE_THETA)))
    # Like the real camera, the target is not seen when the robot is on it.
    seen_goal = goal
    if math.dist(pose[:2], goal) < common.CELL / 4:
        seen_goal = None
    return grid.copy(), cam_pose, seen_goal


# --- fake global_nav.py ------------------------------------------------------

def plan(grid, pose, goal):
    """Shortest path on the grid, four neighbours. Stand-in for the real one."""
    start = common.xy_to_cell(pose[0], pose[1])
    end = common.xy_to_cell(goal[0], goal[1])

    def free(cell):
        row, col = cell
        return (0 <= row < common.ROWS and 0 <= col < common.COLS
                and grid[row, col] == 0)

    if not free(start) or not free(end):
        return []
    came_from = {start: None}
    to_visit = [start]
    while to_visit:
        cell = to_visit.pop(0)
        if cell == end:
            break
        row, col = cell
        for neighbour in [(row - 1, col), (row + 1, col), (row, col - 1),
                          (row, col + 1)]:
            if free(neighbour) and neighbour not in came_from:
                came_from[neighbour] = cell
                to_visit.append(neighbour)
    if end not in came_from:
        return []

    cells = []
    cell = end
    while cell != start:
        cells.append(cell)
        cell = came_from[cell]
    cells.reverse()
    path = [common.cell_to_xy(row, col) for row, col in cells]
    return path[:-1] + [goal]


# --- fake local_nav.py -------------------------------------------------------

FOLLOW_THRESHOLD = 3000    # same names as in local_nav.py
PARKING_THRESHOLD = 4600


def room_on_side(side):
    """True if 20 cm to that side of the robot (+1 left, -1 right) is free."""
    angle = pose[2] + side * math.pi / 2
    x = pose[0] + common.CELL * math.cos(angle)
    y = pose[1] + common.CELL * math.sin(angle)
    if not (0 <= x < common.COLS * common.CELL
            and 0 <= y < common.ROWS * common.CELL):
        return False
    return grid[common.xy_to_cell(x, y)] == 0


def avoid(prox, ground, threshold):
    """Drive around an obstacle: turn in place towards the free side while a
    front sensor sees it, then straight on for 3 s to get past it, then give
    the wheels back."""
    global passing, side
    if max(prox[:5]) > 0:
        if passing == 0:   # start of an avoidance: choose the side once
            side = 1 if room_on_side(1) else -1
        passing = 30
        return -80 * side, 80 * side
    if passing > 0:
        passing -= 1
        return 120, 120
    return None
