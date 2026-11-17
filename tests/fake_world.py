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
camera_on = True                 # False = the camera is hidden
turn = 0                         # number of read() so far
events = {}                      # turn number -> function called at that turn
track = []                       # every (x, y, left, right) of the run
backing = 0                      # turns the fake avoidance still backs away


def reset():
    """Put the world back to the shared example."""
    global pose, wheels, grid, goal, obstacle, camera_on, turn, events, track
    global backing
    random.seed(1)
    pose = common.EXAMPLE_POSE
    wheels = (0, 0)
    grid = common.EXAMPLE_GRID.copy()
    goal = common.EXAMPLE_GOAL
    obstacle = None
    camera_on = True
    turn = 0
    events = {}
    track = []
    backing = 0


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

    # The front centre sensor sees the added obstacle (a 15 cm box in the
    # middle of its cell) when the front of the robot, 5.5 cm ahead of the
    # wheel axis, is within 3 cm of it, roughly in front.
    prox = [0, 0, 0, 0, 0, 0, 0]
    if obstacle is not None:
        ox, oy = common.cell_to_xy(obstacle[0], obstacle[1])
        distance = math.hypot(ox - pose[0], oy - pose[1])
        angle = wrap(math.atan2(oy - pose[1], ox - pose[0]) - pose[2])
        if distance < 0.055 + 0.03 + 0.075 and abs(angle) < math.radians(35):
            prox[2] = 4000   # what the real sensor gives at 1-2 cm
    return prox, list(common.EXAMPLE_GROUND), wheels


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

def avoid(prox, ground):
    """Back away while something is in front, and for one more second."""
    global backing
    if max(prox[:5]) > 0:
        backing = 10
    if backing > 0:
        backing -= 1
        return -100, -100
    return None
