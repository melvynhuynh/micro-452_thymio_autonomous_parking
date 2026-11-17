"""Simulation of go_to: a fake two-wheeled robot follows the shared example path.

The wheel speeds given by go_to are fed into the movement equations
(tests/odometry.py), which give the next pose, which goes back into go_to.

Two runs are drawn:
    perfect    the pose is exact and the wheels obey instantly;
    realistic  noisy and late pose (like a camera), wheels that take time to
               reach their speed, one wheel a bit faster than the other.

No robot and no camera needed. Start it from the root of the repository:

    python -m tests.sim_go_to

The figure is also saved as data/sim_go_to.png.
"""

import math
import random

import matplotlib.pyplot as plt

from src import common
from src.motion import go_to, soften
from tests.odometry import move

DT = 0.1            # one turn of the main loop, in seconds
SUB_STEPS = 10      # the fake robot moves in smaller steps than the loop

# What makes the realistic run imperfect (guesses, to replace by measurements).
NOISE_XY = 0.003                 # error of the seen position, in metres
NOISE_THETA = math.radians(2)    # error of the seen heading, in radians
LATE_STEPS = 2                   # the seen pose is this many loop turns old
MOTOR_TIME = 0.15                # time the wheels take to reach their speed, in s
RIGHT_WHEEL_FACTOR = 1.05        # the right wheel turns 5 % too fast


def simulate(path, start, realistic, seed=1):
    """Follow the path from the start pose. Gives back the log of the run.

    Each line of the log is (time, x, y, theta, left, right): the true pose
    of the fake robot and the wheel speeds asked by go_to.
    """
    random.seed(seed)              # same noise at every run
    pose = start
    wheels = (0.0, 0.0)            # true speeds of the wheels
    old_poses = [start] * (LATE_STEPS + 1)
    target = 0                     # index of the waypoint the robot drives to
    asked = (0, 0)                 # wheel speeds asked at the last turn
    log = []

    for turn in range(3000):
        # What the controller is told about the pose.
        seen = pose
        if realistic:
            x, y, theta = old_poses[0]
            seen = (x + random.gauss(0, NOISE_XY), y + random.gauss(0, NOISE_XY),
                    theta + random.gauss(0, NOISE_THETA))

        left, right, reached = go_to(path[target], seen)
        left, right = soften(asked, (left, right))   # soft start, as main.py
        asked = (left, right)
        log.append((turn * DT, pose[0], pose[1], pose[2], left, right))
        if reached:
            target += 1
            if target == len(path):
                break

        # The fake robot moves during one loop turn.
        for _ in range(SUB_STEPS):
            dt = DT / SUB_STEPS
            if realistic:
                wheels = (wheels[0] + (left - wheels[0]) * dt / MOTOR_TIME,
                          wheels[1] + (right - wheels[1]) * dt / MOTOR_TIME)
                pose = move(pose, wheels[0], wheels[1] * RIGHT_WHEEL_FACTOR, dt)
            else:
                pose = move(pose, left, right, dt)
        old_poses = old_poses[1:] + [pose]

    return log


def distance_to_path(x, y, points):
    """Distance from (x, y) to the closest point of the line through points."""
    best = math.inf
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        length2 = (bx - ax) ** 2 + (by - ay) ** 2
        along = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / length2
        along = max(0.0, min(1.0, along))
        best = min(best, math.hypot(x - ax - along * (bx - ax),
                                    y - ay - along * (by - ay)))
    return best


def report(name, log, path, start):
    """Print the numbers that say whether the run was smooth."""
    points = [start[:2]] + list(path)
    worst = max(distance_to_path(x, y, points) for _, x, y, _, _, _ in log)
    end = log[-1]
    miss = math.hypot(end[1] - path[-1][0], end[2] - path[-1][1])

    # Oscillation: how many times the robot goes from steering left to
    # steering right or back. A smooth run does it only at the corners.
    steer = [right - left for _, _, _, _, left, right in log]
    steer = [s for s in steer if abs(s) > 10]      # ignore tiny corrections
    flips = sum(1 for a, b in zip(steer, steer[1:]) if a * b < 0)

    # Stops: loop turns where a wheel command is close to zero, or reversed.
    slow = sum(1 for _, _, _, _, left, right in log if min(left, right) < 20)

    print(f"{name}: {end[0]:.1f} s, "
          f"ends {miss * 100:.1f} cm from the goal, "
          f"at most {worst * 100:.1f} cm from the path, "
          f"{flips} left/right steering flips, "
          f"{slow * DT:.1f} s with a wheel slow or reversed")


def draw(runs, path, start):
    """Draw the trajectories on the lot, and the wheel speeds over time."""
    figure, (lot, speeds) = plt.subplots(1, 2, figsize=(13, 5))

    # The lot: grid lines, cars of the example, waypoints with their tolerance.
    width = common.COLS * common.CELL
    height = common.ROWS * common.CELL
    for col in range(common.COLS + 1):
        lot.plot([col * common.CELL] * 2, [0, height], color="lightblue", lw=1)
    for row in range(common.ROWS + 1):
        lot.plot([0, width], [row * common.CELL] * 2, color="lightblue", lw=1)
    for row in range(common.ROWS):
        for col in range(common.COLS):
            if common.EXAMPLE_GRID[row, col] == 1:
                x, y = common.cell_to_xy(row, col)
                lot.add_patch(plt.Rectangle((x - 0.075, y - 0.075), 0.15, 0.15,
                                            color="black"))
    for x, y in path:
        lot.add_patch(plt.Circle((x, y), common.WAYPOINT_TOL, color="red",
                                 fill=False))
    lot.plot(start[0], start[1], "ko")

    for name, log in runs:
        lot.plot([line[1] for line in log], [line[2] for line in log], label=name)
        times = [line[0] for line in log]
        speeds.plot(times, [line[4] for line in log], label=name + ", left")
        speeds.plot(times, [line[5] for line in log], label=name + ", right",
                    linestyle="--")

    lot.set_aspect("equal")
    lot.set_xlabel("x (m)")
    lot.set_ylabel("y (m)")
    lot.set_title("Trajectory (red circles: WAYPOINT_TOL)")
    lot.legend()
    speeds.set_xlabel("time (s)")
    speeds.set_ylabel("wheel speed asked (Thymio units)")
    speeds.set_title("Wheel speeds given by go_to")
    speeds.legend()
    figure.tight_layout()
    return figure


if __name__ == "__main__":
    start = common.EXAMPLE_POSE
    path = common.EXAMPLE_PATH
    runs = [("perfect", simulate(path, start, realistic=False)),
            ("realistic", simulate(path, start, realistic=True))]
    for name, log in runs:
        report(name, log, path, start)
    draw(runs, path, start).savefig("data/sim_go_to.png", dpi=120)
    print("figure saved as data/sim_go_to.png")
    plt.show()
