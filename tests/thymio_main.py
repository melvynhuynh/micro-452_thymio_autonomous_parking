"""main.py on the real Thymio, before the camera exists.

main.py runs unchanged with the real robot.py and motion.py. Two blocks are
replaced:
    vision      a fake camera: the shared example grid and target, and the
                pose computed from the measured wheel speeds (it drifts, but
                over one run it is close enough);
    global_nav  the small grid planner of tests/fake_world.py, because the
                stub always gives back the same path, even around a blocked
                cell.

Put the Thymio as in tests/thymio_drive.py: at EXAMPLE_POSE (bottom-left cell
of the lot, facing +x), with 1 m free ahead and 0.6 m to its left. Then, from
the root of the repository:

    python -m tests.thymio_main            local navigation = the stub
    python -m tests.thymio_main laurent    Laurent's avoid(), read from his
                                           branch origin/local-navigation

With "laurent", put a box that is not black in the bottom row, third cell
(about 50 cm ahead of the start): the robot should stop before it, back away,
and take the next row up. Press Ctrl+C once the robot is parked (state DONE).
The run is saved in data/ (readings and figure).
"""

import csv
import os
import subprocess
import sys
import time
import types

import matplotlib.pyplot as plt

import main
from src import common, robot
from tests import fake_world
from tests.odometry import move
from tests.sim_go_to import draw

pose = common.EXAMPLE_POSE   # pose from the wheel speeds
last_read = None             # time of the last robot.read()
log = []                     # (time, x, y, theta, prox front max)


def read_and_track():
    """robot.read(), and the pose moved by the wheel speeds since last time."""
    global pose, last_read
    prox, ground, wheel_speeds = robot.read()
    now = time.time()
    if last_read is not None:
        pose = move(pose, wheel_speeds[0], wheel_speeds[1], now - last_read)
    last_read = now
    log.append((now, pose[0], pose[1], pose[2], max(prox[:5])))
    return prox, ground, wheel_speeds


def read_frame():
    """No camera: any non-None value means "the lot is visible"."""
    return "no image"


def detect(frame):
    """The example lot and target, and the pose from the wheel speeds."""
    return common.EXAMPLE_GRID.copy(), pose, common.EXAMPLE_GOAL


def laurent_local_nav():
    """Laurent's local_nav.py, read from his branch without checking it out."""
    repository = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    code = subprocess.run(
        ["git", "show", "origin/local-navigation:src/local_nav.py"],
        capture_output=True, text=True, check=True, cwd=repository).stdout
    module = types.ModuleType("laurent_local_nav")
    exec(code, module.__dict__)
    return module


if __name__ == "__main__":
    main.robot = types.SimpleNamespace(connect=robot.connect,
                                       read=read_and_track,
                                       set_speed=robot.set_speed)
    main.vision = types.SimpleNamespace(read_frame=read_frame, detect=detect)
    main.global_nav = fake_world
    if "laurent" in sys.argv:
        main.local_nav = laurent_local_nav()
        print("Local navigation: Laurent's avoid() from origin/local-navigation")
    else:
        print("Local navigation: the stub (never takes the wheels)")

    input("Robot at the start, facing +x? Press Enter to start, Ctrl+C once "
          "it is parked.")
    try:
        main.main()
    except KeyboardInterrupt:
        print("Stopped by Ctrl+C.")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    with open(f"data/thymio_main_{stamp}.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", "x_m", "y_m", "theta_rad", "prox_front_max"])
        writer.writerows([(t - log[0][0], x, y, th, p) for t, x, y, th, p in log])
    draw_log = [(t - log[0][0], x, y, th, 0, 0) for t, x, y, th, _ in log]
    draw([("Thymio, main.py, pose from wheel speeds", draw_log)],
         common.EXAMPLE_PATH, common.EXAMPLE_POSE).savefig(
        f"data/thymio_main_{stamp}.png", dpi=120)
    print(f"Saved as data/thymio_main_{stamp}.csv and .png")
    plt.show()
