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

    python -m tests.thymio_main            local navigation = src/local_nav.py
    python -m tests.thymio_main laurent    Laurent's avoid(), read from his
                                           branch origin/local-navigation
    python -m tests.thymio_main empty      the fake camera sees an empty lot
                                           and the target 80 cm straight ahead
    python -m tests.thymio_main empty label=R3.5_40
                                           the label goes into the file names

The robot believes what the fake camera says: with the example lot, it
believes cars stand where the example puts them, even on an empty floor, and
plans around them. Use "empty" to test going around a box on an empty floor:
put the box (not black) about 40 cm straight ahead of the start.
Press Ctrl+C once the robot is parked (state DONE). The run is saved in
data/ (readings and figure).
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
log = []                     # (time, x, y, theta, 5 front prox values)
lot_grid = common.EXAMPLE_GRID.copy()   # what the fake camera sees
lot_goal = common.EXAMPLE_GOAL


def read_and_track():
    """robot.read(), and the pose moved by the wheel speeds since last time."""
    global pose, last_read
    prox, ground, wheel_speeds = robot.read()
    now = time.time()
    if last_read is not None:
        pose = move(pose, wheel_speeds[0], wheel_speeds[1], now - last_read)
    last_read = now
    log.append((now, pose[0], pose[1], pose[2], *prox[:5]))
    return prox, ground, wheel_speeds


def read_frame():
    """No camera: any non-None value means "the lot is visible"."""
    return "no image"


def detect(frame):
    """The lot and target, and the pose from the wheel speeds."""
    return lot_grid.copy(), pose, lot_goal


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
    if "empty" in sys.argv:
        lot_grid[:, :] = 0
        lot_goal = common.cell_to_xy(common.ROWS - 1, common.COLS - 1)
        print(f"Empty lot, target straight ahead at {lot_goal}")
    if "laurent" in sys.argv:
        main.local_nav = laurent_local_nav()
        print("Local navigation: Laurent's avoid() from origin/local-navigation")
    else:
        print("Local navigation: src/local_nav.py of this branch")

    input("Robot at the start, facing +x? Press Enter to start, Ctrl+C once "
          "it is parked.")
    try:
        main.main()
    except KeyboardInterrupt:
        print("Stopped by Ctrl+C.")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    # Optional label, to recognise the run: label=R3.5_40 -> ..._R3.5_40.csv
    for argument in sys.argv:
        if argument.startswith("label="):
            stamp += "_" + argument[len("label="):]
    with open(f"data/thymio_main_{stamp}.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", "x_m", "y_m", "theta_rad", "prox_front_left",
                         "prox_front_centre_left", "prox_front_centre",
                         "prox_front_centre_right", "prox_front_right"])
        writer.writerows([(line[0] - log[0][0], *line[1:]) for line in log])
    draw_log = [(line[0] - log[0][0], *line[1:4], 0, 0) for line in log]
    draw([("Thymio, main.py, pose from wheel speeds", draw_log)],
         [lot_goal] if "empty" in sys.argv else common.EXAMPLE_PATH,
         common.EXAMPLE_POSE, lot_grid).savefig(
        f"data/thymio_main_{stamp}.png", dpi=120)
    print(f"Saved as data/thymio_main_{stamp}.csv and .png")
    plt.show()
