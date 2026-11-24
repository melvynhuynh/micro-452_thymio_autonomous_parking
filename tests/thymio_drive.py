"""Step 3 on the real Thymio: go_to follows the shared example path.

There is no camera yet, so the pose is computed from the measured wheel speeds
(tests/odometry.py). It drifts, but over one metre it is good enough to see
whether the robot drives smoothly.

Put the Thymio on the floor. It believes it starts at EXAMPLE_POSE, facing +x,
so it needs free space: 1 m ahead and 0.6 m to its left. Calibrate first
(tests/calibrate.py), or the turns will not be 90 degrees. From the root:

    python -m tests.thymio_drive

At the end, measure where the robot really is and compare with where it
believes it is. The run is saved in data/ (readings and figure).
"""

import csv
import time

import matplotlib.pyplot as plt

from src import common, robot
from src.motion import go_to, soften
from tests.odometry import move
from tests.sim_go_to import draw

PERIOD = 0.1   # seconds between two turns of the loop, as in main.py

start = common.EXAMPLE_POSE
path = list(common.EXAMPLE_PATH)
pose = start
asked = (0, 0)   # wheel speeds asked at the last turn, for the soft start
log = []       # (time, x, y, theta, left asked, right asked, left, right measured)

print("Connecting (Thymio Suite open? Thymio plugged in?)...")
robot.connect()
input("Robot in place, facing +x? Press Enter to drive (Ctrl+C stops it).")
begin = time.time()
last = begin
try:
    while path and time.time() - begin < 120:
        time.sleep(PERIOD)
        now = time.time()
        _, _, wheel_speeds = robot.read()
        pose = move(pose, wheel_speeds[0], wheel_speeds[1], now - last)
        last = now

        left, right, reached = go_to(path[0], pose)
        left, right = soften(asked, (left, right))   # soft start, as main.py
        if reached:
            print(f"waypoint {path[0]} reached after {now - begin:.1f} s")
            path.pop(0)
        if not path:
            left, right = 0, 0
        robot.set_speed(left, right)
        asked = (left, right)
        log.append((now - begin, pose[0], pose[1], pose[2], left, right,
                    wheel_speeds[0], wheel_speeds[1]))
except KeyboardInterrupt:
    print("Stopped by Ctrl+C.")
finally:
    robot.set_speed(0, 0)

print(f"The robot believes it is at x = {pose[0]:.3f} m, y = {pose[1]:.3f} m, "
      f"theta = {pose[2]:.2f} rad.")
print(f"Target of the path: {common.EXAMPLE_PATH[-1]}. Measure where it really is.")

stamp = time.strftime("%Y%m%d_%H%M%S")
with open(f"data/thymio_drive_{stamp}.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["time_s", "x_m", "y_m", "theta_rad", "left_asked",
                     "right_asked", "left_measured", "right_measured"])
    writer.writerows(log)
draw([("Thymio, pose from wheel speeds", log)], common.EXAMPLE_PATH,
     start).savefig(f"data/thymio_drive_{stamp}.png", dpi=120)
print(f"Saved as data/thymio_drive_{stamp}.csv and .png")
plt.show()
