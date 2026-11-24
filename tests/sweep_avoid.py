"""Limits of obstacle avoidance: where does the robot get past a box?

main.py runs unchanged in the fake world (tests/fake_world.py) with Laurent's
real avoid(), on an empty lot: start (0.1, 0.1) facing +x, target (0.9, 0.1)
straight ahead, waypoints every 20 cm. A 15 cm box is put beside the path,
sticking into the band the robot sweeps (11 cm wide, y from 0.045 to 0.155)
by 0 to 8 cm, on the right or on the left, at four places along the path.

For each case: parked or STUCK, how close the robot came to the box (a gap
below 0 means it touched), and how far it left the line of its path.

Start it from the root of the repository (about two minutes):

    python -m tests.sweep_avoid

The table is printed and saved as data/sweep_avoid.csv.
"""

import contextlib
import csv
import io
import math

import main
from src import common, local_nav
from tests import fake_world

main.LOOP_PERIOD = 0      # no waiting: the fake world has its own clock
BOX = 0.075               # half the side of the box, in m
ROBOT = 0.055             # half the width of the robot, in m

PLACES = [(0.40, "between waypoints, early"),
          (0.50, "on a waypoint"),
          (0.60, "between waypoints"),
          (0.80, "just before the target")]
DEPTHS_CM = [0, 1, 2, 3, 4, 5, 6, 7, 8]


def gap_to_box(x, y, box):
    """Space between the robot (a circle of ROBOT) and the square box, in m."""
    dx = max(abs(x - box[0]) - BOX, 0.0)
    dy = max(abs(y - box[1]) - BOX, 0.0)
    return math.hypot(dx, dy) - ROBOT


def one_case(box_x, depth, side):
    """Run main.py with one box. side: -1 right of the path, +1 left."""
    # Box edge 'depth' into the band the robot sweeps around y = 0.10.
    if side < 0:
        box_y = 0.10 - ROBOT + depth - BOX
    else:
        box_y = 0.10 + ROBOT - depth + BOX
    fake_world.reset()
    fake_world.grid[:, :] = 0
    fake_world.goal = (0.9, 0.1)
    fake_world.obstacle_at = (box_x, box_y)
    main.robot = main.vision = main.global_nav = fake_world
    main.local_nav = local_nav
    local_nav.reset()
    printed = io.StringIO()
    with contextlib.redirect_stdout(printed):
        main.main(max_turns=900)
    log = printed.getvalue()

    parked = (math.dist(fake_world.pose[:2], fake_world.goal)
              < 1.5 * common.WAYPOINT_TOL and fake_world.wheels == (0, 0))
    result = "parked" if parked else ("STUCK" if "STUCK" in log else "moving")
    gap = min(gap_to_box(x, y, (box_x, box_y))
              for x, y, _, _ in fake_world.track)
    off = max(abs(y - 0.10) for _, y, _, _ in fake_world.track)
    return result, gap, off, log.count("-> AVOID")


if __name__ == "__main__":
    rows = []
    for box_x, place in PLACES:
        for side, side_name in [(-1, "right"), (1, "left")]:
            print(f"\nBox at x = {box_x:.2f} ({place}), on the {side_name}")
            print("  depth  result   gap to box  off the path  avoidances")
            for depth_cm in DEPTHS_CM:
                result, gap, off, avoids = one_case(box_x, depth_cm / 100,
                                                    side)
                flag = "  TOUCH" if gap < 0 else ""
                print(f"  {depth_cm:3d} cm  {result:7s}  {gap * 100:6.1f} cm"
                      f"   {off * 100:6.1f} cm    {avoids:3d}{flag}")
                rows.append([box_x, place, side_name, depth_cm, result,
                             round(gap * 100, 1), round(off * 100, 1), avoids])

    with open("data/sweep_avoid.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["box_x_m", "place", "side", "depth_cm", "result",
                         "gap_to_box_cm", "off_path_cm", "avoidances"])
        writer.writerows(rows)
    print("\nSaved as data/sweep_avoid.csv")
