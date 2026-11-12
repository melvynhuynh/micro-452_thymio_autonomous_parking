"""Runs the full demo. Owner: Antoine (to confirm).

For now it only calls every block in order, a few times, on fake data, to show
that the blocks fit together. The real loop and its four states (PLAN, FOLLOW,
AVOID, DONE, see the README) are still to write.

Start it from the root of the repository:

    python main.py
"""

import time

from src import (vision, localization, global_nav, motion, robot, local_nav,
                 visualization)

LOOP_PERIOD = 0.1   # seconds between two turns of the loop


def main():
    robot.connect()
    last_time = time.time()

    # TODO (Antoine): replace this by the real loop and its four states.
    for turn in range(3):
        now = time.time()
        dt = now - last_time
        last_time = now

        grid, cam_pose, goal = vision.detect(vision.read_frame())
        prox, ground, wheel_speeds = robot.read()
        pose, cov = localization.update(cam_pose, wheel_speeds, dt)
        override = local_nav.avoid(prox, ground)
        path = global_nav.plan(grid, pose, goal)

        left, right = 0, 0
        if path:
            left, right, reached = motion.go_to(path[0], pose)
        robot.set_speed(left, right)

        visualization.show(grid, goal, path, cam_pose, pose)
        print(f"turn {turn}: pose={pose} goal={goal} "
              f"waypoints={len(path)} override={override} "
              f"speeds=({left}, {right})")

        time.sleep(LOOP_PERIOD)

    robot.set_speed(0, 0)


if __name__ == "__main__":
    main()
