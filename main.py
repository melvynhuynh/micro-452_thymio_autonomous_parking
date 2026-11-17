"""Runs the full demo. Owner: Antoine (to confirm).

About ten times per second: look (camera, sensors), estimate the pose, then
one of four states decides what goes to the wheels (see the README):

    PLAN    robot stopped, ask global navigation for a path
    FOLLOW  drive to the next waypoint of the path
    AVOID   local navigation has the wheels
    DONE    robot stopped on the target

Start it from the root of the repository, stop it with Ctrl+C:

    python main.py

No Thymio or no camera at hand? Swap one import, as the README says, for
example:  from src import stubs as robot
"""

import math
import time

import numpy as np

from src import (common, vision, localization, global_nav, motion, robot,
                 local_nav, visualization)

LOOP_PERIOD = 0.1              # seconds between two turns of the loop
GOAL_MOVED = common.CELL / 2   # the target has moved if it is seen this far
                               # from where it was, in metres
OBSTACLE_AHEAD = 0.16          # where a seen obstacle is, ahead of the wheel
                               # axis, in m: 5.5 cm to the front of the robot,
                               # about 3 cm of sensor range, half a 15 cm box

PLAN, FOLLOW, AVOID, DONE = "PLAN", "FOLLOW", "AVOID", "DONE"


def cell_in_front(pose):
    """Cell (row, col) of the obstacle the robot sees, just in front of it.

    Gives back None if that point is outside the lot, or still in the cell of
    the robot.
    """
    x, y, theta = pose
    front_x = x + OBSTACLE_AHEAD * math.cos(theta)
    front_y = y + OBSTACLE_AHEAD * math.sin(theta)
    if not (0 <= front_x < common.COLS * common.CELL
            and 0 <= front_y < common.ROWS * common.CELL):
        return None
    cell = common.xy_to_cell(front_x, front_y)
    if cell == common.xy_to_cell(x, y):
        return None
    return cell


def with_blocked(grid, blocked):
    """Copy of the camera grid in which the blocked cells count as cars."""
    grid = grid.copy()
    for row, col in blocked:
        grid[row, col] = 1
    return grid


def off_path(pose, leaving, waypoint):
    """True when the robot is not between two waypoints any more.

    Waypoints are one cell apart, so the robot must be either in the cell it
    is leaving or in the cell of the waypoint it drives to. Anywhere else, it
    has been moved by hand or pushed away: a new path is needed.
    """
    here = common.xy_to_cell(pose[0], pose[1])
    return here != leaving and here != common.xy_to_cell(waypoint[0], waypoint[1])


def main(max_turns=None):
    """Run the loop until Ctrl+C (or for max_turns turns, for the tests)."""
    robot.connect()

    state = PLAN
    path = []          # waypoints still to reach
    grid = None        # last grid seen by the camera
    goal = None        # last target seen by the camera
    blocked = set()    # cells (row, col) where an added obstacle was met
    leaving = None     # cell of the last waypoint reached (or of the start)
    sent = (0, 0)      # wheel speeds sent at the last turn, for the soft start

    turn = 0
    last_time = time.time()
    try:
        while max_turns is None or turn < max_turns:
            turn += 1
            now = time.time()
            dt = now - last_time
            last_time = now

            # --- Look -------------------------------------------------------
            frame = vision.read_frame()
            seen_grid, cam_pose, seen_goal = vision.detect(frame)
            prox, ground, wheel_speeds = robot.read()
            pose, cov = localization.update(cam_pose, wheel_speeds, dt)
            override = local_nav.avoid(prox, ground)

            # --- Remember the last grid and the last target seen -----------
            grid_changed = seen_grid is not None and (
                grid is None or not np.array_equal(seen_grid, grid))
            if grid_changed:
                grid = seen_grid

            goal_moved = seen_goal is not None and (
                goal is None or math.dist(seen_goal, goal) > GOAL_MOVED)
            if goal_moved:
                goal = seen_goal
                blocked.clear()      # new target: forget the old obstacles

            # --- Decide -----------------------------------------------------
            left, right = 0, 0       # unless a state says otherwise: stop

            if pose is not None:     # unknown position: stay stopped
                if state == AVOID and override is None:
                    print("AVOID -> PLAN: the way is clear again")
                    state = PLAN

                if state == DONE and goal_moved:
                    print("DONE -> PLAN: the target has moved")
                    state = PLAN

                if state == FOLLOW and (grid_changed or goal_moved
                                        or off_path(pose, leaving, path[0])):
                    print("FOLLOW -> PLAN: the grid, the target or the robot "
                          "has moved")
                    state = PLAN

                if state == PLAN and grid is not None and goal is not None:
                    path = global_nav.plan(with_blocked(grid, blocked), pose, goal)
                    if path:
                        print(f"PLAN -> FOLLOW: {len(path)} waypoints")
                        state = FOLLOW
                        leaving = common.xy_to_cell(pose[0], pose[1])
                    # No path: stay in PLAN, stopped, and try again next turn.

                if state == FOLLOW and override is not None:
                    cell = cell_in_front(pose)
                    if cell is not None:
                        blocked.add(cell)
                    print(f"FOLLOW -> AVOID: obstacle, cell {cell} blocked")
                    state = AVOID

                # This comes after the PLAN block on purpose: with a new
                # path the robot drives on in the same turn, without a stop.
                if state == FOLLOW:
                    left, right, reached = motion.go_to(path[0], pose)
                    left, right = motion.soften(sent, (left, right))
                    if reached:
                        leaving = common.xy_to_cell(path[0][0], path[0][1])
                        path.pop(0)   # the next turn aims at the next waypoint
                        if not path:
                            print("FOLLOW -> DONE: target reached")
                            state = DONE
                            left, right = 0, 0
                            blocked.clear()

                if state == AVOID:
                    left, right = override

            # --- Act and show -----------------------------------------------
            robot.set_speed(left, right)
            sent = (left, right)
            shown_grid = None if grid is None else with_blocked(grid, blocked)
            visualization.show(frame, shown_grid, goal, path, cam_pose, pose, cov)

            time.sleep(max(0.0, LOOP_PERIOD - (time.time() - now)))
    finally:
        robot.set_speed(0, 0)   # whatever happens, stop the wheels


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped by Ctrl+C.")
