"""Runs the full demo. Owner: Antoine (to confirm).

About ten times per second: look (camera, sensors), estimate the pose, then
one state decides what goes to the wheels (see the README):

    PLAN    robot stopped, ask global navigation for a path
    FOLLOW  drive along the path, aiming at a point of it just ahead
    AVOID   local navigation has the wheels and steers away from an obstacle;
            then back to FOLLOW, on the same path
    STUCK   robot stopped: MAX_AVOIDS avoidances in a row without progress
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
LOOKAHEAD = 0.10               # the robot aims this far ahead of itself along
                               # the path, in m (the "carrot")
SIDESTEP = 0.05                # after an avoidance, the robot follows a line
                               # parallel to the path, this far to the side
                               # it turned to, in m...
SIDESTEP_LENGTH = 0.25         # ...for this distance, in m: a 15 cm box plus
                               # the robot. The box is then beside the robot,
                               # where the front sensors do not look.
SIDESTEP_MORE = 0.03           # seen again while beside it: the box sticks out
                               # further, step this much further aside, in m
SIDESTEP_MAX = 0.06            # never further: 1 cm from a 15 cm car parked
                               # in the next cell (10 + 2.5 - 5.5 - 1 cm)
LOOKAHEAD_SIDESTEP = 0.20      # carrot distance while beside the box, in m:
                               # gentler turns, the front corner stays clear
MIN_PROGRESS = 0.03            # an avoidance after less progress than this
                               # along the path, in m, made no progress...
MAX_AVOIDS = 4                 # ...and after this many in a row the robot
                               # stops: it cannot get past

PLAN, FOLLOW, AVOID, STUCK, DONE = "PLAN", "FOLLOW", "AVOID", "STUCK", "DONE"


def on_segment(point, a, b):
    """How far along the segment a -> b point is: 0 at a, 1 at b."""
    length2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
    if length2 == 0:
        return 0.0
    along = ((point[0] - a[0]) * (b[0] - a[0])
             + (point[1] - a[1]) * (b[1] - a[1])) / length2
    return max(0.0, min(1.0, along))


def point_at(a, b, along):
    """Point of the segment a -> b, along = 0 at a and 1 at b."""
    return a[0] + along * (b[0] - a[0]), a[1] + along * (b[1] - a[1])


def carrot(pose, start, waypoint, shift, lookahead):
    """The point of the path the robot drives to, instead of the waypoint.

    It lies on the segment start -> waypoint, lookahead metres ahead of the
    robot along it. After stepping aside from an obstacle, the robot is pulled back
    onto the line of the path, instead of cutting across to the waypoint.
    It never goes past the waypoint, so corners are still reached exactly.

    shift moves the point sideways, in m, positive to the left of the
    direction of travel: just after an avoidance, the robot follows a line
    parallel to the path, beside the box.
    """
    length = math.dist(start, waypoint)
    if length == 0:
        return waypoint
    along = on_segment(pose, start, waypoint) + lookahead / length
    x, y = point_at(start, waypoint, min(along, 1.0))
    left_x = -(waypoint[1] - start[1]) / length   # unit vector to the left
    left_y = (waypoint[0] - start[0]) / length
    return x + shift * left_x, y + shift * left_y


def off_path(pose, start, waypoint):
    """True when the robot is more than a cell away from the segment it follows.

    Further away than a cell, the robot has been moved by hand or pushed
    away: a new path is needed.
    """
    closest = point_at(start, waypoint, on_segment(pose, start, waypoint))
    return math.dist(pose[:2], closest) > common.CELL


def main(max_turns=None):
    """Run the loop until Ctrl+C (or for max_turns turns, for the tests)."""
    robot.connect()

    state = PLAN
    path = []          # waypoints still to reach
    grid = None        # last grid seen by the camera
    goal = None        # last target seen by the camera
    start = None       # (x, y) where the current segment of the path begins
    avoids = 0         # avoidances in a row without progress
    last_progress = None  # progress along the segment at the last avoidance
    side = 1           # side the last avoidance turned to: +1 left, -1 right
    shift = 0.0        # sideways shift of the carrot after an avoidance, m
    sidestep_left = 0.0  # distance still to drive beside the path, m
    last_xy = None     # (x, y) of the robot at the last turn
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
            # Boxes stand on both sides of the parking bay: while driving
            # into it (last waypoint), only a closer object counts.
            threshold = local_nav.FOLLOW_THRESHOLD
            override = local_nav.avoid(prox, ground, threshold)

            # --- Remember the last grid and the last target seen -----------
            grid_changed = seen_grid is not None and (
                grid is None or not np.array_equal(seen_grid, grid))
            if grid_changed:
                grid = seen_grid

            goal_moved = seen_goal is not None and (
                goal is None or math.dist(seen_goal, goal) > GOAL_MOVED)
            if goal_moved:
                goal = seen_goal

            # --- Decide -----------------------------------------------------
            left, right = 0, 0       # unless a state says otherwise: stop

            if pose is not None:     # unknown position: stay stopped
                if state == AVOID and override is None:
                    if avoids >= MAX_AVOIDS:
                        print(f"AVOID -> STUCK: {avoids} avoidances without "
                              f"progress, the robot stops")
                        state = STUCK
                    else:
                        if sidestep_left > 0 and shift * side > 0:
                            # Already beside the box, on the same side: it
                            # sticks out further than thought.
                            shift = side * min(abs(shift) + SIDESTEP_MORE,
                                               SIDESTEP_MAX)
                        else:
                            shift = side * SIDESTEP
                        sidestep_left = SIDESTEP_LENGTH
                        print(f"AVOID -> FOLLOW: beside the path, "
                              f"{abs(shift) * 100:.0f} cm to the "
                              f"{'left' if side > 0 else 'right'}")
                        state = FOLLOW

                if state == DONE and goal_moved:
                    print("DONE -> PLAN: the target has moved")
                    state = PLAN

                if state == STUCK and (goal_moved or grid_changed):
                    print("STUCK -> PLAN: the target or the map has changed")
                    state = PLAN

                if state == FOLLOW and (grid_changed or goal_moved
                                        or off_path(pose, start, path[0])):
                    print("FOLLOW -> PLAN: the grid, the target or the robot "
                          "has moved")
                    state = PLAN

                if state == PLAN and grid is not None and goal is not None:
                    path = global_nav.plan(grid, pose, goal)
                    if path:
                        print(f"PLAN -> FOLLOW: {len(path)} waypoints")
                        state = FOLLOW
                        start = (pose[0], pose[1])
                        avoids = 0
                        last_progress = None
                        shift = sidestep_left = 0.0
                    # No path: stay in PLAN, stopped, and try again next turn.

                if state == FOLLOW and override is not None:
                    # Progress: how far along the current segment the robot
                    # is. Avoidances that do not move it on count up to STUCK.
                    progress = (on_segment(pose, start, path[0])
                                * math.dist(start, path[0]))
                    if (last_progress is not None
                            and progress - last_progress < MIN_PROGRESS):
                        avoids += 1
                    else:
                        avoids = 1
                    last_progress = progress
                    # Laurent's avoid() turns left when the right wheel goes
                    # faster than the left one.
                    side = 1 if override[1] > override[0] else -1
                    print(f"FOLLOW -> AVOID: obstacle, avoidance {avoids} "
                          f"in a row without progress")
                    state = AVOID

                # This comes after the PLAN block on purpose: with a new
                # path the robot drives on in the same turn, without a stop.
                if state == FOLLOW:
                    if sidestep_left > 0 and last_xy is not None:
                        sidestep_left -= math.dist(pose[:2], last_xy)
                        if sidestep_left <= 0:
                            shift = 0.0   # past the box: back on the path
                    lookahead = LOOKAHEAD_SIDESTEP if shift else LOOKAHEAD
                    target = carrot(pose, start, path[0], shift, lookahead)
                    left, right, _ = motion.go_to(target, pose)
                    left, right = motion.soften(sent, (left, right))
                    # Beside the path, passing next to a waypoint counts as
                    # reaching it. Not the last one: the target is exact.
                    tolerance = common.WAYPOINT_TOL
                    if len(path) > 1:
                        tolerance += abs(shift)
                    if math.dist(pose[:2], path[0]) < tolerance:
                        start = path.pop(0)   # the next turn aims further on
                        avoids = 0
                        last_progress = None
                        if not path:
                            print("FOLLOW -> DONE: target reached")
                            state = DONE
                            left, right = 0, 0

                if state == AVOID:
                    left, right = override

            # --- Act and show -----------------------------------------------
            robot.set_speed(left, right)
            sent = (left, right)
            if pose is not None:
                last_xy = (pose[0], pose[1])
            visualization.show(frame, grid, goal, path, cam_pose, pose, cov)

            time.sleep(max(0.0, LOOP_PERIOD - (time.time() - now)))
    finally:
        robot.set_speed(0, 0)   # whatever happens, stop the wheels


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped by Ctrl+C.")
