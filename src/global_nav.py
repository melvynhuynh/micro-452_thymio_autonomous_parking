"""Global navigation block. Owner: Malvyn.

Find the path from the robot to the target that avoids the cars of the grid.

For now the function gives back a fake path. Replace what is inside it; keep
its name, its inputs and what it gives back (see the README).
"""

from src import common


def plan(grid, pose, goal):
    """Find the shortest path from the robot to the target.

    grid  array of ROWS x COLS, 0 = free, 1 = car.
    pose  (x, y, theta) of the robot, in metres and radians.
    goal  (x, y) of the target, in metres.

    Gives back the path: a list of (x, y) waypoints in metres, in driving
    order.
        - The start is not in the list.
        - Waypoints are cell centres, except the last one: the exact goal.
        - An empty list [] means no path: the robot or the goal is outside the
          lot or on a car, or the goal cannot be reached.

    Use common.xy_to_cell and common.cell_to_xy to go between metres and cells.
    """
    # TODO (Malvyn): real path planning.
    return list(common.EXAMPLE_PATH)
