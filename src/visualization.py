"""Live visualisation. Owner: Melvyn (to confirm).

Show, while the robot drives, what the system sees and believes.

For now the function draws nothing. Replace what is inside it; keep its name
and its inputs (see the README).
"""


def show(grid, goal, path, cam_pose, pose):
    """Refresh the live display.

    grid      array of ROWS x COLS, 0 = free, 1 = car, or None.
    goal      (x, y) of the target, in metres, or None.
    path      list of (x, y) waypoints, in metres. Can be empty.
    cam_pose  (x, y, theta) seen by the camera, or None.
    pose      (x, y, theta) best estimate (after filtering), or None.

    Gives back nothing.
    """
    # TODO (Melvyn): draw the grid, the target, the path and the two poses.
