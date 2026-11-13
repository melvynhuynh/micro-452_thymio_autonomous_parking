"""Motion control block. Owner: Antoine.

Drive the robot towards one waypoint.

For now the function gives back fake values. Replace what is inside it; keep
its name, its inputs and what it gives back (see the README).
"""


def go_to(waypoint, pose):
    """Wheel speeds that bring the robot towards one waypoint.

    waypoint  (x, y) to reach, in metres.
    pose      (x, y, theta) of the robot, in metres and radians.

    Gives back three values:
        left     speed of the left wheel, in Thymio units.
        right    speed of the right wheel, in Thymio units.
        reached  True when the robot is within common.WAYPOINT_TOL of the
                 waypoint.
    """
    # TODO (Antoine): real controller.
    return 0, 0, True
