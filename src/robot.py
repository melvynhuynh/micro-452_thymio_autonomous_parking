"""Link with the Thymio. Owner: Antoine.

The only file that talks to the robot: reads its sensors, sets its wheels.

For now these functions are fake: no robot is needed. Replace what is inside
them; keep their names, their inputs and what they give back (see the README).
"""

from src import common


def connect():
    """Connect to the Thymio. Call it once, before anything else."""
    # TODO (Antoine): real connection.


def read():
    """Read the sensors of the Thymio.

    Gives back three values:
        prox          list of 7 numbers, horizontal proximity sensors, in the
                      Thymio order: front left, front centre-left, front
                      centre, front centre-right, front right, back left,
                      back right. 0 = nothing, larger = closer.
        ground        list of 2 numbers, ground sensors, left then right.
                      High on white paper, low on the black border.
        wheel_speeds  (left, right) speeds measured by the Thymio, in Thymio
                      units.
    """
    # TODO (Antoine): read the real robot.
    return (list(common.EXAMPLE_PROX), list(common.EXAMPLE_GROUND),
            common.EXAMPLE_WHEEL_SPEEDS)


def set_speed(left, right):
    """Set the speed of the two wheels, in Thymio units. (0, 0) stops."""
    # TODO (Antoine): send to the real robot.
