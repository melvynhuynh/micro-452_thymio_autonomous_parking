"""Local navigation block. Owner: Laurent.

Avoid, with the sensors of the Thymio, obstacles that are not on the map, and
never cross the black border.

For now the function says that nothing is ever in the way. Replace what is
inside it; keep its name, its inputs and what it gives back (see the README).
"""


def avoid(prox, ground):
    """Decide whether local navigation must take the wheels.

    prox    list of 7 numbers, horizontal proximity sensors (see robot.read).
    ground  list of 2 numbers, ground sensors, left then right.

    Gives back:
        None           nothing is in the way: motion control keeps the wheels.
        (left, right)  wheel speeds, in Thymio units, to go around the
                       obstacle. On the black border: (0, 0).
    """
    # TODO (Laurent): real avoidance.
    return None
