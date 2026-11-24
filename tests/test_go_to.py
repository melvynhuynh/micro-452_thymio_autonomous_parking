"""Test of go_to by hand: a few poses, and what the wheels must do.

No robot and no camera needed. Start it from the root of the repository:

    python -m tests.test_go_to
"""

import math

from src import common
from src.motion import go_to, soften, wrap

failures = 0


def check(name, condition):
    """Print one line per test."""
    global failures
    if not condition:
        failures += 1
    print("ok  " if condition else "FAIL", name)


# The robot is at the centre of a cell; the waypoint is the next cell centre.
here = common.cell_to_xy(common.ROWS - 1, 0)    # bottom-left cell
right_cell = common.cell_to_xy(common.ROWS - 1, 1)
up_cell = common.cell_to_xy(common.ROWS - 2, 0)
x, y = here

print("--- heading error brought back between -pi and pi")
check("wrap(0) = 0", wrap(0) == 0)
check("wrap(pi) = pi", math.isclose(wrap(math.pi), math.pi))
check("wrap(-pi) = pi", math.isclose(wrap(-math.pi), math.pi))
check("wrap(3*pi/2) = -pi/2", math.isclose(wrap(3 * math.pi / 2), -math.pi / 2))
check("wrap(-3*pi/2) = pi/2", math.isclose(wrap(-3 * math.pi / 2), math.pi / 2))

print("--- waypoint straight ahead: both wheels forward, same speed")
left, right, reached = go_to(right_cell, (x, y, 0.0))
print("    wheels:", left, right)
check("left == right", left == right)
check("forward", left > 0)
check("not reached", reached is False)

print("--- waypoint on the left (90 degrees): turn in place to the left")
left, right, reached = go_to(up_cell, (x, y, 0.0))
print("    wheels:", left, right)
check("left wheel backwards, right wheel forwards", left < 0 < right)
check("no forward speed", left == -right)

print("--- waypoint on the right (90 degrees): turn in place to the right")
left, right, reached = go_to(right_cell, (x, y, math.pi / 2))
print("    wheels:", left, right)
check("right wheel backwards, left wheel forwards", right < 0 < left)
check("no forward speed", left == -right)

print("--- waypoint a little to the left (10 degrees): drive and steer")
left, right, reached = go_to(right_cell, (x, y, math.radians(-10)))
print("    wheels:", left, right)
check("both wheels forward", left > 0 and right > 0)
check("right wheel faster", right > left)

print("--- waypoint behind: turn in place")
left, right, reached = go_to(right_cell, (x, y, math.radians(170)))
print("    wheels:", left, right)
check("no forward speed", left == -right and left != 0)

print("--- the short way around: robot faces 170, waypoint is at -170 degrees")
far_left = (x - 1.0, y - math.tan(math.radians(10)))
left, right, reached = go_to(far_left, (x, y, math.radians(170)))
print("    wheels:", left, right)
check("turns left by 20 degrees, not right by 340", right > left)

print("--- mirror: same situation on the other side swaps the wheels")
a = go_to((x + 0.2, y + 0.05), (x, y, 0.0))
b = go_to((x + 0.2, y - 0.05), (x, y, 0.0))
print("    wheels:", a[:2], b[:2])
check("left and right swapped", a[0] == b[1] and a[1] == b[0])

print("--- reached only inside WAYPOINT_TOL")
inside = (right_cell[0] - 0.9 * common.WAYPOINT_TOL, right_cell[1], 0.0)
outside = (right_cell[0] - 1.1 * common.WAYPOINT_TOL, right_cell[1], 0.0)
check("inside the tolerance: reached", go_to(right_cell, inside)[2] is True)
check("outside the tolerance: not reached", go_to(right_cell, outside)[2] is False)
check("still driving when reached (no stop at each waypoint)",
      go_to(right_cell, inside)[0] > 0)

print("--- unknown pose: stop")
check("(0, 0, False)", go_to(right_cell, None) == (0, 0, False))

print("--- no hard switch: wheel speeds change smoothly with the heading error")
biggest_jump = 0
previous = None
for step in range(-1790, 1800):          # heading error from -179 to 179.9 degrees
    theta = -math.radians(step / 10)
    left, right, reached = go_to(right_cell, (x, y, theta))
    if previous is not None:
        biggest_jump = max(biggest_jump, abs(left - previous[0]),
                           abs(right - previous[1]))
    previous = (left, right)
print("    biggest change for 0.1 degree of heading error:", biggest_jump,
      "Thymio units")
check("never more than 2 Thymio units", biggest_jump <= 2)

print("--- soft start: the forward speed rises slowly, the rest is immediate")
check("start: (0, 0) -> (150, 150) gives (30, 30)",
      soften((0, 0), (150, 150)) == (30, 30))
check("start while steering: (0, 0) -> (140, 160) gives (20, 40)",
      soften((0, 0), (140, 160)) == (20, 40))
check("corner: (150, 150) -> (-240, 240) goes through at once",
      soften((150, 150), (-240, 240)) == (-240, 240))
check("slowing down goes through at once",
      soften((150, 150), (60, 60)) == (60, 60))
check("steering at the same speed goes through at once",
      soften((100, 100), (110, 90)) == (110, 90))
speeds, turns = (0, 0), 0
while speeds != (150, 150):
    speeds, turns = soften(speeds, (150, 150)), turns + 1
check(f"0 to 150 in {turns} turns (0.5 s)", turns == 5)

print()
print("ALL GOOD" if failures == 0 else f"{failures} TEST(S) FAILED")
