"""Test of main.py and its four states, without robot and without camera.

main.py runs unchanged. Only the blocks it talks to are replaced: the robot
and the camera by the fake world (tests/fake_world.py), which moves a
simulated robot with the wheel speeds it receives.

Start it from the root of the repository:

    python -m tests.test_main

The trajectories are saved as data/sim_main.png.
"""

import math

import matplotlib.pyplot as plt

import main
from src import common, stubs, global_nav, local_nav
from tests import fake_world

main.LOOP_PERIOD = 0     # no waiting: the fake world has its own clock
failures = 0
drawings = []            # (name, track, grid, obstacle, goals) of each run


def check(name, condition):
    """Print one line per test."""
    global failures
    if not condition:
        failures += 1
    print("    ok  " if condition else "    FAIL", name)


def run(name, turns):
    """Run main.py in the fake world for a number of loop turns."""
    main.robot = fake_world
    main.vision = fake_world
    if hasattr(main.local_nav, "reset"):
        main.local_nav.reset()         # Laurent's avoid() remembers things
    main.main(max_turns=turns)
    drawings.append((name, list(fake_world.track), fake_world.grid.copy(),
                     fake_world.obstacle, fake_world.obstacle_at,
                     fake_world.goal))


def box_gap():
    """Smallest space between the robot (11 cm wide) and the added 15 cm box,
    in m, over the whole run. Below 0, the robot touched it."""
    ox, oy = fake_world.box_centres()[-1]
    gaps = []
    for x, y, _, _ in fake_world.track:
        dx = max(abs(x - ox) - 0.075, 0.0)
        dy = max(abs(y - oy) - 0.075, 0.0)
        gaps.append(math.hypot(dx, dy) - 0.055)
    return min(gaps)


def stuck():
    """True when the fake robot stopped short of the target (state STUCK)."""
    return fake_world.wheels == (0, 0) and not on_target()


def on_target():
    """True when the fake robot stands on the target, wheels stopped."""
    return (math.dist(fake_world.pose[:2], fake_world.goal)
            < 1.5 * common.WAYPOINT_TOL and fake_world.wheels == (0, 0))


def never_in_a_car():
    """True when the fake robot stayed in the lot and never entered a cell
    with a car or the added obstacle."""
    for x, y, _, _ in fake_world.track:
        if not (0 <= x < common.COLS * common.CELL
                and 0 <= y < common.ROWS * common.CELL):
            return False
        cell = common.xy_to_cell(x, y)
        if fake_world.grid[cell] == 1 or cell == fake_world.obstacle:
            return False
    return True


print("--- 1. Robot and camera are the stubs: nothing moves, but nothing breaks")
main.robot, main.vision = stubs, stubs
main.main(max_turns=20)
check("no error", True)

print("--- 2. Shared example, real go_to, path of the stub planner")
fake_world.reset()
main.global_nav, main.local_nav = global_nav, local_nav
run("2. shared example", 400)
check("stops on the target", on_target())
check("never in a car", never_in_a_car())

# From here on a small real planner is needed, to see new paths, and the
# real local navigation of this branch (Laurent's avoid()).
main.global_nav = fake_world
main.local_nav = local_nav

print("--- 3. A box right on a waypoint, next to the edge: stuck, stopped")
fake_world.reset()
fake_world.grid[:, :] = 0                      # an empty lot
first_path = fake_world.plan(fake_world.grid, common.EXAMPLE_POSE,
                             common.EXAMPLE_GOAL)
fake_world.obstacle = common.xy_to_cell(*first_path[1])  # on the 2nd waypoint
run("3. box on a waypoint", 900)
check("stops short of the target (STUCK)", stuck())
print(f"    info smallest gap to the box: {box_gap() * 100:.1f} cm "
      f"(below 0: touched; known limit, detection threshold)")

print("--- 4. The target is moved while the robot drives, then again once parked")
fake_world.reset()


def move_goal_1():
    fake_world.goal = common.cell_to_xy(0, 0)          # top-left cell


def move_goal_2():
    fake_world.goal = common.cell_to_xy(1, 1)


fake_world.events = {80: move_goal_1, 500: move_goal_2}
run("4. target moved twice", 800)
check("stops on the last target", on_target())
check("never in a car", never_in_a_car())

print("--- 5. The robot is moved by hand (kidnapping)")
fake_world.reset()


def kidnap():
    x, y = common.cell_to_xy(0, 0)                     # top-left cell
    fake_world.pose = (x, y, -math.pi / 2)


fake_world.events = {60: kidnap}
run("5. kidnapping", 700)
check("stops on the target", on_target())
check("never in a car", never_in_a_car())

print("--- 6. The camera is hidden for 3 seconds (no filter yet: pose unknown)")
fake_world.reset()
seen = {}


def hide():
    fake_world.camera_on = False


def note_pose():
    seen["hidden"] = fake_world.pose


def show_again():
    seen["after"] = fake_world.pose
    fake_world.camera_on = True


fake_world.events = {50: hide, 55: note_pose, 80: show_again}
run("6. camera hidden", 500)
check("does not move while the pose is unknown", seen["hidden"] == seen["after"])
check("then stops on the target", on_target())

print("--- 7. The obstacle closes the only way: no way around, no error")
fake_world.reset()
fake_world.grid[common.ROWS - 1, 2] = 0
fake_world.grid[:, 2] = 1                      # a wall of cars...
fake_world.grid[common.ROWS - 1, 2] = 0        # ...with one gap, bottom row
fake_world.obstacle = (common.ROWS - 1, 2)     # and the obstacle in the gap
run("7. only way closed", 900)
check("stops short of the target (STUCK)", stuck())
print(f"    info smallest gap to the box: {box_gap() * 100:.1f} cm "
      f"(below 0: touched; known limit, detection threshold)")

print("--- 8. A box at a corner, a car beside it: stuck, stopped")
fake_world.reset()
fake_world.obstacle = (common.ROWS - 1, 3)     # the corner (0.7, 0.1) of the path
run("8. box at a corner", 900)
check("stops short of the target (STUCK)", stuck())
print(f"    info smallest gap to the box: {box_gap() * 100:.1f} cm "
      f"(below 0: touched; known limit, detection threshold)")

print("--- 9. A box right on a waypoint of a straight lane: stuck, stopped")
fake_world.reset()
fake_world.grid[:, :] = 0                      # an empty lot
fake_world.goal = common.cell_to_xy(common.ROWS - 1, common.COLS - 1)
fake_world.obstacle = (common.ROWS - 1, 2)     # bottom row, middle
run("9. box on a waypoint", 900)
check("stops short of the target (STUCK)", stuck())
print(f"    info smallest gap to the box: {box_gap() * 100:.1f} cm "
      f"(below 0: touched; known limit, detection threshold)")

for number, side, y in [(10, "right", -0.01), (11, "left", 0.21)]:
    print(f"--- {number}. A box sticking 2 cm into the lane on the {side}, "
          f"between two waypoints")
    fake_world.reset()
    fake_world.grid[:, :] = 0                  # an empty lot
    fake_world.goal = common.cell_to_xy(common.ROWS - 1, common.COLS - 1)
    # Robot on y = 0.10, 5.5 cm half-width; a 15 cm box centred 11 cm to the
    # side reaches 2 cm into the band the robot sweeps.
    fake_world.obstacle_at = (0.60, y)
    run(f"{number}. box 2 cm in, {side}", 900)
    check("steps aside and stops on the target", on_target())
    check(f"does not touch the box (smallest gap {box_gap() * 100:.1f} cm)",
          box_gap() >= 0)
    check("stays close to its lane (less than 10 cm off)",
          max(abs(y - 0.10) for _, y, _, _ in fake_world.track) < 0.10)

# --- One drawing per run -----------------------------------------------------
figure, axes = plt.subplots(2, 5, figsize=(25, 7))
for axis, (name, track, grid, obstacle, obstacle_at, goal) in zip(axes.flat,
                                                                  drawings):
    for row in range(common.ROWS):
        for col in range(common.COLS):
            x, y = common.cell_to_xy(row, col)
            half = common.CELL / 2
            colour = "white"
            if grid[row, col] == 1:
                colour = "black"
            if (row, col) == obstacle:
                colour = "orange"
            axis.add_patch(plt.Rectangle((x - half, y - half), common.CELL,
                                         common.CELL, facecolor=colour,
                                         edgecolor="lightblue"))
    if obstacle_at is not None:                # a 15 cm box, anywhere
        axis.add_patch(plt.Rectangle((obstacle_at[0] - 0.075,
                                      obstacle_at[1] - 0.075), 0.15, 0.15,
                                     facecolor="orange"))
    axis.plot([p[0] for p in track], [p[1] for p in track])
    axis.plot(track[0][0], track[0][1], "ko")
    axis.plot(goal[0], goal[1], "r*", markersize=12)
    axis.set_xlim(0, common.COLS * common.CELL)
    axis.set_ylim(0, common.ROWS * common.CELL)
    axis.set_aspect("equal")
    axis.set_title(name)
figure.tight_layout()
figure.savefig("data/sim_main.png", dpi=110)

print()
print("ALL GOOD" if failures == 0 else f"{failures} TEST(S) FAILED")
print("figure saved as data/sim_main.png (black: cars, orange: added obstacle)")
