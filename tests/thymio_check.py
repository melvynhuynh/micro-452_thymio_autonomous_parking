"""Step 1 on the real Thymio: does robot.py work? A guided check.

Before starting:
    - the device manager runs (see robot.py), the Thymio is plugged in;
    - the Thymio is on the floor or a table, with 30 cm free all around it
      and nothing near its sensors (no laptop, wall or arm).

From the root of the repository:

    python -m tests.thymio_check

At every step the program says what to do, waits for Enter, measures for
3 seconds, then says whether the robot answered as expected.
"""

import time

from src import robot

SENSOR_NAMES = ["front left", "front centre-left", "front centre",
                "front centre-right", "front right", "back left", "back right"]
SEEN = 1000          # a prox value above this means "something is close"
results = []         # (step, passed)


def measure(seconds=3.0):
    """Read the robot for a few seconds. Gives back the highest prox values,
    the lowest ground values and the list of wheel speeds read."""
    prox_max = [0] * 7
    ground_min = [10000, 10000]
    wheels = []
    end = time.time() + seconds
    while time.time() < end:
        prox, ground, wheel_speeds = robot.read()
        prox_max = [max(a, b) for a, b in zip(prox_max, prox)]
        ground_min = [min(a, b) for a, b in zip(ground_min, ground)]
        wheels.append(wheel_speeds)
        time.sleep(0.1)
    return prox_max, ground_min, wheels


def result(step, passed, details):
    """Print and remember the result of one step."""
    results.append((step, passed))
    print(("    PASS  " if passed else "    FAIL  ") + details)


def step_nothing_around():
    input("\n[1] Move away from the robot: nothing within 20 cm of it. "
          "Press Enter, then wait 3 s.")
    prox, ground, _ = measure()
    print("    highest prox:", prox, " lowest ground:", ground)
    result("nothing around", max(prox) < SEEN,
           "no sensor sees anything" if max(prox) < SEEN else
           "a sensor sees something: move objects away and run again")


def step_one_sensor(number):
    name = SENSOR_NAMES[number]
    input(f"\n[{number + 2}] Hold one finger 1-2 cm in front of the {name} "
          f"sensor (sensor {number + 1} of 7) and keep it still. Press Enter.")
    prox, _, _ = measure()
    print("    highest prox:", prox)
    strongest = prox.index(max(prox))
    passed = prox[number] > SEEN and strongest == number
    result(f"sensor {name}", passed,
           f"the {name} sensor answers most" if passed else
           f"expected sensor {number + 1} ({name}) to answer most, "
           f"got sensor {strongest + 1} ({SENSOR_NAMES[strongest]}) "
           f"with {max(prox)}")


def step_ground():
    input("\n[9] Lift the robot 10 cm above the table and hold it level. "
          "Press Enter, keep it up 3 s, then put it back down.")
    _, ground, _ = measure()
    print("    lowest ground while lifted:", ground)
    result("ground sensors", max(ground) < 300,
           "both ground values drop when lifted" if max(ground) < 300 else
           "ground values did not drop: lift it higher, keep it level")


def step_wheels(number, left, right, what):
    input(f"\n[{number}] The robot will move: {what}. Hands off, watch it. Press Enter.")
    robot.set_speed(left, right)
    _, _, wheels = measure(2.0)
    robot.set_speed(0, 0)
    steady = wheels[len(wheels) // 2:]          # skip the start-up jolt
    mean_left = sum(w[0] for w in steady) / len(steady)
    mean_right = sum(w[1] for w in steady) / len(steady)
    print(f"    asked ({left}, {right}), measured on average "
          f"({mean_left:.0f}, {mean_right:.0f})")
    close = abs(mean_left - left) < 20 and abs(mean_right - right) < 20
    seen = input(f"    Did the robot really go {what}? (y/n): ").strip().lower()
    result(what, close and seen == "y",
           "measured speeds match, and you saw it" if close and seen == "y" else
           "measured speeds do not match the ones asked" if not close else
           "you did not see the expected movement")


if __name__ == "__main__":
    print("Connecting...")
    robot.connect()
    print("Connected. 13 steps, about 2 minutes.")
    try:
        step_nothing_around()
        for number in range(7):
            step_one_sensor(number)
        step_ground()
        step_wheels(10, 100, 100, "straight forward, about 8 cm")
        step_wheels(11, -100, 100, "turning LEFT on the spot (counter-clockwise)")
        step_wheels(12, 100, -100, "turning RIGHT on the spot (clockwise)")
        step_wheels(13, -100, -100, "straight backward, about 8 cm")
    finally:
        robot.set_speed(0, 0)

    print("\n--- Summary")
    for step, passed in results:
        print("PASS" if passed else "FAIL", step)
    failed = sum(1 for _, passed in results if not passed)
    print("ALL GOOD" if failed == 0 else f"{failed} step(s) failed")
