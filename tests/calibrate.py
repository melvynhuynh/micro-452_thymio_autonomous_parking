"""Step 2 on the real Thymio: measure THYMIO_TO_MS and WHEEL_BASE.

Needs the two A4 sheets print/calibration_straight_A4.pdf and
print/calibration_spin_A4.pdf, printed at 100 %. From the root:

    python -m tests.calibrate

Straight run -> THYMIO_TO_MS. The robot drives straight for about 20 cm; you
read how far it went. THYMIO_TO_MS = distance / (measured wheel speed x time).

Spin -> WHEEL_BASE. The robot turns in place for 15 s; you read the turns it
made. WHEEL_BASE = (right - left wheel distance) / angle turned. Do the
straight runs first: the spin uses the mean THYMIO_TO_MS of the straight runs
saved in data/calibration_results.csv.

Every run is recorded in data/ (one line per reading), and its result is
added to data/calibration_results.csv. Do each run a few times and at a few
speeds, then put the average in src/common.py. The spread of the measured
wheel speeds is the noise the filter needs.
"""

import csv
import math
import statistics
import time

from src import common, robot

PERIOD = 0.1   # seconds between two readings


def record(left, right, duration, name):
    """Drive at (left, right) for duration seconds, reading the wheel speeds.

    Saves every reading in data/<name>.csv and gives back the readings, as
    lines (time, left asked, right asked, left measured, right measured).
    """
    lines = []
    start = time.time()
    robot.set_speed(left, right)
    try:
        # One extra second after the stop, to record the braking too.
        while time.time() - start < duration + 1.0:
            if time.time() - start >= duration and (left, right) != (0, 0):
                left, right = 0, 0
                robot.set_speed(0, 0)
            _, _, (left_measured, right_measured) = robot.read()
            lines.append((time.time() - start, left, right,
                          left_measured, right_measured))
            time.sleep(PERIOD)
    finally:
        robot.set_speed(0, 0)

    with open(f"data/{name}.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", "left_asked", "right_asked",
                         "left_measured", "right_measured"])
        writer.writerows(lines)
    print(f"Recording saved as data/{name}.csv")
    return lines


def travelled(lines):
    """Add up the measured speeds over time, in Thymio units x seconds.

    Gives back two values: the sum for the average of the two wheels (forward
    movement) and the sum for right minus left (turning).
    """
    forward = 0.0
    turning = 0.0
    for before, after in zip(lines, lines[1:]):
        dt = after[0] - before[0]
        forward += (before[3] + before[4]) / 2 * dt
        turning += (before[4] - before[3]) * dt
    return forward, turning


def save_result(kind, speed, measured, result, noise):
    """Add one line to data/calibration_results.csv.

    measured is the distance in metres (straight) or the number of turns
    (spin); result is THYMIO_TO_MS (straight) or WHEEL_BASE (spin).
    """
    with open("data/calibration_results.csv", "a", newline="") as file:
        writer = csv.writer(file)
        if file.tell() == 0:   # new file: start with the names of the columns
            writer.writerow(["date", "kind", "speed", "measured", "result",
                             "speed_noise"])
        writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), kind, speed,
                         measured, result, noise])


def speed_noise(lines, speed):
    """Spread of the measured wheel speeds while the robot drives steadily."""
    steady = [abs(line[3]) for line in lines[10:] if line[1] != 0]
    steady += [abs(line[4]) for line in lines[10:] if line[1] != 0]
    return statistics.stdev(steady) if len(steady) > 1 else 0.0


STRAIGHT_DISTANCE = 0.20   # the straight run aims at 20 cm, to fit on the A4 ruler

# Our printer shrank the straight-run sheet: its 26 cm mark measures 24.9 cm
# on a real ruler (2026-11-17). Readings on that sheet are multiplied by this.
# Measure it again if the sheet is printed again; 1.0 for a print at 100 %.
PRINT_SCALE = 24.9 / 26


def ask_number(question):
    """Ask until the answer is a number. A comma works too: 16,3 = 16.3."""
    while True:
        answer = input(question).strip().replace(",", ".")
        try:
            return float(answer)
        except ValueError:
            print("    Type only the number, for example 16.3")


def straight_results():
    """THYMIO_TO_MS of every straight run saved in data/calibration_results.csv."""
    try:
        with open("data/calibration_results.csv", newline="") as file:
            rows = list(csv.DictReader(file))
    except FileNotFoundError:
        return []
    return [float(row["result"]) for row in rows if row["kind"] == "straight"]


def straight():
    """Straight run on print/calibration_straight_A4.pdf."""
    speed = int(ask_number("Speed in Thymio units (for example 100, 150, 200): "))
    input("Flat back of the Thymio on the 0 line, inside the outline. "
          "Hands off, press Enter.")
    # Drive long enough for about 20 cm, whatever the speed.
    duration = STRAIGHT_DISTANCE / (speed * common.THYMIO_TO_MS)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    lines = record(speed, speed, duration, f"calibration_straight_{speed}_{stamp}")
    left = ask_number("Where is the flat back on the LEFT scale, in cm "
                      "(only the number, for example 20.35)? ")
    right = ask_number("And on the RIGHT scale, in cm? ")
    distance = (left + right) / 2 / 100 * PRINT_SCALE   # real metres
    print(f"Real distance: {distance * 100:.2f} cm "
          f"(sheet reading x {PRINT_SCALE:.4f})")
    forward, _ = travelled(lines)
    result = distance / forward
    noise = speed_noise(lines, speed)
    print(f"THYMIO_TO_MS = {result:.6f} m/s per unit "
          f"(speed noise: {noise:.1f} units)")
    save_result("straight", speed, distance, result, noise)


def spin():
    """Spin on print/calibration_spin_A4.pdf, pointer taped on the robot."""
    speed = int(ask_number("Speed in Thymio units (for example 100, 150): "))
    input("Wheel axis on the cross, robot in the outline, pointer on 0. "
          "Hands off, press Enter.")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    lines = record(-speed, speed, 15.0, f"calibration_spin_{speed}_{stamp}")
    full = int(ask_number("Full turns made (count the times the pointer "
                          "passed 0)? "))
    angle = ask_number("Angle under the tip of the pointer now, in degrees? ")
    turns = full + angle / 360

    # WHEEL_BASE needs THYMIO_TO_MS: the mean of the straight runs saved in
    # data/calibration_results.csv, otherwise the value in common.py.
    to_ms = common.THYMIO_TO_MS
    origin = "common.py"
    saved = straight_results()
    if saved:
        to_ms = statistics.mean(saved)
        origin = f"the mean of the {len(saved)} straight run(s) saved"
    _, turning = travelled(lines)
    result = turning * to_ms / (turns * 2 * math.pi)
    noise = speed_noise(lines, speed)
    print(f"{turns:.3f} turns -> WHEEL_BASE = {result:.4f} m, with "
          f"THYMIO_TO_MS = {to_ms:.6f} from {origin} "
          f"(speed noise: {noise:.1f} units)")
    save_result("spin", speed, turns, result, noise)


if __name__ == "__main__":
    print("Connecting (Thymio Suite open? Thymio plugged in?)...")
    robot.connect()
    while True:
        choice = input("1 = straight run, 2 = spin, anything else = quit: ")
        if choice == "1":
            straight()
        elif choice == "2":
            spin()
        else:
            break
