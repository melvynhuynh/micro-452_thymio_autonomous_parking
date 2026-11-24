"""Measure the camera noise (Q of the filter). Owner: Joshua.

The robot stands still at known spots; at each, read many camera poses
and compare them with the true pose.

    python -m tests.camera_noise                  record
    python -m tests.camera_noise analyse <file>   print bias, jitter and Q
"""

import csv
import sys
import time

import numpy as np

from src import common, vision, localization

N_FRAMES = 100      # camera readings per pose
PERIOD = 0.1        # seconds between two readings

POSES = [
    ("r0c0_0", 0, 0, 0),
    ("r0c0_90", 0, 0, np.pi/2),
    ("r0c4_0", 0, 4, 0),
    ("r0c4_90", 0, 4, np.pi/2),
    ("r3c0_0", 3, 0, 0),
    ("r3c0_90", 3, 0, np.pi/2),
    ("r3c4_0", 3, 4, 0),
    ("r3c4_90", 3, 4, np.pi/2),
    ("r2c2_0", 2, 2, 0),
    ("r2c2_90", 2, 2, np.pi/2),
]       # part A

def record_pose(label, true_x, true_y, true_theta, n=N_FRAMES):
    """Read the camera n times while the robot stands still at a known pose.

    Gives back one row per reading: label, true x, y, theta, then the camera
    x, y, theta, or three empty values when the robot was not found.
    """
    for _ in range(5):        # skip old frames still queued in the camera
        vision.read_frame()

    rows=[]
    for _ in range(n):
        frame = vision.read_frame()
        grid, pose, goal = vision.detect(frame)       
        if pose is None:
            rows.append([label, true_x, true_y, true_theta, "", "", ""])  
        else:
            rows.append([label, true_x, true_y, true_theta, pose[0], pose[1], pose[2]])
        time.sleep(PERIOD)

    found = sum(1 for row in rows if row[4] != "")
    print(f"{label}: robot found in {found} / {n} frames")
    return rows


def record():                                          # part C
    """Go through all the poses, record each one, and save everything in data/."""
    all_rows = []
    for label, row, col, theta in POSES:
        true_x, true_y = common.cell_to_xy(row, col)
        input(f"\n{label}: wheel axis centre on cell ({row}, {col}), "
              f"facing {np.degrees(theta):.0f} deg, hands out of view, then Enter ")
        time.sleep(1)                 # time to get the arm out of the picture
        all_rows += record_pose(label, true_x, true_y, theta)

    name = "data/camera_noise_" + time.strftime("%Y%m%d_%H%M%S") + ".csv"
    with open(name, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["label", "true_x", "true_y", "true_theta",
                         "cam_x", "cam_y", "cam_theta"])
        writer.writerows(all_rows)
    print(f"\nSaved {name}")
    print(f"Next: python -m tests.camera_noise analyse {name}")

def analyse(filename):                                 # part D
    """Print the bias and jitter of the camera at each pose, then Q for the filter.

    Error of one frame = camera pose - true pose.
    bias   = mean error at one pose (always off the same way)
    jitter = std of the error at one pose (jumping around)
    Q      = mean of error² over all frames: holds both bias and jitter.
    """
    errors = {}   # label -> list of (ex, ey, et), only frames where the robot was found
    total = {}    # label -> number of frames recorded
    truth = {}    # label -> true (x, y, theta)
    with open(filename, newline="") as f:
        for row in csv.DictReader(f):
            label = row["label"]
            truth[label] = (float(row["true_x"]), float(row["true_y"]),
                            float(row["true_theta"]))
            total[label] = total.get(label, 0) + 1
            errors.setdefault(label, [])
            if row["cam_x"] == "":        # robot not found in this frame
                continue
            ex = float(row["cam_x"]) - float(row["true_x"])
            ey = float(row["cam_y"]) - float(row["true_y"])
            et = localization.wrap(float(row["cam_theta"]) - float(row["true_theta"]))
            errors[label].append((ex, ey, et))

    # one line per pose: x, y in mm, theta in degrees
    print(f"{'pose':9s} {'found':>9s}   {'bias x':>7s} {'y':>6s} {'theta':>6s}"
          f"   {'jitter x':>8s} {'y':>6s} {'theta':>6s}")
    for label in errors:
        if not errors[label]:
            print(f"{label:9s} {0:>4d}/{total[label]:<4d}   robot never found")
            continue
        e = np.array(errors[label])       # n x 3
        bias = e.mean(axis=0)
        jitter = e.std(axis=0)
        print(f"{label:9s} {len(e):>4d}/{total[label]:<4d}   "
              f"{bias[0]*1000:7.1f} {bias[1]*1000:6.1f} {np.degrees(bias[2]):6.2f}   "
              f"{jitter[0]*1000:8.1f} {jitter[1]*1000:6.1f} {np.degrees(jitter[2]):6.2f}")

    # all frames together -> Q
    all_errors = [err for label in errors for err in errors[label]]
    if not all_errors:
        print("\nRobot never found: no Q")
        return
    E = np.array(all_errors)
    q = (E ** 2).mean(axis=0)             # sigma_x², sigma_y², sigma_theta²
    std = np.sqrt(q)
    print(f"\nTypical camera error: {std[0]*1000:.1f} mm, {std[1]*1000:.1f} mm, "
          f"{np.degrees(std[2]):.2f} deg")
    print("Paste in src/localization.py:")
    print(f"Q = np.diag([{q[0]:.2e}, {q[1]:.2e}, {q[2]:.2e}])")

    # --- What in the bias comes from the marker and the marker height --------
    # Mean error at a pose = marker offset, turning with the robot (f ahead,
    # l to the left) + a stretch k away from the point below the camera (p0):
    #   ex = cos(t) f - sin(t) l + k x + a,   ey = sin(t) f + cos(t) l + k y + b
    # with a = -k p0x and b = -k p0y. Linear in f, l, k, a, b: least squares.
    used = [label for label in errors if errors[label]]
    if len(used) < 3 or len({truth[label][2] for label in used}) < 2:
        print("\nFit needs at least 3 poses and 2 headings: skipped")
        return
    rows, values = [], []
    for label in used:
        x, y, t = truth[label]
        ex, ey, _ = np.array(errors[label]).mean(axis=0)
        rows.append([np.cos(t), -np.sin(t), x, 1, 0])
        values.append(ex)
        rows.append([np.sin(t), np.cos(t), y, 0, 1])
        values.append(ey)
    A, values = np.array(rows), np.array(values)
    solution = np.linalg.lstsq(A, values, rcond=None)[0]
    f, l, k, a, b = solution
    left = np.sqrt(np.mean((values - A @ solution) ** 2))
    angle = E[:, 2].mean()

    print(f"\nFit of the bias (error left after it: {left * 1000:.1f} mm)")
    print(f"  marker {f * 1000:+.1f} mm ahead of the wheel axis, "
          f"{l * 1000:+.1f} mm to the left (should be about 0)")
    print(f"  marker turned by {np.degrees(angle):+.2f} deg")
    print(f"  stretch from the marker height: {k * 100:.1f} %")
    print("Paste in src/vision.py:")
    print(f"MARKER_D = {vision.MARKER_D + f:.4f}")
    print(f"MARKER_ANGLE = {vision.MARKER_ANGLE + angle:.4f}")
    if abs(k) < 0.005:
        print("# HEIGHT_K, CAM_FLOOR: keep as they are (stretch under 0.5 %)")
    elif vision.HEIGHT_K == 0:
        print(f"HEIGHT_K = {k:.4f}")
        print(f"CAM_FLOOR = ({-a / k:.3f}, {-b / k:.3f})")
    else:
        print("# Stretch left although HEIGHT_K is set: set HEIGHT_K = 0, "
              "record again, and paste the new values")


if __name__ == "__main__":                             # part E
    if len(sys.argv) >= 3 and sys.argv[1] == "analyse":
        analyse(sys.argv[2])
    else:
        record()