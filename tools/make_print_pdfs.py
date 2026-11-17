"""Draws everything we print, as PDFs in the print/ folder.

    parking_lot_A0.pdf   the empty lot, one A0 sheet
    test_map_A0.pdf      the lot with the shared example printed on it
                         (cars, Thymio, target): to test vision without boxes
    car_box_A3.pdf       pattern of one car box, to cut, fold and glue
    target_A4.pdf        the red target
    marker_A4.pdf        six markers to stick on the Thymio
    calibration_straight_A4.pdf   ruler to measure THYMIO_TO_MS
    calibration_spin_A4.pdf       dial and pointer to measure WHEEL_BASE

The lot sizes come from src/common.py (CELL, ROWS, COLS) and the test map
from its shared example, so the prints always match the code. Always print at
100 % ("actual size"), never "fit to page": each sheet has a 10 cm bar to
check it.

Run from the root of the repository:

    python -m tools.make_print_pdfs
"""

import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

from src import common

OUT = "print"   # folder of the PDFs

# Everything below is in millimetres. Sheets as (width, height).
A0 = (1189, 841)                    # landscape
A3 = (297, 420)                     # portrait
A4 = (210, 297)                     # portrait

BAND = 20                           # width of the black band around the lot
LINE = 1.5                          # width of the blue grid lines

# Same sizes as Joshua's tools/make_test_lot.py
CAR = 150                           # side of a car box, seen from above
TARGET = 100                        # side of the red square
MARKER_L, MARKER_W = 70, 50         # marker, long side along the heading
DOT_D, DOT_R = 25, 10               # dot: centre ahead of the marker centre, radius
THYMIO_R, THYMIO_BACK = 55, 57      # round front radius, flat back behind the axis

CAR_H = 60                          # height of a car box
TAB = 15                            # width of the glue tabs

BLACK = (0, 0, 0)
BLUE = (0.25, 0.50, 0.95)           # light enough not to look black to vision
GREY = (0.70, 0.70, 0.70)           # labels, never inside what vision looks for
RED = (1, 0, 0)
GREEN = (0, 1, 0)
MAGENTA = (1, 0, 1)

CELL = common.CELL * 1000
LOT_W = common.COLS * CELL
LOT_H = common.ROWS * CELL
assert LOT_W + 2 * BAND <= A0[0] and LOT_H + 2 * BAND <= A0[1], \
    "the lot and its black band do not fit on an A0 sheet"

# bottom-left inside corner of the black band = origin (0, 0) of the lot
X0 = (A0[0] - LOT_W) / 2
Y0 = (A0[1] - LOT_H) / 2


# --- Drawing helpers ----------------------------------------------------------

def new_sheet(size):
    """Empty sheet of size (width, height) in mm; draw on it in mm."""
    w, h = size
    fig = plt.figure(figsize=(w / 25.4, h / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.set_axis_off()
    return fig, ax


def save(fig, name, title):
    path = os.path.join(OUT, name)
    fig.savefig(path, metadata={"Title": title})
    plt.close(fig)
    print("written:", path)


def box(ax, x, y, w, h, color):
    """Filled rectangle, bottom-left corner (x, y)."""
    ax.add_patch(Rectangle((x, y), w, h, facecolor=color, edgecolor="none"))


def scale_bar(ax, x, y, vertical=False):
    """10 cm bar starting at (x, y), to check that the print is at 100 %."""
    for tick in range(0, 101, 10):
        if vertical:
            box(ax, x - 6, y + tick - 0.4, 12, 0.8, GREY)
        else:
            box(ax, x + tick - 0.4, y - 6, 0.8, 12, GREY)
    if vertical:
        box(ax, x - 2, y, 4, 100, GREY)
        ax.text(x - 14, y + 50, "Print at 100 %: this bar must measure 10 cm",
                color=GREY, fontsize=24, rotation=90, ha="center", va="center")
    else:
        box(ax, x, y - 2, 100, 4, GREY)
        ax.text(x + 50, y + 9, "this bar must measure 10 cm", color=GREY,
                fontsize=9, ha="center", va="bottom")


def on_robot(pose, a, b):
    """Point (a, b) of the robot's own frame (a forward, b left), in page mm.

    pose = (x, y, theta) of the wheel axis, x and y in page mm.
    """
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    return x + c * a - s * b, y + s * a + c * b


def draw_marker(ax, pose):
    """Green rectangle and magenta dot, centred on the wheel axis."""
    l, w = MARKER_L / 2, MARKER_W / 2
    corners = [on_robot(pose, a, b) for a, b in [(-l, -w), (l, -w), (l, w), (-l, w)]]
    ax.add_patch(Polygon(corners, facecolor=GREEN, edgecolor="none"))
    ax.add_patch(Circle(on_robot(pose, DOT_D, 0), DOT_R,
                        facecolor=MAGENTA, edgecolor="none"))


def draw_thymio(ax, pose, marker=True):
    """Thymio seen from above: flat back, round front, marker on top.

    marker=False draws only the outline, to show where to put the robot.
    """
    outline = [(-THYMIO_BACK, THYMIO_R), (-THYMIO_BACK, -THYMIO_R)]
    for k in range(31):
        angle = -math.pi / 2 + k * math.pi / 30
        outline.append((THYMIO_R * math.cos(angle), THYMIO_R * math.sin(angle)))
    ax.add_patch(Polygon([on_robot(pose, a, b) for a, b in outline],
                         facecolor=(0.90, 0.90, 0.90) if marker else "none",
                         edgecolor=(0.5, 0.5, 0.5), linewidth=2))
    if marker:
        draw_marker(ax, pose)


def lot_mm(x, y):
    """Point (x, y) of the lot, in metres, as page mm on the A0 sheet."""
    return X0 + x * 1000, Y0 + y * 1000


def draw_lot(ax, title):
    """The lot on an A0 sheet: black band, blue lines, cell labels, margins."""
    # black band: a black rectangle with the white lot on top of it
    box(ax, X0 - BAND, Y0 - BAND, LOT_W + 2 * BAND, LOT_H + 2 * BAND, BLACK)
    box(ax, X0, Y0, LOT_W, LOT_H, "white")

    # blue lines between the cells (the band already marks the outside edges)
    for col in range(1, common.COLS):
        box(ax, X0 + col * CELL - LINE / 2, Y0, LINE, LOT_H, BLUE)
    for row in range(1, common.ROWS):
        box(ax, X0, Y0 + row * CELL - LINE / 2, LOT_W, LINE, BLUE)

    # small (row, col) label in the top-left corner of each cell, same
    # numbering as grid[row, col]: row 0 at the top
    for row in range(common.ROWS):
        for col in range(common.COLS):
            ax.text(X0 + col * CELL + 6, Y0 + (common.ROWS - row) * CELL - 6,
                    f"{row},{col}", color=GREY, fontsize=18, ha="left", va="top")

    margin = (X0 - BAND) / 2   # middle of the left and right margins
    scale_bar(ax, margin, Y0 + LOT_H / 2 - 50, vertical=True)
    ax.text(A0[0] - margin, Y0 + LOT_H / 2, title,
            color=GREY, fontsize=24, rotation=90, ha="center", va="center")
    # which long edge is the bottom of the lot
    for x in (margin, A0[0] - margin):
        ax.text(x, Y0 + 30, "bottom\n↓", color=GREY, fontsize=30,
                ha="center", va="center", linespacing=1.0)


# --- The sheets ---------------------------------------------------------------

def parking_lot():
    fig, ax = new_sheet(A0)
    draw_lot(ax, f"MICRO-452 Thymio parking  ·  {common.COLS} × {common.ROWS} "
                 f"cells of {CELL:.0f} mm  ·  origin (0, 0) at the bottom-left "
                 f"inside corner")
    save(fig, "parking_lot_A0.pdf", "Thymio parking lot, A0")


def test_map():
    """The shared example of common.py, printed flat on the lot."""
    fig, ax = new_sheet(A0)
    draw_lot(ax, "TEST MAP  ·  a photo of this sheet must give EXAMPLE_GRID, "
                 "EXAMPLE_POSE and EXAMPLE_GOAL of src/common.py")
    for row in range(common.ROWS):
        for col in range(common.COLS):
            if common.EXAMPLE_GRID[row, col] == 1:
                x, y = lot_mm(*common.cell_to_xy(row, col))
                box(ax, x - CAR / 2, y - CAR / 2, CAR, CAR, BLACK)
    x, y = lot_mm(*common.EXAMPLE_GOAL)
    box(ax, x - TARGET / 2, y - TARGET / 2, TARGET, TARGET, RED)
    x, y = lot_mm(common.EXAMPLE_POSE[0], common.EXAMPLE_POSE[1])
    draw_thymio(ax, (x, y, common.EXAMPLE_POSE[2]))
    save(fig, "test_map_A0.pdf", "Thymio parking test map, A0")


def car_box():
    """Pattern of a box open at the bottom: a black top and four black sides."""
    fig, ax = new_sheet(A3)
    cx, cy = A3[0] / 2, A3[1] / 2 + 20      # centre of the top face
    c, h = CAR / 2, CAR_H

    box(ax, cx - c, cy - c, CAR, CAR, BLACK)            # top, seen by the camera
    box(ax, cx - c, cy + c, CAR, h, BLACK)              # side above
    box(ax, cx - c, cy - c - h, CAR, h, BLACK)          # side below
    box(ax, cx - c - h, cy - c, h, CAR, BLACK)          # side left
    box(ax, cx + c, cy - c, h, CAR, BLACK)              # side right

    # glue tabs on both ends of the sides above and below
    for y0 in (cy + c, cy - c - h):                     # bottom edge of that side
        for x0, out in ((cx - c, -TAB), (cx + c, TAB)):
            ax.add_patch(Polygon([(x0, y0), (x0 + out, y0 + 6),
                                  (x0 + out, y0 + h - 6), (x0, y0 + h)],
                                 facecolor=GREY, edgecolor="none"))

    # fold lines: the four edges of the top, and the base of each tab
    fold = dict(color="white", linewidth=1.5, linestyle=(0, (6, 4)))
    for (x1, y1), (x2, y2) in [((cx - c, cy - c), (cx + c, cy - c)),
                               ((cx - c, cy + c), (cx + c, cy + c)),
                               ((cx - c, cy - c), (cx - c, cy + c)),
                               ((cx + c, cy - c), (cx + c, cy + c)),
                               ((cx - c, cy + c), (cx - c, cy + c + h)),
                               ((cx + c, cy + c), (cx + c, cy + c + h)),
                               ((cx - c, cy - c - h), (cx - c, cy - c)),
                               ((cx + c, cy - c - h), (cx + c, cy - c))]:
        ax.plot([x1, x2], [y1, y2], **fold)

    ax.text(cx, 52,
            f"Car box: {CAR / 10:.0f} × {CAR / 10:.0f} cm, {CAR_H / 10:.0f} cm "
            f"high, open at the bottom.\n"
            "Print at 100 % on thick A3 paper. Cut around the black and the grey "
            "tabs,\nfold along the dashed lines, glue the grey tabs inside. "
            "One sheet per car.",
            color=(0.4, 0.4, 0.4), fontsize=10, ha="center", va="top",
            linespacing=1.4)
    scale_bar(ax, cx - 50, 18)
    save(fig, "car_box_A3.pdf", "Thymio parking car box, A3")


def target():
    fig, ax = new_sheet(A4)
    cx, cy = A4[0] / 2, A4[1] / 2 + 20
    box(ax, cx - TARGET / 2, cy - TARGET / 2, TARGET, TARGET, RED)
    ax.text(cx, cy - TARGET / 2 - 15,
            f"Target: {TARGET / 10:.0f} × {TARGET / 10:.0f} cm. "
            "Print at 100 %, cut along the red edge.",
            color=(0.4, 0.4, 0.4), fontsize=10, ha="center", va="top")
    scale_bar(ax, cx - 50, 25)
    save(fig, "target_A4.pdf", "Thymio parking target, A4")


def markers():
    """Six markers, long side horizontal, front to the right."""
    fig, ax = new_sheet(A4)
    for i in range(6):
        x = A4[0] / 4 * (1 + 2 * (i % 2))           # two columns
        y = 240 - (i // 2) * 65                     # three rows
        draw_marker(ax, (x, y, 0))
        ax.text(x + MARKER_L / 2 + 4, y, "front", color=GREY, fontsize=8,
                ha="left", va="center")
        # grey ticks above and below the centre: put them above the wheel axis
        for side in (1, -1):
            box(ax, x - 0.4, y + side * (MARKER_W / 2 + 3) - 3, 0.8, 6, GREY)
    ax.text(A4[0] / 2, 280,
            f"Thymio marker: {MARKER_L / 10:.0f} × {MARKER_W / 10:.0f} cm "
            "(six copies). Cut along the green edge.",
            color=(0.4, 0.4, 0.4), fontsize=10, ha="center", va="bottom")
    ax.text(A4[0] / 2, 68,
            "Stick it flat on top of the Thymio, centred left-right,\n"
            "long side along the driving direction, magenta dot at the front,\n"
            "and its centre (the grey ticks) right above the wheel axis.",
            color=(0.4, 0.4, 0.4), fontsize=10, ha="center", va="top",
            linespacing=1.4)
    scale_bar(ax, A4[0] / 2 - 50, 25)
    save(fig, "marker_A4.pdf", "Thymio parking marker, A4")


def straight_sheet():
    """Ruler for tests/calibrate.py, straight run (THYMIO_TO_MS).

    The flat back of the Thymio starts on the 0 line. Where it stops is read
    on the two millimetre scales, one on each side of the robot.
    """
    fig, ax = new_sheet(A4)
    cx, start = A4[0] / 2, 22          # middle of the path, height of the 0 line
    length = 260                       # length of the scales, in mm

    # where to put the robot: flat back on the 0 line, facing the top
    draw_thymio(ax, (cx, start + THYMIO_BACK, math.pi / 2), marker=False)
    box(ax, cx - THYMIO_R - 12, start - 0.2, 2 * THYMIO_R + 24, 0.4, BLACK)

    for side in (-1, 1):
        x = cx + side * (THYMIO_R + 14)   # just outside the robot's side
        box(ax, x - 0.15, start, 0.3, length, BLACK)
        for mm in range(length + 1):
            tick = 8 if mm % 10 == 0 else 5 if mm % 5 == 0 else 3
            # ticks point towards the robot, to read along its back edge
            inner = x - side * tick
            box(ax, min(x, inner), start + mm - 0.12, tick, 0.24, BLACK)
            if mm % 10 == 0:
                ax.text(x + side * 3, start + mm, str(mm // 10), fontsize=7,
                        ha="left" if side > 0 else "right", va="center")
        ax.text(x + side * 3, start + length + 6, "cm", fontsize=7,
                ha="left" if side > 0 else "right", va="center")

    ax.text(cx, 11,
            "STRAIGHT RUN (tests/calibrate.py, choice 1).  Flat back of the "
            "Thymio on the 0 line, inside the outline.\n"
            "After the run, read where the flat back is on BOTH scales, to "
            "the millimetre (e.g. 20.35 cm).",
            color=(0.4, 0.4, 0.4), fontsize=6.5, ha="center", va="center",
            linespacing=1.4)
    ax.text(cx, start + length + 6,
            "Print at 100 %: check 0 to 10 cm with a real ruler",
            color=GREY, fontsize=7, ha="center", va="center")
    save(fig, "calibration_straight_A4.pdf", "Thymio calibration, straight run")


def spin_sheet():
    """Dial and pointer for tests/calibrate.py, spin (WHEEL_BASE).

    The Thymio turns in place on the dial, counter-clockwise. A paper pointer
    taped on top of it shows the angle on the 1-degree ticks.
    """
    fig, ax = new_sheet(A4)
    cx, cy = A4[0] / 2, 108            # centre of the dial
    ring = 98                          # outer radius of the ticks
    tip = 87                           # the pointer ends here, inside the ticks

    # where to put the robot: wheel axis on the centre, facing 0 degrees
    draw_thymio(ax, (cx, cy, math.pi / 2), marker=False)
    box(ax, cx - 6, cy - 0.2, 12, 0.4, BLACK)
    box(ax, cx - 0.2, cy - 6, 0.4, 12, BLACK)

    for deg in range(360):
        a = math.radians(deg)
        dx, dy = -math.sin(a), math.cos(a)   # 0 at the top, counter-clockwise
        tick = 10 if deg % 10 == 0 else 6 if deg % 5 == 0 else 3
        ax.plot([cx + dx * ring, cx + dx * (ring - tick)],
                [cy + dy * ring, cy + dy * (ring - tick)],
                color=BLACK, linewidth=0.7 if deg % 10 == 0 else 0.4)
        if deg % 10 == 0:
            ax.text(cx + dx * (ring - 15), cy + dy * (ring - 15), str(deg),
                    fontsize=5.5, ha="center", va="center", rotation=deg)
    ax.text(cx, cy + ring + 4, "degrees, counter-clockwise", color=GREY,
            fontsize=7, ha="center", va="bottom")

    # the pointer to cut out: from 10 mm behind the wheel axis to the tip
    w = 6
    px, py = (A4[0] - tip + 10) / 2, 262   # axis mark of the pointer
    ax.add_patch(Polygon([(px - 10, py - w), (px + tip - 12, py - w),
                          (px + tip, py), (px + tip - 12, py + w),
                          (px - 10, py + w)],
                         facecolor="none", edgecolor=BLACK, linewidth=0.8))
    box(ax, px - 10, py - 0.15, tip + 10, 0.3, BLACK)
    box(ax, px - 0.2, py - w - 3, 0.4, 2 * w + 6, BLACK)
    ax.text(px, py + w + 4, "above the wheel axis", fontsize=6,
            ha="center", va="bottom")
    ax.text(px + tip, py + w + 4, "tip", fontsize=6, ha="center", va="bottom")
    ax.text(A4[0] / 2, 285,
            "SPIN (tests/calibrate.py, choice 2).  Cut out the pointer below. "
            "Tape it flat on top of the Thymio,\n"
            "along the middle line of the robot, tip forward, its axis mark "
            "right above the wheel axis (centre of the marker).",
            color=(0.4, 0.4, 0.4), fontsize=6.5, ha="center", va="center",
            linespacing=1.4)
    ax.text(A4[0] / 2, 240,
            "Robot in the outline, pointer on 0. After the run: count the full "
            "turns, then read the angle\n"
            "under the tip, looking straight down on it (e.g. 2 turns and "
            "137 degrees).",
            color=(0.4, 0.4, 0.4), fontsize=6.5, ha="center", va="center",
            linespacing=1.4)
    scale_bar(ax, 12, 222)
    save(fig, "calibration_spin_A4.pdf", "Thymio calibration, spin")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    parking_lot()
    test_map()
    car_box()
    target()
    markers()
    straight_sheet()
    spin_sheet()
