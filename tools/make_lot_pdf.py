"""Draws the parking lot to print on one A0 sheet: parking_lot_A0.pdf.

The sizes come from src/common.py (CELL, ROWS, COLS), so the printed lot
always matches the code. Print it at 100 % ("actual size"), never "fit to page".

Run from the root of the repository:

    python -m tools.make_lot_pdf
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from src import common

# Everything below is in millimetres.
SHEET_W, SHEET_H = 1189, 841        # A0, landscape
BAND = 20                           # width of the black band around the lot
LINE = 1.5                          # width of the blue grid lines
BLUE = (0.25, 0.50, 0.95)           # light enough not to look black to vision
GREY = (0.70, 0.70, 0.70)           # labels outside the lot and in the cells

LOT_W = common.COLS * common.CELL * 1000
LOT_H = common.ROWS * common.CELL * 1000
CELL = common.CELL * 1000
assert LOT_W + 2 * BAND <= SHEET_W and LOT_H + 2 * BAND <= SHEET_H, \
    "the lot and its black band do not fit on an A0 sheet"

# bottom-left inside corner of the black band = origin (0, 0) of the lot
X0 = (SHEET_W - LOT_W) / 2
Y0 = (SHEET_H - LOT_H) / 2

fig = plt.figure(figsize=(SHEET_W / 25.4, SHEET_H / 25.4))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, SHEET_W)
ax.set_ylim(0, SHEET_H)
ax.set_axis_off()


def box(x, y, w, h, color):
    ax.add_patch(Rectangle((x, y), w, h, facecolor=color, edgecolor="none"))


# black band: a black rectangle with the white lot on top of it
box(X0 - BAND, Y0 - BAND, LOT_W + 2 * BAND, LOT_H + 2 * BAND, "black")
box(X0, Y0, LOT_W, LOT_H, "white")

# blue lines between the cells (the band already marks the outside edges)
for col in range(1, common.COLS):
    box(X0 + col * CELL - LINE / 2, Y0, LINE, LOT_H, BLUE)
for row in range(1, common.ROWS):
    box(X0, Y0 + row * CELL - LINE / 2, LOT_W, LINE, BLUE)

# small (row, col) label in the top-left corner of each cell, same numbering
# as grid[row, col]: row 0 at the top
for row in range(common.ROWS):
    for col in range(common.COLS):
        x = X0 + col * CELL + 6
        y = Y0 + (common.ROWS - row) * CELL - 6
        ax.text(x, y, f"{row},{col}", color=GREY, fontsize=18,
                ha="left", va="top")

# left margin: 10 cm scale bar, to check that the print is at 100 %
bar_x = (X0 - BAND) / 2
box(bar_x - 2, Y0 + LOT_H / 2 - 50, 4, 100, GREY)
for tick in range(0, 101, 10):
    box(bar_x - 6, Y0 + LOT_H / 2 - 50 + tick - 0.4, 12, 0.8, GREY)
ax.text(bar_x - 14, Y0 + LOT_H / 2, "Print at 100 %: this bar must measure 10 cm",
        color=GREY, fontsize=24, rotation=90, ha="center", va="center")

# right margin: what the sheet is, and which side is the bottom
ax.text(SHEET_W - (X0 - BAND) / 2, Y0 + LOT_H / 2,
        f"MICRO-452 Thymio parking  ·  {common.COLS} × {common.ROWS} cells "
        f"of {CELL:.0f} mm  ·  origin (0, 0) at the bottom-left inside corner",
        color=GREY, fontsize=24, rotation=90, ha="center", va="center")

# both margins, near the bottom: which long edge is the bottom of the lot
for x in ((X0 - BAND) / 2, SHEET_W - (X0 - BAND) / 2):
    ax.text(x, Y0 + 30, "bottom\n↓", color=GREY, fontsize=30,
            ha="center", va="center", linespacing=1.0)

fig.savefig("parking_lot_A0.pdf", metadata={"Title": "Thymio parking lot, A0"})
print(f"parking_lot_A0.pdf: lot {LOT_W:.0f} × {LOT_H:.0f} mm, "
      f"{BAND} mm black band, sheet {SHEET_W} × {SHEET_H} mm")
