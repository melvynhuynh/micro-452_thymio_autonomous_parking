"""Shared constants, helpers and example. Belongs to all four of us.

Only change this file when we all agree, and update the README in the same commit.
"""

import numpy as np

# --- The parking lot ---------------------------------------------------------

CELL = 0.20   # side of a cell, in metres
ROWS = 4      # number of rows of the grid
COLS = 5      # number of columns of the grid

# --- The robot (to measure, Antoine) -----------------------------------------

# Measured on 2026-11-17 with tests/calibrate.py; every run is in data/
# (calibration_results.csv and one recording per run).
THYMIO_TO_MS = 0.000299   # m/s per Thymio speed unit: mean of 4 straight runs
                          # at 100-200 units (0.000286 to 0.000304)
WHEEL_BASE = 0.0964       # m, effective distance between the wheels: mean of
                          # 2 spins (0.0950 and 0.0977)
WAYPOINT_TOL = 0.02       # distance at which a waypoint counts as reached, in m


# --- Cells <-> metres --------------------------------------------------------
# Nobody converts by hand: always use these two functions.

def cell_to_xy(row, col):
    """Centre of the cell (row, col), as (x, y) in metres."""
    return (col + 0.5) * CELL, (ROWS - row - 0.5) * CELL


def xy_to_cell(x, y):
    """Cell (row, col) that contains the point (x, y) in metres."""
    return ROWS - 1 - int(y // CELL), int(x // CELL)


# --- Shared example ----------------------------------------------------------
# The situation of the first sketch. The stubs give back these values.

EXAMPLE_GRID = np.array([
    [0, 0, 0, 0, 1],
    [0, 0, 1, 0, 0],
    [1, 0, 1, 0, 1],
    [0, 0, 0, 0, 0],
])
EXAMPLE_POSE = (0.10, 0.10, 0.0)   # bottom-left cell, facing +x
EXAMPLE_GOAL = (0.90, 0.50)        # row 1, column 4
EXAMPLE_PATH = [(0.30, 0.10), (0.50, 0.10), (0.70, 0.10),
                (0.70, 0.30), (0.70, 0.50), (0.90, 0.50)]
EXAMPLE_PROX = [0, 0, 0, 0, 0, 0, 0]   # nothing in front, nothing behind
EXAMPLE_GROUND = [900, 900]            # white paper under both sensors
EXAMPLE_WHEEL_SPEEDS = (0, 0)          # robot not moving
