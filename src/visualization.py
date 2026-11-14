"""Live visualisation. Owner: Melvyn (to confirm).

Show, while the robot drives, what the system sees and believes: the grid,
the target, the path given by global navigation, the camera pose and the
filtered pose with its uncertainty, all drawn over the camera image.

Positions are in metres; vision.to_pixels(x, y) says where they are in the
image, so this file never needs to know how the camera looks at the lot.
"""

import cv2
import numpy as np

from src import common, vision

WINDOW = "Thymio parking"

# True: draw only the cars, the path, the robot and the target over the image.
# False: also the grid, the cars, the camera pose, the uncertainty and a legend.
SIMPLE = False

# Colours, in OpenCV order (blue, green, red). Same as the report figures.
GRID_LINE = (187, 187, 187)
CAR = (95, 95, 95)            # mid-grey: visible on white paper and on black
CAR_EDGE = (215, 215, 215)    # light grey outline and cross
PATH = (189, 103, 148)        # purple: the variant used by plan()
TARGET = (40, 39, 214)        # red
CAM_POSE = (14, 127, 255)     # orange: what the camera sees
POSE = (44, 160, 44)          # green: filtered pose, used by every block
TEXT = (40, 40, 40)

ARROW = 0.08                  # length of the heading arrows, in metres
SIGMAS = 2                    # the ellipse holds about 86 % of the positions


def show(frame, grid, goal, path, cam_pose, pose, cov):
    """Refresh the live display, drawn over the camera image.

    frame     image from vision.read_frame(), or None.
    grid      array of ROWS x COLS, 0 = free, 1 = car, or None.
    goal      (x, y) of the target, in metres, or None.
    path      list of (x, y) waypoints, in metres. Can be empty.
    cam_pose  (x, y, theta) seen by the camera, or None.
    pose      (x, y, theta) best estimate (after filtering), or None.
    cov       3 x 3 array, uncertainty of pose, or None.

    vision.to_pixels(x, y) says where a point in metres is in the image.

    Gives back nothing.
    """
    cv2.imshow(WINDOW, _draw(frame, grid, goal, path, cam_pose, pose, cov))
    cv2.waitKey(1)


def _draw(frame, grid, goal, path, cam_pose, pose, cov):
    """The image show() displays. The frame itself is never changed."""
    if frame is None:
        img = np.full((480, 640, 3), 255, dtype=np.uint8)   # white, like the paper
    else:
        img = frame.copy()

    if vision.to_pixels(0.0, 0.0) is None:
        _legend(img, ["lot not seen yet"], [TEXT])
        return img

    if SIMPLE:
        if grid is not None:
            _draw_cars(img, grid)
        if path:
            _draw_path(img, path, pose)
        if goal is not None:
            cv2.drawMarker(img, _px(goal[0], goal[1]), TARGET, cv2.MARKER_STAR, 22, 2)
        if pose is not None:
            _draw_pose(img, pose, POSE, filled=True)
        return img

    if grid is not None:
        _draw_grid(img, grid)
    if path:
        _draw_path(img, path, pose)
    if goal is not None:
        cv2.drawMarker(img, _px(goal[0], goal[1]), TARGET, cv2.MARKER_STAR, 22, 2)
    if cov is not None and pose is not None:
        _draw_ellipse(img, pose, cov)
    if cam_pose is not None:
        _draw_pose(img, cam_pose, CAM_POSE, filled=False)
    if pose is not None:
        _draw_pose(img, pose, POSE, filled=True)

    _legend(img, ["planned path", "target", "camera pose", "filtered pose"],
            [PATH, TARGET, CAM_POSE, POSE])
    return img


def _px(x, y):
    """Pixel of a point in metres, as a tuple of ints for OpenCV."""
    u, v = vision.to_pixels(x, y)
    return int(u), int(v)


def _cell_corners(grid, row, col):
    """The four corners of a cell, in pixels."""
    rows = grid.shape[0]
    c = common.CELL
    x0, x1 = col * c, (col + 1) * c
    y0, y1 = (rows - 1 - row) * c, (rows - row) * c
    return np.array([_px(x0, y0), _px(x1, y0), _px(x1, y1), _px(x0, y1)], dtype=np.int32)


def _draw_grid(img, grid):
    """Thin grid lines, and the cars (and blocked cells) darkened."""
    rows, cols = grid.shape
    c = common.CELL
    for col in range(cols + 1):
        cv2.line(img, _px(col * c, 0), _px(col * c, rows * c), GRID_LINE, 1)
    for row in range(rows + 1):
        cv2.line(img, _px(0, row * c), _px(cols * c, row * c), GRID_LINE, 1)

    _draw_cars(img, grid)


def _draw_cars(img, grid):
    """Cars and blocked cells: a mid-grey fill crossed out in light grey, so
    they stand out from free cells on a white sheet and on a dark image."""
    cars = [_cell_corners(grid, row, col) for row, col in np.argwhere(grid == 1)]
    if not cars:
        return
    overlay = img.copy()
    for corners in cars:
        cv2.fillPoly(overlay, [corners], CAR)
    cv2.addWeighted(overlay, 0.8, img, 0.2, 0, dst=img)
    for a, b, c, d in cars:
        cv2.line(img, tuple(int(v) for v in a), tuple(int(v) for v in c), CAR_EDGE, 2)
        cv2.line(img, tuple(int(v) for v in b), tuple(int(v) for v in d), CAR_EDGE, 2)
    cv2.polylines(img, cars, True, CAR_EDGE, 2)


def _draw_path(img, path, pose):
    """The path from the robot through every waypoint; the next one larger."""
    points = [_px(x, y) for x, y in path]
    if pose is not None:
        points = [_px(pose[0], pose[1])] + points
    cv2.polylines(img, [np.array(points, dtype=np.int32)], False, PATH, 3)
    for x, y in path[:-1]:
        cv2.circle(img, _px(x, y), 4, PATH, -1)
    cv2.circle(img, _px(*path[0]), 8, PATH, 2)        # next waypoint


def _draw_pose(img, pose, colour, filled):
    """A circle at the robot and an arrow along its heading."""
    x, y, theta = pose
    centre = _px(x, y)
    tip = _px(x + ARROW * np.cos(theta), y + ARROW * np.sin(theta))
    cv2.circle(img, centre, 9, colour, -1 if filled else 2)
    cv2.arrowedLine(img, centre, tip, colour, 2, tipLength=0.3)


def _draw_ellipse(img, pose, cov):
    """Uncertainty of the position: the 2-sigma ellipse of cov[x, y]."""
    values, vectors = np.linalg.eigh(np.asarray(cov, dtype=float)[:2, :2])
    radii = SIGMAS * np.sqrt(np.maximum(values, 0))
    angles = np.linspace(0, 2 * np.pi, 48)
    circle = np.stack([radii[0] * np.cos(angles), radii[1] * np.sin(angles)])
    xy = vectors @ circle + np.array(pose[:2], dtype=float)[:, None]
    points = np.array([_px(x, y) for x, y in xy.T], dtype=np.int32)
    cv2.polylines(img, [points], True, POSE, 1)


def _legend(img, labels, colours):
    """A small white box in the top-left corner naming each colour."""
    height = 18 * len(labels) + 8
    cv2.rectangle(img, (6, 6), (150, 6 + height), (255, 255, 255), -1)
    cv2.rectangle(img, (6, 6), (150, 6 + height), GRID_LINE, 1)
    for i, (label, colour) in enumerate(zip(labels, colours)):
        y = 22 + 18 * i
        cv2.rectangle(img, (12, y - 9), (24, y + 1), colour, -1)
        cv2.putText(img, label, (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, TEXT, 1, cv2.LINE_AA)
