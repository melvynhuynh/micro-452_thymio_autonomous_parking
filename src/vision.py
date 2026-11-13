"""Vision block. Owner: Joshua.

From a camera image, find the cars, the robot and the target.

For now these functions give back fake values. Replace what is inside them;
keep their names, their inputs and what they give back (see the README).
"""

import numpy as np

from src import common


def read_frame():
    """Take one image from the camera.

    Gives back the image (as OpenCV reads it), or None if the camera gives
    nothing.
    """
    # TODO (Joshua): read the real camera.
    return np.zeros((480, 640, 3), dtype=np.uint8)


def detect(frame):
    """Find the cars, the robot and the target in one image.

    frame: image from read_frame(), or None.

    Gives back three values:
        grid      array of ROWS x COLS, 0 = free, 1 = car.
                  None when the lot cannot be seen (camera hidden).
        cam_pose  (x, y, theta) of the robot, in metres and radians.
                  None when the robot marker is not found.
        goal      (x, y) centre of the red paper, in metres.
                  None when the red paper is not found.
    """
    if frame is None:
        return None, None, None
    # TODO (Joshua): real detection.
    return common.EXAMPLE_GRID.copy(), common.EXAMPLE_POSE, common.EXAMPLE_GOAL


def to_pixels(x, y):
    """Where a point of the lot is in the camera image (to confirm).

    The live display uses it to draw on the image.

    x, y  position in metres.

    Gives back (u, v): column and row of the pixel, as OpenCV drawing
    functions expect. None as long as the lot has never been seen.
    """
    # TODO (Joshua): real conversion, with what detect() found.
    u = round(x / (common.COLS * common.CELL) * 640)
    v = round(480 - y / (common.ROWS * common.CELL) * 480)
    return u, v
