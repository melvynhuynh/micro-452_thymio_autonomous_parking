"""Vision block. Owner: Joshua.

From a camera image, find the cars, the robot and the target.

Detection works on test images; real camera, automatic corners and to_pixels still to do
"""

import numpy as np
import cv2

from src import common

PX_PER_M = 1000
LOT_W_PX = int(round(common.COLS * common.CELL * PX_PER_M))
LOT_H_PX = int(round(common.ROWS * common.CELL * PX_PER_M))
CELL_PX = int(round(common.CELL * PX_PER_M))
CAR_THRESHOLD = 0.1
BLACK_V_MAX=50  #Threshold for black boxes, 30-70 maybe depends on lighting

MARKER_D    = 0.0     # marker centre ahead of the wheel axis, m

CORNERS = [(50, 50), (1250, 50), (1250, 850), (50, 850)]

def warp_to_lot(frame, corners):
    """Warp the camera image to a top-down view of the lot."""
    src = np.float32(corners)
    dst = np.float32([[0,0],[LOT_W_PX,0],[LOT_W_PX,LOT_H_PX],[0,LOT_H_PX]])
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(frame, M, (LOT_W_PX, LOT_H_PX))


def px_to_m(u, v):
    """Convert pixel coordinates to meters."""
    x =  u / PX_PER_M   # metres column
    y =  (LOT_H_PX-v) / PX_PER_M   # metres row
    return x,y

def largest_blob_centre(mask, min_area=100):
    """Centre (u, v) in pixels of the largest white blob in a mask,
    or None if there is none bigger than min_area."""

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return None

    largest_contour = max(contours, key=cv2.contourArea)

    if cv2.contourArea(largest_contour) < min_area:
        return None

    m = cv2.moments(largest_contour)
    u=m["m10"] / m["m00"]
    v=m["m01"] / m["m00"]
    
    return u,v

def find_goal(top):
    """Find the goal in the image."""
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([160, 150, 70])
    upper_red1 = np.array([180, 255, 255])
    lower_red2 = np.array([0, 150, 70])
    upper_red2 = np.array([10, 255, 255])
    mask1=cv2.inRange(hsv, lower_red1, upper_red1)
    mask2=cv2.inRange(hsv, lower_red2, upper_red2)

    mask = cv2.bitwise_or(mask1, mask2)

    centre = largest_blob_centre(mask)
    if centre is None:
        return None
    
    return px_to_m(*centre)
    

def find_grid(top):
    """Find the cars in the lot."""
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([0, 0, 0])
    upper_bound = np.array([180, 255, BLACK_V_MAX])
    mask=cv2.inRange(hsv, lower_bound, upper_bound)
    grid = np.zeros((common.ROWS, common.COLS), dtype=int)
    for row in range(common.ROWS):
        for col in range(common.COLS):
            v0=row*CELL_PX
            v1=(row+1)*CELL_PX
            u0=col*CELL_PX    
            u1=(col+1)*CELL_PX
            block = mask[v0:v1, u0:u1]
            fraction_black=np.count_nonzero(block) / block.size
            if fraction_black> CAR_THRESHOLD:
                grid[row, col] = 1
    return grid

def find_marker(top):
    """Function to find the marker on top of the robot 
    Returns two points, (centre,front) in metres, the markers center 
    and the circle marker at the fronts center
    """
    #Test version of the function, real version depends on the marker we choose to use
    #Green rectangle as marker and magenta circle as front marker
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)

    lower_green = np.array([40, 150, 70])
    upper_green = np.array([80, 255, 255])
    lower_magenta = np.array([140, 150, 70])
    upper_magenta = np.array([165, 255, 255])
    mask_green=cv2.inRange(hsv, lower_green, upper_green)
    mask_magenta=cv2.inRange(hsv, lower_magenta, upper_magenta)

    centre_marker=largest_blob_centre(mask_green)
    centre_circle=largest_blob_centre(mask_magenta)

    if centre_marker is None or centre_circle is None:
        return None

    return (px_to_m(*centre_marker),px_to_m(*centre_circle))

def find_robot(top):
    """ Finds the robot in the lot
    Returns robot pose (x, y, theta) in metres and radians, or None."""

    found = find_marker(top)
    if found is None:
        return None
    (cx, cy), (fx, fy) = found
    theta = np.arctan2(fy-cy, fx-cx)

    x = cx - MARKER_D * np.cos(theta)
    y = cy - MARKER_D * np.sin(theta)
    return float(x), float(y), float(theta)

def read_frame():
    """Take one image from the camera.

    Gives back the image (as OpenCV reads it), or None if the camera gives
    nothing.
    """
    # TODO (Joshua): read the real camera.
    return cv2.imread("data/test_lot_0.png")


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
    
    top = warp_to_lot(frame, CORNERS)
    grid = find_grid(top)
    robot_pose = find_robot(top)
    goal = find_goal(top)
    # TODO: find corners automatically; grid = None when the lot is not seen.
    return grid, robot_pose, goal


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

if __name__ == "__main__":
    frame = cv2.imread("data/test_lot_1.png")
    top = warp_to_lot(frame, CORNERS)
    print(top.shape)                 
    print(find_goal(top))
    print(find_grid(top))
    print(find_robot(top))
    cv2.imshow("top", top)
    cv2.waitKey(0)
