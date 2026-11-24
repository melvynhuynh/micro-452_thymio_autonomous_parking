"""Vision block. Owner: Joshua.

From a camera image, find the cars, the robot and the target.

Detection works on the real camera, to_pixels and threshold tuning still to do
"""

import atexit
import time

import numpy as np
import cv2

from src import common

PX_PER_M = 1000
LOT_W_PX = int(round(common.COLS * common.CELL * PX_PER_M))
LOT_H_PX = int(round(common.ROWS * common.CELL * PX_PER_M))
CELL_PX = int(round(common.CELL * PX_PER_M))
CAR_THRESHOLD = 0.1
BLACK_V_MAX=90  #Threshold for black boxes, 30-90 maybe depends on lighting
RED1_LO, RED1_HI   = (172, 120, 70), (180, 255, 255)
RED2_LO, RED2_HI   = (0, 120, 70),   (10, 255, 255)
GREEN_LO, GREEN_HI = (35, 50, 70),   (85, 255, 255)
DOT_LO, DOT_HI     = (145, 80, 70),  (170, 255, 255)

MARKER_D    = 0.0     # marker centre ahead of the wheel axis, m

MARKER_ANGLE = 0.0        # rad: heading error of the marker on the robot (subtract) #TODO: Add the real value after measuring, currently 0 for testing
HEIGHT_K = 0.0            # stretch from the marker height (0 = no correction)
CAM_FLOOR = (0.0, 0.0)    # m: point of the lot straight below the camera

#CORNERS = [(50, 50), (1250, 50), (1250, 850), (50, 850)]

last_corners = None

USE_CAMERA = True
HIDE_CAMERA = None   # e.g. (5, 12): from 5 s to 12 s after the first image, act as if
                     # the camera sees nothing (filter test). None = off.
_first_read = None   # time of the first read_frame() call
_failed_reads = 0    # camera reads that failed in a row

CAP = cv2.VideoCapture(1, cv2.CAP_DSHOW) if USE_CAMERA else None # 0 = laptop webcam, 1 = usually the USB camera
if CAP is not None and not CAP.isOpened():
    print("Camera could not be opened (busy, or wrong number)")
    CAP = None
if CAP is not None:
    atexit.register(CAP.release)   # give the camera back to Windows when the program ends
    for _ in range(30):       # about 1 s: let auto-exposure settle
        CAP.read()

def detect_corners(frame):
    """Detect the inner corners of the black border in the camera image.
    frame: image from read_frame(), or None.
    Gives back four points (u, v) in pixels, ordered top-left, top-right,
    bottom-right, bottom-left, or None if the corners cannot be found.
    """
    if frame is None:
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:            # no contours at all
        return None

    # largest hole = inside of the black border
    best = None
    best_area = 0
    for i in range(len(contours)):
        if hierarchy[0][i][3] == -1:     # no parent: outer edge, not a hole
            continue
        area = cv2.contourArea(contours[i])
        if area > best_area:
            best_area = area
            best = contours[i]

    if best is None:
        return None
    # the lot must fill a good part of the image, otherwise it's not the border
    h, w = frame.shape[:2]
    if best_area < 0.2 * h * w:
        return None
    # simplify the outline to its corners
    hull = cv2.convexHull(best)
    eps = 0.02 * cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, eps, True)
    if len(approx) != 4:
        return None
    pts = approx.reshape(4, 2)

    # order: top-left, top-right, bottom-right, bottom-left
    s = pts.sum(axis=1)              # u + v
    d = pts[:, 1] - pts[:, 0]        # v - u
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(d)]
    bl = pts[np.argmax(d)]

    return [tuple(tl), tuple(tr), tuple(br), tuple(bl)]

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

def clean(mask):
    """Remove small specks of noise from a mask."""
    kernel = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

def find_goal(top):
    """Find the goal in the image."""
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array(RED1_LO)
    upper_red1 = np.array(RED1_HI)
    lower_red2 = np.array(RED2_LO)
    upper_red2 = np.array(RED2_HI)
    mask1=cv2.inRange(hsv, lower_red1, upper_red1)
    mask2=cv2.inRange(hsv, lower_red2, upper_red2)

    mask = cv2.bitwise_or(mask1, mask2)
    mask=clean(mask)
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
    mask=clean(mask)
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

    lower_green = np.array(GREEN_LO) 
    upper_green = np.array(GREEN_HI)
    lower_magenta = np.array(DOT_LO)
    upper_magenta = np.array(DOT_HI)
    mask_green=cv2.inRange(hsv, lower_green, upper_green)
    mask_green=clean(mask_green)
    mask_magenta=cv2.inRange(hsv, lower_magenta, upper_magenta)
    #Maybe clean the magenta mask too, but it is small and might be removed by cleaning

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
    theta = np.arctan2(fy-cy, fx-cx) - MARKER_ANGLE
    theta = (theta + np.pi) % (2 * np.pi) - np.pi     # back to [-pi, pi]

    cx = CAM_FLOOR[0] + (cx - CAM_FLOOR[0]) / (1 + HEIGHT_K)
    cy = CAM_FLOOR[1] + (cy - CAM_FLOOR[1]) / (1 + HEIGHT_K)

    x = cx - MARKER_D * np.cos(theta)
    y = cy - MARKER_D * np.sin(theta)
    return float(x), float(y), float(theta)

def read_frame():
    """Take one image from the camera.

    Gives back the image (as OpenCV reads it), or None if the camera gives
    nothing.
    """
    global _failed_reads, _first_read
    if not USE_CAMERA:
        return cv2.imread("data/captured_image3.png")
    if CAP is None:
        raise SystemExit("No camera: camera 1 could not be opened (busy, or wrong number)")
    ok, frame = CAP.read()          # still read while hidden: no old images pile up
    if not ok:
        _failed_reads += 1
        if _failed_reads % 10 == 1:     # 1st, 11th, 21st...: don't flood the terminal
            print(f"Camera read failed ({_failed_reads} in a row)")
        return None
    _failed_reads = 0
    if _first_read is None:
        _first_read = time.time()
    if HIDE_CAMERA is not None and HIDE_CAMERA[0] <= time.time() - _first_read < HIDE_CAMERA[1]:
        return None                 # camera "hidden"
    return frame


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
    global last_corners 
    if frame is None:
        return None, None, None

    corners = detect_corners(frame)
    if corners is not None:
        last_corners = corners
    if last_corners is None:          # lot never seen yet
        return None, None, None
    
    top = warp_to_lot(frame, last_corners)
    grid = find_grid(top) if corners is not None else None
    robot_pose = find_robot(top)
    goal = find_goal(top)
    return grid, robot_pose, goal


def to_pixels(x, y):
    """Where a point of the lot is in the camera image (to confirm).

    The live display uses it to draw on the image.

    x, y  position in metres.

    Gives back (u, v): column and row of the pixel, as OpenCV drawing
    functions expect. None as long as the lot has never been seen.
    """
    if last_corners is None:                 # lot never seen
        return None
    # warped image -> camera image (dst and src swapped compared to warp_to_lot)
    lot = np.float32([[0, 0], [LOT_W_PX, 0], [LOT_W_PX, LOT_H_PX], [0, LOT_H_PX]])
    M_inv = cv2.getPerspectiveTransform(lot, np.float32(last_corners))
    # metres -> warped pixels (inverse of px_to_m)
    pt = np.float32([[[x * PX_PER_M, LOT_H_PX - y * PX_PER_M]]])
    u, v = cv2.perspectiveTransform(pt, M_inv)[0][0]
    return int(round(u)), int(round(v))

if __name__ == "__main__":
    frame = cv2.imread("data/captured_image3.png")
    corners = detect_corners(frame)
    if corners is None:
        print("Corners not found")
        cv2.imshow("frame", frame)
        cv2.waitKey(0)
        raise SystemExit
    print(corners)

    # draw the corners on a copy, numbered 0-3
    shown = frame.copy()
    for k, p in enumerate(corners):
        cv2.circle(shown, (int(p[0]), int(p[1])), 6, (0, 0, 255), -1)
        cv2.putText(shown, str(k), (int(p[0]) + 8, int(p[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imshow("corners", shown)

    top = warp_to_lot(frame, corners)   # clean frame, no dots
    print("Top shape:", top.shape)
    print("Goal:", find_goal(top))
    print(find_grid(top))
    print("Robot:", find_robot(top))
    cv2.imshow("top", top)
    cv2.waitKey(0)
