"""
Tool program to generate a image of a test lot, for testing the vision code.
The image is saved in data/

run with : python -m tools.make_test_lot
"""

import cv2
import numpy as np

from src import common

PX_PER_M = 1000 #1 pixel is 1 mm, so 1000 pixels is 1 m
MARGIN = 50 # pixels of margin around the lot
CAR = 0.15 # metres, side of a car
GOAL = 0.10 # metres, side of the goal

THYMIO_R    = 0.055   # half the width = radius of the round front, m
THYMIO_BACK = 0.057   # flat back edge, this far behind the wheel axis, m
MARKER_L    = 0.07    # marker length along the heading, m
MARKER_W    = 0.05    # marker width, m
MARKER_D    = 0.0     # marker centre ahead of the wheel axis, m
DOT_D       = 0.025   # dot ahead of the marker centre, m

def m_to_px(x, y):
    u = MARGIN + x * PX_PER_M   # pixel column
    v = height - MARGIN - y * PX_PER_M   # pixel row
    return int(round(u)), int(round(v))

def robot_to_px(pose, a, b):
    """Point (a, b) in the robot's own frame (a forward, b left) -> pixel."""
    x, y, theta = pose
    c, s = np.cos(theta), np.sin(theta)
    return m_to_px(x + c * a - s * b, y + s * a + c * b)

def draw_robot(img, pose):
    # body: flat back corners, then a half circle around the front
    outline = [(-THYMIO_BACK, THYMIO_R), (-THYMIO_BACK, -THYMIO_R)]
    for ang in np.linspace(-np.pi / 2, np.pi / 2, 30):
        outline.append((THYMIO_R * np.cos(ang), THYMIO_R * np.sin(ang)))
    body = np.int32([robot_to_px(pose, a, b) for a, b in outline])
    cv2.fillPoly(img, [body], (230, 230, 230))           # light grey body
    cv2.polylines(img, [body], True, (80, 80, 80), 2)    # dark outline

    # green marker on top
    hl, hw = MARKER_L / 2, MARKER_W / 2
    corners = [(MARKER_D - hl, -hw), (MARKER_D + hl, -hw),
               (MARKER_D + hl,  hw), (MARKER_D - hl,  hw)]
    marker = np.int32([robot_to_px(pose, a, b) for a, b in corners])
    cv2.fillPoly(img, [marker], (0, 255, 0))

    # magenta dot towards the front
    dot = robot_to_px(pose, MARKER_D + DOT_D, 0)
    cv2.circle(img, dot, 10, (255, 0, 255), -1)

(rows, cols) = common.EXAMPLE_GRID.shape
height = int(round(rows*common.CELL*PX_PER_M+2*MARGIN))
width = int(round(cols*common.CELL*PX_PER_M+2*MARGIN))
print(f"Generating test lot image of size {width}x{height} pixels")

img=np.full((height, width, 3), 255, dtype=np.uint8)

thickness=20
u1, v1 = m_to_px(0, 0)
u1=u1-thickness//2
v1=v1+thickness//2
u2, v2 = m_to_px(cols*common.CELL, rows*common.CELL)
u2=u2+thickness//2
v2=v2-thickness//2
pt1 = (u1, v1)
pt2 = (u2, v2)
color=(0, 0, 0) # black
cv2.rectangle(img, pt1, pt2, color, thickness) #Draw border

for k in range(cols + 1): #Draw grid vertical lines
    x = k * common.CELL
    pt1 = m_to_px(x, 0)
    pt2 = m_to_px(x, rows * common.CELL)
    cv2.line(img, pt1, pt2, (255, 0, 0), 2)
for k in range(rows + 1): #Draw grid horizontal lines
    y = k * common.CELL
    pt1 = m_to_px(0, y)
    pt2 = m_to_px(cols * common.CELL, y)
    cv2.line(img, pt1, pt2, (255, 0, 0), 2)

for row in range(rows): #Draw cars in the lot
    for col in range(cols):
        if common.EXAMPLE_GRID[row, col] == 1:
            x, y = common.cell_to_xy(row, col)
            pt1=m_to_px(x-CAR/2, y-CAR/2)
            pt2=m_to_px(x+CAR/2, y+CAR/2)
            cv2.rectangle(img, pt1, pt2, color, -1)


pt1 = m_to_px(common.EXAMPLE_GOAL[0]-GOAL/2, common.EXAMPLE_GOAL[1]-GOAL/2)
pt2 = m_to_px(common.EXAMPLE_GOAL[0] + GOAL/2, common.EXAMPLE_GOAL[1] + GOAL/2)
cv2.rectangle(img, pt1, pt2, (0, 0, 255), -1) #draw goal


poses = [(0.10, 0.10, np.pi/4), (0.30, 0.10, np.pi/2), (1.10, 0.10, np.radians(135))]
for i, pose in enumerate(poses):
    frame = img.copy()          # fresh lot without the previous robot
    draw_robot(frame, pose)
    cv2.imwrite(f"data/test_lot_{i}.png", frame)
    print(i, pose)


#Explanation of the poses in data/:
#test_lot (0.1, 0.1, 0.0)
#test_lot_0 (0.1, 0.1, 0.7853981633974483)
#test_lot_1 (0.3, 0.1, 1.5707963267948966)
#test_lot_2 (1.1, 0.1, np.float64(2.356194490192345))