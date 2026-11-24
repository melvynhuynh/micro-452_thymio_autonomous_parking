"""How long one camera read takes. Owner: Joshua.

Prints the time of every read. Pull the cable while it runs: if the times jump
to 500 ms or more, a missing camera slows the main loop down (the weaving seen
on 24 Nov).

    python tools/camera_timing.py        (stop with Ctrl+C)
"""

import time

import cv2

CAMERA = 1   # same number as in src/vision.py

cap = cv2.VideoCapture(CAMERA, cv2.CAP_DSHOW)
print("Camera opened:", cap.isOpened())
try:
    while True:
        start = time.time()
        ok, _ = cap.read()
        print(f"ok={ok}  {(time.time() - start) * 1000:6.0f} ms")
        time.sleep(0.05)      # only so a failing camera doesn't flood the terminal
except KeyboardInterrupt:
    pass
finally:
    cap.release()
