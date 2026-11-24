# Recordings

What each file in this folder is: measurements and test runs, kept for the
report. File names end with the date and time of the run
(`YYYYMMDD_HHMMSS`). All positions are in metres, in the frame of the
README (start at (0.10, 0.10), facing +x).

## Calibration of the robot (17 Nov), `tests/calibrate.py`

| File | What |
|---|---|
| `calibration_results.csv` | One line per run: kind, speed, measured distance or turns, result, wheel-speed noise |
| `calibration_straight_*.csv` | Straight runs at 100, 100, 150, 200 units: asked and measured wheel speeds every 0.1 s. Give `THYMIO_TO_MS` = 0.000299 |
| `calibration_spin_*.csv` | Spins at 100 and 150 units. Give `WHEEL_BASE` = 0.0964 m |

## Camera noise, `tests/camera_noise.py`

The robot stands still at 10 known poses (4 corner cells and cell (2, 2),
facing 0 and 90 degrees); 100 camera poses at each. One line per frame: true
pose and camera pose. `python -m tests.camera_noise analyse <file>` gives the
bias, the jitter and Q.

| File | Setup | Result |
|---|---|---|
| `camera_noise_20261124_180851.csv` | First trial: markers only placed on the Thymio, not taped | Jitter about 1 mm and 0.3 deg, but bias up to 4 cm and 6.6 deg: marker 28 mm behind the wheel axis, turned by -5 deg, and a 3 % stretch from the marker height. Not used for Q |

## Simulations (no robot)

| File | What |
|---|---|
| `sim_go_to.png` | `tests/sim_go_to.py`: `go_to` on the shared example path, perfect and realistic (noise, delay, slow motors) |
| `sim_main.png` | `tests/test_main.py`: the `main.py` scenarios in the fake world |
| `sweep_avoid.csv` | `tests/sweep_avoid.py`: a 15 cm box sticking 0 to 8 cm into the robot's path, right or left, at four places. Parked or STUCK, gap to the box, distance off the path |

## `go_to` on the robot, pose from the wheel speeds (17 Nov), `tests/thymio_drive.py`

| File | What |
|---|---|
| `thymio_drive_20261117_173439` | Shared example path. Loop at 0.21 s per turn: `set_speed` still waited for the robot's reply |
| `thymio_drive_20261117_173957` | Same, after the fix: 0.107 s per turn |

## `main.py` on the robot, pose from the wheel speeds, `tests/thymio_main.py`

The fake camera reports the shared example lot unless the run says "empty
lot" (no cars, target at (0.90, 0.10), 80 cm straight ahead). The csv has the
pose and the front sensors every 0.1 s (only their maximum before 18 Nov
22:00). Several versions of `main.py` were tried; the column says which.

| File | Box | `main.py` version | Result |
|---|---|---|---|
| `thymio_main_20261117_175427` | none | four states, no soft start | Parked; drifted right at the start (wheel-speed misreading at the start-up jolt) |
| `thymio_main_20261117_181009` | none | with the soft start | Parked; heading at the start within -0.5 to +1.1 degrees |
| `thymio_main_20261117_185317` | in the bottom row | blocked cell + new plan (Laurent's avoid) | Turned back and took another lane: the U-turn |
| `thymio_main_20261117_190307` | in the bottom row | go around, waypoints near the box skipped | Parked, but cut the corner diagonally through a bay with a car on the map |
| `thymio_main_20261118_212319` | none, empty lot | carrot + line check + detour (not kept) | Parked straight |
| `thymio_main_20261118_212458` | right, sticking out more | same | Detour through the row above, starting back from the waypoint behind |
| `thymio_main_20261118_212600` | right, sticking out 2 cm | same | Same detour, for a box it could have passed |
| `thymio_main_20261118_214131` | right, near the end | MVP: carrot + 5 cm sidestep | Passed and parked, at an angle (box just before the target) |
| `thymio_main_20261118_214227` | right, closer, sticking out more | same | Four avoidances while moving on, then STUCK |
| `thymio_main_20261118_215625_R3.5_40` | right, about 40 cm ahead | MVP + growing sidestep, progress-based STUCK | Stepped 4 cm left, passed, parked; sharp turn at the end of the sidestep |
| `thymio_main_20261118_215732_L3.5_40` | left, about 40 cm ahead | same | Stepped 4 cm right, passed, parked |

Run labels since 18 Nov: `R`/`L` = box on the right/left, then the distance
from the robot's line to the box face in cm, then how far ahead the box is,
in cm.
