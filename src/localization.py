"""Localization block (the filter). Owner: Joshua.

Gives the best estimate of where the robot is. Every other block uses this
estimate, never the camera directly.

For now the function just trusts the camera. The extended Kalman filter comes
last and goes inside the same function; keep its name, its inputs and what it gives
back (see the README).
"""


def update(cam_pose, wheel_speeds, dt):
    """Estimate where the robot is.

    cam_pose      (x, y, theta) seen by the camera, or None if the robot is
                  not visible.
    wheel_speeds  (left, right) speeds measured by the Thymio, in Thymio units.
    dt            time since the last call, in seconds.

    Gives back two values:
        pose  (x, y, theta) best estimate, or None when the position is unknown.
        cov   3 x 3 array, uncertainty of (x, y, theta). None until the filter
              exists.
    """
    # TODO (Joshua): extended Kalman filter. Predict with wheel_speeds, correct
    # with cam_pose when it is not None. Noise values measured on our own
    # recordings in data/, not default ones.
    return cam_pose, None
