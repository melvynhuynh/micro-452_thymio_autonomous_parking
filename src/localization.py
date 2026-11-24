"""Localization block (the filter). Owner: Joshua.

Gives the best estimate of where the robot is. Every other block uses this
estimate, never the camera directly.

Extended Kalman filter on (x, y, theta). Predicts with the wheel speeds and
corrects with the camera pose when the robot is visible; resets to the camera
if the robot gets moved by hand.
"""
import numpy as np

from src import common

mu = None   # best estimate of (x, y, theta), or None when unknown
Sigma = None  # uncertainty of (x, y, theta), or None when unknown
THYMIO_TO_MS = common.THYMIO_TO_MS
WHEEL_BASE = common.WHEEL_BASE

KIDNAP_DIST = 0.10    # m: camera this far from the prediction -> robot was moved
KIDNAP_ANGLE = 1.0    # rad (~57 deg): same for the heading

#R = np.diag([1e-5, 1e-5, 1e-4]) #TODO: Use real motion noise values after measuring, 
#currently: per step: ~3 mm, ~3 mm, ~0.01 rad std

SPEED_NOISE = 5        # Thymio units: spread of the measured wheel speeds (data/calibration_results.csv)
SCALE_ERROR = 0.03     # 3 %: spread of THYMIO_TO_MS between calibration runs

Q = np.diag([0.003**2, 0.003**2, 0.05**2])#TODO: Use real noise values after measuring,
#Currently: std: 3 mm, 3 mm, 0.05 rad

def wrap(angle):
    """Wrap an angle in radians to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def process_noise(theta, vl, vr, dt):
    """Noise added by one prediction step (R), from the wheel-speed noise."""
    # 1. sigma_l², sigma_r²  (step 1, point 2 of the guide)
    sigma_l2=(SPEED_NOISE*THYMIO_TO_MS)**2+(SCALE_ERROR*vl)**2
    sigma_r2=(SPEED_NOISE*THYMIO_TO_MS)**2+(SCALE_ERROR*vr)**2
    # 2. M = 2x2 matrix
    M = np.array([[sigma_l2, 0], [0, sigma_r2]])
    # 3. V = 3x2 matrix
    V = dt * np.array([[np.cos(theta)/2, np.cos(theta)/2],
                  [np.sin(theta)/2, np.sin(theta)/2],
                  [-1/WHEEL_BASE, 1/WHEEL_BASE]]) 
    # 4. return V @ M @ V.T
    return V @ M @ V.T

def predict(mu, Sigma, wheel_speeds, dt):
    vl=wheel_speeds[0]*THYMIO_TO_MS 
    vr=wheel_speeds[1]*THYMIO_TO_MS
    #Motion model:
    v=(vl+vr)/2
    w=(vr-vl)/WHEEL_BASE 
    x_new=mu[0]+v*np.cos(mu[2])*dt
    y_new=mu[1]+v*np.sin(mu[2])*dt
    theta_new=mu[2]+w*dt
    theta_new = wrap(theta_new)

    G = np.array([[1, 0, -np.sin(mu[2])*v*dt],
              [0, 1, np.cos(mu[2])*v*dt],
              [0, 0, 1]])

    R=process_noise(mu[2],vl,vr,dt)
    Sigma=G@Sigma@G.T+R
    mu=np.array([x_new,y_new,theta_new])
    
    return mu, Sigma

def correct(mu, Sigma, z):
    H=np.eye(3)
    K=Sigma@H.T@np.linalg.inv(H@Sigma@H.T+Q)
    innovation = np.array(z) - H @ mu
    innovation[2] = wrap(innovation[2])
    mu = mu + K @ innovation
    mu[2] = wrap(mu[2]) 
    Sigma=(np.eye(3)-K@H)@Sigma
    return mu, Sigma

def update(cam_pose, wheel_speeds, dt):
    """Estimate where the robot is.

    cam_pose      (x, y, theta) seen by the camera, or None if the robot is
                  not visible.
    wheel_speeds  (left, right) speeds measured by the Thymio, in Thymio units.
    dt            time since the last call, in seconds.

    Gives back two values:
        pose  (x, y, theta) best estimate, or None when the position is unknown.
        cov   3 x 3 array, uncertainty of (x, y, theta). None until the robot has been seen.
    """
    global mu, Sigma
    if mu is None:   #First call everything is None so needs to initalize mu and Sigma
        if cam_pose is None:
            return None, None           # never seen the robot yet
        mu = np.array(cam_pose, dtype=float)
        Sigma = Q.copy()                # start as uncertain as one camera measurement
        return (float(mu[0]), float(mu[1]), float(mu[2])), Sigma
    
    #Prediction step:
    mu,Sigma=predict(mu,Sigma,wheel_speeds,dt)

    #Update step:
    # Correction step, or reset if the robot was moved by hand
    if cam_pose is not None:
        dist = np.hypot(cam_pose[0] - mu[0], cam_pose[1] - mu[1])
        dtheta = abs(wrap(cam_pose[2] - mu[2]))
        if dist > KIDNAP_DIST or dtheta > KIDNAP_ANGLE:
            mu = np.array(cam_pose, dtype=float)     # jump to the camera
            Sigma = Q.copy()
        else:
            mu, Sigma = correct(mu, Sigma, cam_pose)
    # TODO (Joshua): Fix noise to use values measured on our own recordings in data/, not default ones.
    # Use common values for THYMIO_TO_MS and WHEEL_BASE when available
    return (float(mu[0]), float(mu[1]), float(mu[2])), Sigma



#Testing the filter

if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)

    def show(label, pose, cov):
        if pose is None:
            print(f"{label:28s} pose=None")
            return
        std = np.sqrt(np.diag(cov))          # standard deviations: x, y (m), theta (rad)
        print(f"{label:28s} pose=({pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:+.4f})"
              f"  std=({std[0]*1000:.1f} mm, {std[1]*1000:.1f} mm, {std[2]:.3f} rad)")

    dt = 0.1

    # 1. No camera yet -> filter cannot start
    pose, cov = update(None, (0, 0), dt)
    show("1. never seen", pose, cov)

    # 2. First camera pose -> filter starts there
    pose, cov = update((0.10, 0.10, 0.0), (0, 0), dt)
    show("2. init from camera", pose, cov)

    # 3. Drive straight 1 s, camera hidden: x +4 mm per step, std grows
    for k in range(10):
        pose, cov = update(None, (100, 100), dt)
    show("3. 1 s straight, no camera", pose, cov)     # expect x ~ 0.14

    # 4. Camera comes back: estimate jumps towards it, std shrinks
    pose, cov = update((0.145, 0.10, 0.0), (100, 100), dt)
    show("4. camera back", pose, cov)

    # 5. Turn in place, no camera: theta changes, x and y stay
    for k in range(10):
        pose, cov = update(None, (-100, 100), dt)
    show("5. 1 s turning left", pose, cov)          # expect theta ~ +0.84

    # 6. Angle wrap: estimate near +pi, camera says just past -pi
    mu = np.array([0.5, 0.5, 3.12])
    Sigma = Q.copy()
    pose, cov = update((0.5, 0.5, -3.13), (0, 0), dt)
    show("6. wrap across +-pi", pose, cov)          # expect theta ~ +-3.13, NOT ~0

    # 7. Jacobian check: G against a numerical derivative
    m0 = np.array([0.3, 0.2, 0.7])
    eps = 1e-6
    m1, _ = predict(m0, np.zeros((3, 3)), (150, 80), dt)
    m2, _ = predict(m0 + [0, 0, eps], np.zeros((3, 3)), (150, 80), dt)
    numeric = (m2 - m1) / eps
    v = (150 + 80) / 2 * THYMIO_TO_MS
    analytic = np.array([-np.sin(m0[2]) * v * dt, np.cos(m0[2]) * v * dt, 1])
    print("7. G theta column  numeric:", numeric, " analytic:", analytic)

    # 8. Kidnapping: camera suddenly 40 cm away -> jump there
    mu = np.array([0.10, 0.10, 0.0]); Sigma = Q.copy()
    pose, cov = update((0.50, 0.30, 1.57), (0, 0), dt)
    show("8. kidnapped", pose, cov)        # expect exactly (0.50, 0.30, 1.57)