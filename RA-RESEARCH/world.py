import numpy as np
import matplotlib.pyplot as plt


def wrap_angle(angle):
    """
    Wrap angle to [-pi, pi]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def generate_square_loop(num_per_side=25):
    """
    Generate a square shaped simulated trajectory
    """
    poses = []

    #Bottom Edge
    xs = np.linspace(2.0, 8.0, num_per_side, endpoint=False)
    ys = np.full_like(xs, 2.0)
    thetas = np.full_like(xs, 0.0)
    poses.extend(zip(xs, ys, thetas))

    #Right Edge
    ys = np.linspace(2.0, 8.0, num_per_side, endpoint=False)
    xs = np.full_like(ys, 8.0)
    thetas = np.full_like(ys, np.pi / 2)
    poses.extend(zip(xs, ys, thetas))

    #Top Edge
    xs = np.linspace(8.0, 2.0, num_per_side, endpoint=False)
    ys = np.full_like(xs, 8.0)
    thetas = np.full_like(xs, np.pi)
    poses.extend(zip(xs, ys, thetas))

    #Left Edge
    ys = np.linspace(8.0, 2.0, num_per_side, endpoint=False)
    xs = np.full_like(ys, 2.0)
    thetas = np.full_like(ys, -np.pi / 2)
    poses.extend(zip(xs, ys, thetas))

    return np.array(poses)

def generate_custom_loop(num_per_segment=10):
    """
    Generate the specified piecewise trajectory with adjustable sampling.

    Original waypoint measurements were given in inches.
    This function converts all coordinates to meters.

    Conversion:
        1 inch = 0.0254 meters

    Returns:
        np.ndarray of shape (N, 3), where each row is [x, y, theta]
    """

    INCH_TO_M = 0.0254

    # Original waypoints in inches
    waypoints_in = [
        (4.0, 10.0),
        (12.0, 10.0),
        (12.0, 0.0),
        (12.0, -10.0),
        (20.0, -10.0),
        (28.0, -10.0),
        (36.0, -10.0),
        (48.0, -10.0),
        (48.0, 0.0),
        (40.0, 10.0),
        (32.0, 10.0),
        (24.0, 10.0),
        (16.0, 10.0),
        (8.0, 10.0),
    ]

    # Convert inches -> meters
    waypoints = [(x * INCH_TO_M, y * INCH_TO_M) for x, y in waypoints_in]

    poses = []

    for i in range(len(waypoints) - 1):
        x0, y0 = waypoints[i]
        x1, y1 = waypoints[i + 1]

        xs = np.linspace(x0, x1, num_per_segment, endpoint=False)
        ys = np.linspace(y0, y1, num_per_segment, endpoint=False)

        theta = np.arctan2(y1 - y0, x1 - x0)
        thetas = np.full_like(xs, theta, dtype=float)

        poses.extend(zip(xs, ys, thetas))

    # Add final waypoint
    x_last0, y_last0 = waypoints[-2]
    x_last1, y_last1 = waypoints[-1]
    theta_last = np.arctan2(y_last1 - y_last0, x_last1 - x_last0)
    poses.append((x_last1, y_last1, theta_last))

    return np.array(poses, dtype=float)

def make_world():
    """
    Generate simulated world with landmarks and bounds
    """
    INCH_TO_M = 0.0254

    waypoints_in = [
        (4.0, 10.0, 0.0),
        (12.0, 10.0, 0.0),
        (12.0, 0.0, 0.0),
        (12.0, -10.0, 0.0),
        (20.0, -10.0, 0.0),
        (28.0, -10.0, 0.0),
        (36.0, -10.0, 0.0),
        (48.0, -10.0, 0.0),
        (48.0, 0.0, 0.0),
        (40.0, 10.0, 0.0),
        (32.0, 10.0, 0.0),
        (24.0, 10.0, 0.0),
        (16.0, 10.0, 0.0),
        (8.0, 10.0, 0.0),
    ]
    waypoints = [(x * INCH_TO_M, y * INCH_TO_M, th) for x, y, th in waypoints_in]

    world = {
        "bounds": {
        "x_min": 0.0,
        "x_max": 60.0 * INCH_TO_M,
        "y_min": -25.0 * INCH_TO_M,
        "y_max": 25.0 * INCH_TO_M,
        },

        "landmarks": np.array([
            [12 * INCH_TO_M,  22 * INCH_TO_M],   # beacon 1 -> L0
            [12 * INCH_TO_M, -22 * INCH_TO_M],   # beacon 2 -> L1
            [45 * INCH_TO_M,  22 * INCH_TO_M],   # beacon 6 -> L2
            [45 * INCH_TO_M, -22 * INCH_TO_M],   # beacon ? -> L3
            [55 * INCH_TO_M,   0 * INCH_TO_M],   # beacon ? -> L4
        ], dtype=float),
        "poses": np.array(waypoints)
    }
    print("POSES: " + str(world["poses"]))
    return world


def compute_true_odometry(poses):
    """
    Compute odometry trajectory based on poses
    """

    odom = []

    for k in range(len(poses) - 1):
        x0,y0,th0 = poses[k]
        x1,y1,th1 = poses[k+1]

        dx_world = x1 - x0
        dy_world = y1 - y0

        c = np.cos(th0)
        s = np.sin(th0)

        dx_local = c*dx_world + s*dy_world
        dy_local = -s*dx_world + c*dy_world
        dtheta = wrap_angle(th1 - th0)

        odom.append([k, k+1, dx_local, dy_local, dtheta])

    return np.array(odom)


def add_odometry_noise(true_odom, sigma_dx=0.02, sigma_dy=0.02, sigma_dtheta=0.01, rng=None):
    """
    Add gaussian noise to odometry true odometry data
    """
    if rng is None:
        rng = np.random.default_rng(0)

    noisy_odom = true_odom.copy().astype(float)
    noisy_odom[:, 2] += rng.normal(0.0, sigma_dx, size=len(true_odom))
    noisy_odom[:, 3] += rng.normal(0.0, sigma_dy, size=len(true_odom))
    noisy_odom[:, 4] += rng.normal(0.0, sigma_dtheta, size=len(true_odom))
    noisy_odom[:, 4] = np.array([wrap_angle(a) for a in noisy_odom[:, 4]])

    return noisy_odom


def integrate_odometry(noisy_odom, x0):
    """
    Integrate local frame odometry forward to produce odometry trajectory
    """
    poses_dr = [np.array(x0, dtype=float)]

    for row in noisy_odom:
        _, _, dx_local, dy_local, dtheta = row

        x, y, th = poses_dr[-1]

        c = np.cos(th)
        s = np.sin(th)

        dx_world = c * dx_local - s*dy_local
        dy_world = s*dx_local + c*dy_local

        x_new = x + dx_world
        y_new = y + dy_world

        th_new = wrap_angle(th + dtheta)

        poses_dr.append(np.array([x_new, y_new, th_new]))
    return np.array(poses_dr)