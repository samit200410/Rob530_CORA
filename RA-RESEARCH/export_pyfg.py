import numpy as np
import matplotlib.pyplot as plt

def save_sim_to_pyfg(
    filepath,
    world,
    noisy_odom,
    range_measurements,
    odom_cov=None,
):
    """
    Export a simple single-robot 2D factor graph to PyFactorGraph .pyfg format.

    Parameters
    ----------
    filepath : str
        Output .pyfg file path
    world : dict
        Must contain:
          - world["poses"] as Nx3 array of ground-truth poses [x, y, theta]
          - world["landmarks"] as Mx2 array of landmark positions [x, y]
    noisy_odom : np.ndarray
        Kx5 array with rows [k0, k1, dx, dy, dtheta]
    range_measurements : np.ndarray
        Jx3 array with rows [pose_idx, landmark_idx, dist]
    odom_cov : np.ndarray or None
        3x3 covariance matrix for odometry factors.
        If None, uses a default diagonal covariance.
    """
    poses = world["poses"]
    landmarks = world["landmarks"]

    if odom_cov is None:
        odom_cov = np.diag([0.02**2, 0.02**2, 0.01**2])

    # Upper-triangular entries in column-major order for 3x3 symmetric matrix
    # [c00, c01, c11, c02, c12, c22]
    odom_noise_str = (
        f"{odom_cov[0,0]:.9f} "
        f"{odom_cov[0,1]:.9f} "
        f"{odom_cov[1,1]:.9f} "
        f"{odom_cov[0,2]:.9f} "
        f"{odom_cov[1,2]:.9f} "
        f"{odom_cov[2,2]:.9f}"
    )

    with open(filepath, "w") as f:
        # 1) Pose variables: VERTEX_SE2 timestamp name x y theta
        for k, (x, y, theta) in enumerate(poses):
            pose_name = f"A{k}"
            timestamp = float(k)
            f.write(
                f"VERTEX_SE2 {timestamp:.9f} {pose_name} "
                f"{x:.9f} {y:.9f} {theta:.7f}\n"
            )

        # 2) Landmark variables: VERTEX_XY name x y
        for j, (lx, ly) in enumerate(landmarks):
            landmark_name = f"L{j}"
            f.write(
                f"VERTEX_XY {landmark_name} "
                f"{lx:.9f} {ly:.9f}\n"
            )

        # 3) Odometry edges: EDGE_SE2 timestamp base_pose to_pose dx dy dtheta cov...
        for row in noisy_odom:
            k0 = int(row[0])
            k1 = int(row[1])
            dx = float(row[2])
            dy = float(row[3])
            dtheta = float(row[4])

            timestamp = float(k1)
            base_pose = f"A{k0}"
            to_pose = f"A{k1}"

            f.write(
                f"EDGE_SE2 {timestamp:.9f} {base_pose} {to_pose} "
                f"{dx:.9f} {dy:.9f} {dtheta:.7f} "
                f"{odom_noise_str}\n"
            )

        # 4) Range edges: EDGE_RANGE timestamp first_key second_key dist variance
        # Here we use a fixed variance for all range measurements
        range_variance = 0.10**2

        for row in range_measurements:
            pose_idx = int(row[0])
            landmark_idx = int(row[1])
            dist = float(row[2])

            timestamp = float(pose_idx)
            first_key = f"A{pose_idx}"
            second_key = f"L{landmark_idx}"

            f.write(
                f"EDGE_RANGE {timestamp:.9f} {first_key} {second_key} "
                f"{dist:.9f} {range_variance:.9f}\n"
            )
