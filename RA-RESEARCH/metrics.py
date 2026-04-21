import numpy as np
import matplotlib.pyplot as plt

def wrap_angle(angle):
    """
    Wrap angle to [-pi, pi]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def trajectory_rmse(true_poses, estimated_poses):
    """
    Compute pose Root-Mean-Square Error for estimated poses vs ground truth poses
    """
    pose_error = true_poses[:, :2] - estimated_poses[:,:2]
    sq_error = np.sum(pose_error**2, axis=1)

    return np.sqrt(np.mean(sq_error))

def heading_rmse(true_poses, estimated_poses):
    """
    Compute heading Root-Mean-Square Error for estimated poses vs ground truth poses
    """
    angle_errors = [
        wrap_angle(th_true - th_est)
        for th_true, th_est in zip(true_poses[:, 2], estimated_poses[:, 2])
    ]
    angle_errors = np.array(angle_errors)
    return np.sqrt(np.mean(angle_errors**2))

def range_rmse(true_ranges, measured_ranges):
    """
    Compute range Root-Mean-Square Error for estimated poses vs ground truth poses
    """
    range_error = measured_ranges[:,2] - true_ranges[:,2]
    return np.sqrt(np.mean(range_error**2))

def connectivity_stats(ranges, num_poses):
    """
    Compute sensor drop off rate, data richness and sparcity of pose graph
    
    """
    pose_ids = ranges[:,0].astype(int)
    unique_poses = np.unique(pose_ids)

    return {
        "num_measurements": len(ranges),
        "avg_per_pose": len(ranges)/num_poses,
        "pose_coverage": len(unique_poses)/num_poses
    }