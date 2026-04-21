import numpy as np
import matplotlib.pyplot as plt

def plot_range_measurements(true_ranges, ideal_ranges, rssi_ranges):
    plt.figure(figsize=(8, 5))
    plt.plot(true_ranges[:, 2], label="True range")
    plt.plot(ideal_ranges[:, 2], '.', alpha=0.6, label="Ideal noisy range")
    plt.plot(rssi_ranges[:, 2], '.', alpha=0.6, label="RSSI-derived pseudo-range")
    plt.xlabel("Measurement index")
    plt.ylabel("Range [m]")
    plt.title("Range measurement comparison")
    plt.grid(True)
    plt.legend()
    plt.show()

    
def plot_range_measurements_per_landmark(true_ranges, ideal_ranges, rssi_ranges, num_landmarks):

    cols = 2
    rows = int(np.ceil(num_landmarks / cols))

    fig, axes = plt.subplots(rows, cols, figsize=(12, 4 * rows), sharex=True)
    axes = np.array(axes).reshape(-1)

    for j in range(num_landmarks):
        ax = axes[j]

        # Select rows for landmark j
        tr = true_ranges[true_ranges[:, 1].astype(int) == j]
        ir = ideal_ranges[ideal_ranges[:, 1].astype(int) == j]
        rr = rssi_ranges[rssi_ranges[:, 1].astype(int) == j]

        # x = pose index
        ax.plot(tr[:, 0], tr[:, 2], label="True range", linewidth=2)
        ax.plot(ir[:, 0], ir[:, 2], '.', alpha=0.6, label="Ideal noisy")
        ax.plot(rr[:, 0], rr[:, 2], '.', alpha=0.6, label="RSSI-derived")

        ax.set_title(f"Landmark {j}")
        #ax.set_xlabel("Pose index")
        ax.set_ylabel("Range [m]")
        ax.grid(True)

    # Hide unused axes
    for k in range(num_landmarks, len(axes)):
        axes[k].axis("off")

    # One shared legend
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right")

    fig.suptitle("Range Measurements Per Landmark", fontsize=14)
    plt.tight_layout()
    plt.show()

def plot_rssi_vs_distance(true_ranges, rssi_measurements):
    plt.figure(figsize=(6, 5))
    plt.scatter(true_ranges[:, 2], rssi_measurements[:, 2], s=10)
    plt.xlabel("True distance [m]")
    plt.ylabel("RSSI [dBm]")
    plt.title("RSSI vs true distance")
    plt.grid(True)
    plt.show()


def plot_world_and_odometry(world, dead_reckoned):
    poses = world["poses"]
    landmarks = world["landmarks"]
    bounds = world["bounds"]

    plt.figure(figsize=(7, 7))
    plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='o',color='red', s=150, label='Landmarks')
    plt.plot(poses[:, 0], poses[:, 1], label='Ground Truth')
    plt.plot(dead_reckoned[:, 0], dead_reckoned[:, 1], '--', label='Dead Reckoning')
    plt.scatter(poses[0, 0], poses[0, 1], marker='s', s=80, label='Start')

    plt.xlim(bounds["x_min"], bounds["x_max"])
    plt.ylim(bounds["y_min"], bounds["y_max"])
    plt.gca().set_aspect('equal')
    plt.grid(True)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Ground Truth vs Noisy Odometry")
    plt.legend()
    plt.show()

def plot_range_rmse_vs_sigma(rmse, sigmas):
    plt.figure(figsize=(10, 10))
    plt.scatter(sigmas[:], rmse[:], s=20)
    plt.xlabel("RSSI noise sigma")
    plt.ylabel("RMSE")
    plt.title("RMSE vs RSSI noise")
    plt.grid(True)
    plt.show()


def plot_sweep(results_list, y_key, title=None):
    x = [row["sweep_value"] for row in results_list]
    y = [row[y_key] for row in results_list]

    plt.figure(figsize=(7,5))
    plt.plot(x,y,'o-')
    plt.xlabel(results_list[0]["sweep_param"])
    plt.ylabel(y_key)
    plt.title(title if title is not None else f"{y_key} vs {results_list[0]['sweep_param']}")
    plt.grid(True)
    plt.show()


def plot_double_sweep(results_list, results_list2, y_key, y_key2, title=None):
    x = [row["sweep_value"] for row in results_list]
    y = [row[y_key] for row in results_list]

    x2 = [row["sweep_value"] for row in results_list2]
    y2 = [row[y_key2] for row in results_list2]

    plt.figure(figsize=(7,5))
    plt.plot(x,y,'o-', label=y_key)
    plt.plot(x2,y2,'o-', label=y_key2)
    plt.xlabel(results_list[0]["sweep_param"])
    plt.ylabel(y_key)
    plt.title(title if title is not None else f"{y_key} vs {results_list[0]['sweep_param']}")
    plt.grid(True)
    plt.legend()
    plt.show()
