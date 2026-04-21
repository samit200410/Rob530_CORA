import world
import sensors
import metrics
import numpy as np

def run_parameter_sweep(param_name, values, base_kwargs):
    results_list = []

    for val in values:
        kwargs = base_kwargs.copy()
        kwargs[param_name] = val
        res = run_experiment(**kwargs)

        row = {
            "sweep_param": param_name,
            "sweep_value": val,
        }

        row.update(res["metrics"])
        results_list.append(row)

    return results_list


def run_experiment(
        sigma_dx = 0.02,
        sigma_dy = 0.02,
        sigma_dtheta = 0.01,
        sigma_range = 0.1,
        A = -55.0,
        n = 2.5,
        sigma_rssi = 2.0,
        max_range = 12.0,
        measure_every = 1,
        dropout_prob = 0.0,
        seed = 0,
    ):
    rng = np.random.default_rng(seed)

    # 1. Build World
    world_data = world.make_world()
    true_poses = world_data["poses"]
    landmarks = world_data["landmarks"]

    # 2. Odometry pipeline
    true_odom = world.compute_true_odometry(true_poses)
    noisy_odom = world.add_odometry_noise(
        true_odom,
        sigma_dx=sigma_dx,
        sigma_dy=sigma_dy,
        sigma_dtheta = sigma_dtheta,
        rng = rng,
    )
    dead_reckoned = world.integrate_odometry(noisy_odom, true_poses[0])

    # 3. Range pipelines
    true_ranges = sensors.compute_true_ranges_variable(
        true_poses,
        landmarks,
        max_range=max_range,
        measure_every=measure_every,
        dropout_prob = dropout_prob,
        rng = rng,
    )
    
    ideal_ranges = sensors.add_range_noise(
        true_ranges,
        sigma_r=sigma_range,
        rng=rng,
    )

    rssi_measurements = sensors.simulate_rssi_from_range(
        true_ranges,
        A=A,
        n=n,
        sigma_rssi=sigma_rssi,
        rng=rng,
    )

    rssi_ranges = sensors.rssi_to_range(
        rssi_measurements,
        A=A,
        n=n
    )

    # 4. Metrics
    traj_pos_rmse = metrics.trajectory_rmse(true_poses, dead_reckoned)
    traj_heading_rmse = metrics.heading_rmse(true_poses, dead_reckoned)

    ideal_range_error_rmse = metrics.range_rmse(true_ranges, ideal_ranges)
    rssi_range_error_rmse = metrics.range_rmse(true_ranges, rssi_ranges)
    rssi_heading_rmse = metrics.heading_rmse(true_poses, dead_reckoned)

    conn = metrics.connectivity_stats(
        true_ranges,
        num_poses=len(true_poses)
    )
    # 5. Collect results
    results = {
        "params": {
            "sigma_dx": sigma_dx,
            "sigma_dy": sigma_dy,
            "sigma_dtheta": sigma_dtheta,
            "sigma_range": sigma_range,
            "A": A,
            "n": n,
            "sigma_rssi": sigma_rssi,
            "max_range": max_range,
            "measure_every": measure_every,
            "dropout_prob": dropout_prob,
            "seed": seed,
        },
        "metrics": {
            "trajectory_position_rmse": traj_pos_rmse,
            "trajectory_heading_rmse": traj_heading_rmse,
            "ideal_range_rmse": ideal_range_error_rmse,
            "rssi_range_rmse": rssi_range_error_rmse,
            **conn,
        },
        "data": {
            "world": world_data,
            "true_odom": true_odom,
            "noisy_odom": noisy_odom,
            "dead_reckoned": dead_reckoned,
            "true_ranges": true_ranges,
            "ideal_ranges": ideal_ranges,
            "rssi_measurements": rssi_measurements,
            "rssi_ranges": rssi_ranges,
        },
    }

    return results