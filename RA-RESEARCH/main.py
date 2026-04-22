# main.py


from config import BASE_KWARGS, SIGMA_RSSI_VALUES, SIGMA_RANGE_VALUES, MEASURE_EVERY_VALUES
import numpy as np
import experiments
import sensors
import plots
import metrics
import export_pyfg
def build_esp32_ranges():
    raw = {
        0:  {1: [-46.81, -46.85, -46.88], 4: [-55.34, -54.87, -55.50], 6: [-65.26, -64.21, -65.56], 8: [-60.54, -60.43, -59.95], 11: [-53.95, -53.96, -54.97]},
        1:  {1: [-48.88, -48.91, -48.92], 4: [-64.31, -64.45, -64.56], 6: [-62.12, -62.09, -62.08], 8: [-55.37, -55.30, -55.04], 11: [-54.15, -54.32, -54.25]},
        2:  {1: [-53.76, -54.01, -54.41], 4: [-64.79, -64.64, -64.11], 6: [-63.47, -63.57, -63.86], 8: [-52.78, -53.03, -53.02], 11: [-55.51, -55.61, -55.49]},
        3:  {1: [-53.51, -53.61, -53.49], 4: [-54.73, -54.58, -54.27], 6: [-59.89, -59.91, -59.73], 8: [-59.99, -59.99, -59.60], 11: [-47.02, -47.02, -47.01]},
        4:  {1: [-53.29, -53.03, -53.02], 4: [-55.22, -55.17, -55.34], 6: [-64.37, -65.10, -65.28], 8: [-60.98, -61.39, -61.11], 11: [-50.09, -50.08, -49.86]},
        5:  {1: [-49.55, -49.64, -49.51], 4: [-51.49, -51.39, -51.31], 6: [-53.93, -54.15, -53.92], 8: [-62.77, -63.22, -62.78], 11: [-57.25, -57.00, -56.80]},
        6:  {1: [-57.09, -57.07, -56.86], 4: [-53.56, -53.65, -53.52], 6: [-48.42, -48.33, -48.27], 8: [-62.18, -61.95, -62.16], 11: [-52.98, -52.78, -52.62]},
        7:  {1: [-52.02, -52.01, -52.21], 4: [-48.45, -48.36, -48.29], 6: [-56.72, -56.78, -56.62], 8: [-52.51, -52.21, -52.17], 11: [-60.54, -60.84, -60.87]},
        8:  {1: [-52.55, -52.44, -52.35], 4: [-52.12, -51.90, -51.92], 6: [-62.11, -62.09, -62.27], 8: [-53.39, -53.31, -53.25], 11: [-57.22, -57.37, -57.50]},
        9:  {1: [-41.49, -41.39, -41.51], 4: [-47.85, -47.88, -47.71], 6: [-58.86, -59.09, -59.27], 8: [-55.90, -56.12, -55.90], 11: [-58.25, -58.40, -58.52]},
        10: {1: [-57.96, -57.77, -57.81], 4: [-52.83, -52.66, -52.73], 6: [-60.10, -60.08, -59.86], 8: [-52.49, -52.19, -52.36], 11: [-54.70, -54.76, -55.01]},
        11: {1: [-48.70, -48.76, -48.61], 4: [-63.45, -63.96, -63.37], 6: [-65.00, -64.80, -64.44], 8: [-60.94, -60.55, -60.44], 11: [-59.27, -59.62, -59.90]},
        12: {1: [-51.45, -51.56, -51.65], 4: [-63.32, -63.06, -63.05], 6: [-63.58, -63.46, -63.77], 8: [-56.53, -56.43, -56.34], 11: [-48.97, -48.77, -48.82]},
        13: {1: [-50.48, -50.38, -50.51], 4: [-64.88, -64.70, -64.56], 6: [-58.84, -58.67, -58.74], 8: [-56.87, -56.90, -56.92], 11: [-52.56, -52.44, -52.36]},
    }

    beacon_to_landmark = {
        1: 0,
        11: 1,
        4: 2,
        6: 3,
        8: 4,
    }

    rows = []

    for pose_idx in range(14):
        for beacon in [1, 4, 6, 8, 11]:
            mean_rssi = np.mean(raw[pose_idx][beacon])
            d = rssi_to_range_real(mean_rssi)
            rows.append([pose_idx, beacon_to_landmark[beacon], d])

    return np.array(rows, dtype=float)


def print_results(results):
    print("Parameters:")
    for k, v in results["params"].items():
        print(f" {k}: {v}")

    print(" \nMetrics:")
    for k, v in results["metrics"].items():
        print(f" {k}: {v}")

def run_baseline(args, filename1, filename2):
    results = experiments.run_experiment(**args)
    print_results(results)

    world_data = results["data"]["world"]
    dead_reckoned = results["data"]["dead_reckoned"]
    true_ranges = results["data"]["true_ranges"]
    ideal_ranges = results["data"]["ideal_ranges"]
    rssi_ranges = results["data"]["rssi_ranges"]
    """
    plots.plot_world_and_odometry(world_data, dead_reckoned)
    plots.plot_range_measurements_per_landmark(
        true_ranges,
        ideal_ranges,
        rssi_ranges,
        len(world_data["landmarks"])
    )
    """
    export_pyfg.save_sim_to_pyfg(
        filename1,
        world_data,
        results["data"]["noisy_odom"],
        ideal_ranges,
    )

    export_pyfg.save_sim_to_pyfg(
        filename2,
        world_data,
        results["data"]["noisy_odom"],
        rssi_ranges,
    )

    plots.plot_world_and_odometry(world_data, dead_reckoned)
    print("\nSaved sim_ideal.pyfg")
    print("Saved sim_rssi.pyfg")
    
    return results
def run_experiment_baseline(args, filename1):
    results = experiments.run_experiment(**args)
    print_results(results)

    world_data = results["data"]["world"]
    dead_reckoned = results["data"]["dead_reckoned"]
    true_ranges = results["data"]["true_ranges"]
    ideal_ranges = results["data"]["ideal_ranges"]
    rssi_ranges =  build_esp32_ranges()
    """
    plots.plot_world_and_odometry(world_data, dead_reckoned)
    plots.plot_range_measurements_per_landmark(
        true_ranges,
        ideal_ranges,
        rssi_ranges,
        len(world_data["landmarks"])
    )
    """
    export_pyfg.save_sim_to_pyfg(
        filename1,
        world_data,
        results["data"]["noisy_odom"],
        rssi_ranges
    )

    rmse = metrics.range_rmse(true_ranges, rssi_ranges)
    print("RSSI RMSE HARDWARE: " + str(rmse))
    plots.plot_world_and_odometry(world_data, dead_reckoned)
    print("\nSaved sim_ideal.pyfg")
    print("Saved sim_rssi.pyfg")
    
    return results

def rssi_to_range_real(rssi, A=-55.0, n=2.5):
    return 10 ** ((A - rssi) / (10*n))

def run_rssi_sweep():
    sweep_rssi = experiments.run_parameter_sweep(
        "sigma_rssi",
        SIGMA_RSSI_VALUES,
        BASE_KWARGS,
    )

    print("\nSweep: sigma_rssi")
    for row in sweep_rssi:
        print(row)

    plots.plot_sweep(
        sweep_rssi,
        "rssi_range_rmse",
        "RSSI Range RMSE vs RSSI Noise Sigma",
    )

    return sweep_rssi

def run_range_noise_sweep():
    sweep_range = experiments.run_parameter_sweep(
        "sigma_range",
        SIGMA_RSSI_VALUES,
        BASE_KWARGS,
    )

    print("\nSweep: sigma_ideal_range")
    for row in sweep_range:
        print(row)

    plots.plot_sweep(
        sweep_range,
        "ideal_range_rmse",
        "Ideal Range RMSE vs Range Noise Sigma",
    )

    return sweep_range

def run_noise_vs_rssi_sweep():
    sweep_range = experiments.run_parameter_sweep(
        "sigma_range",
        SIGMA_RANGE_VALUES,
        BASE_KWARGS,
    )
    sweep_rssi = experiments.run_parameter_sweep(
        "sigma_rssi",
        SIGMA_RSSI_VALUES,
        BASE_KWARGS,
    )

    plots.plot_double_sweep(
        sweep_range,sweep_rssi,
        "ideal_range_rmse","rssi_range_rmse",
        "Ideal Range noise RMSE vs RSSI noise RMSE from over a small sigma range",
    )
    return sweep_range

def run_measurement_interval_sweep():
    sweep_freq = experiments.run_parameter_sweep(
        "measure_every",
        MEASURE_EVERY_VALUES,
        BASE_KWARGS
    )

    print("\Sweep: measure_every")
    for row in sweep_freq:
        print(row)

    plots.plot_sweep(
        sweep_freq,
        "num_measurements",
        "Number of Measurements vs Measurement Interval",
    )

    plots.plot_sweep(
        sweep_freq,
        "pose_coverage",
        "Pose Coverage vs Measurement Interval",
    )

    return sweep_freq

def main():
    
    ARGS2 = dict(
        sigma_dx=0.005,
        sigma_dy=0.005,
        sigma_dtheta=0.0025,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=7,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS2,"range_odom_0_005.pyfg", "rssi_odom_0_005.pyfg",)
    run_experiment_baseline(ARGS2, "hardware_0_005.pyfg")
    ARGS3 = dict(
        sigma_dx=0.01,
        sigma_dy=0.01,
        sigma_dtheta=0.005,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=7,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS3, "range_odom_0_01.pyfg", "rssi_odom_0_01.pyfg")
    run_experiment_baseline(ARGS3, "hardware_0_01.pyfg")

    ARGS4 = dict(
        sigma_dx=0.02,
        sigma_dy=0.02,
        sigma_dtheta=0.01,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=7,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS4, "range_odom_0_02.pyfg", "rssi_odom_0_02.pyfg")
    run_experiment_baseline(ARGS4, "hardware_0_02.pyfg")



    ARGS5 = dict(
        sigma_dx=0.03,
        sigma_dy=0.03,
        sigma_dtheta=0.015,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=7,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS5, "range_odom_0_03.pyfg", "rssi_odom_0_03.pyfg")
    run_experiment_baseline(ARGS5, "hardware_0_03.pyfg")

    ARGS6 = dict(
        sigma_dx=0.04,
        sigma_dy=0.04,
        sigma_dtheta=0.02,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=7,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS6, "range_odom_0_04.pyfg", "rssi_odom_0_04.pyfg")
    run_experiment_baseline(ARGS6, "hardware_0_04.pyfg")
    ARGS7 = dict(
        sigma_dx=0.06,
        sigma_dy=0.06,
        sigma_dtheta=0.03,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=7,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS7, "range_odom_0_06.pyfg", "rssi_odom_0_06.pyfg")
    run_experiment_baseline(ARGS7, "hardware_0_06.pyfg")
    ARGS8 = dict(
        sigma_dx=0.08,
        sigma_dy=0.08,
        sigma_dtheta=0.04,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=7,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS8, "range_odom_0_08.pyfg", "rssi_odom_0_08.pyfg")
    run_experiment_baseline(ARGS8, "hardware_0_08.pyfg")

    ARGS9 = dict(
        sigma_dx=0.1,
        sigma_dy=0.1,
        sigma_dtheta=0.05,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=4,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS9, "range_odom_0_1.pyfg", "rssi_odom_0_1.pyfg")
    run_experiment_baseline(ARGS9, "hardware_0_1.pyfg")

    ARGS10 = dict(
        sigma_dx=0.2,
        sigma_dy=0.2,
        sigma_dtheta=0.1,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=4,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS10, "range_odom_0_2.pyfg", "rssi_odom_0_2.pyfg")
    run_experiment_baseline(ARGS10, "hardware_0_2.pyfg")

    ARGS11 = dict(
        sigma_dx=0.25,
        sigma_dy=0.25,
        sigma_dtheta=0.125,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=4,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS11, "range_odom_0_25.pyfg", "rssi_odom_0_25.pyfg")
    run_experiment_baseline(ARGS11, "hardware_0_25.pyfg")

    ARGS12 = dict(
        sigma_dx=0.3,
        sigma_dy=0.3,
        sigma_dtheta=0.15,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=4,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )
    run_baseline(ARGS12, "range_odom_0_3.pyfg", "rssi_odom_0_3.pyfg")
    run_experiment_baseline(ARGS12, "hardware_0_3.pyfg")
    
    ARGS13 = dict(
        sigma_dx=0.4,
        sigma_dy=0.4,
        sigma_dtheta=0.2,
        sigma_range=0.2,
        A=-55.0,
        n=2.5,
        sigma_rssi=4,
        max_range=50.0,
        measure_every=1,
        dropout_prob=0.0,
        seed=0,
    )

    run_baseline(ARGS13, "range_odom_0_4.pyfg", "rssi_odom_0_4.pyfg")
    run_experiment_baseline(ARGS13, "hardware_0_4.pyfg")

    #print("\n\nRUN MEASUREMENT INTERVAL SWEEP")
    #run_measurement_interval_sweep()
    run_rssi_sweep()

if __name__ == "__main__":
    main()



