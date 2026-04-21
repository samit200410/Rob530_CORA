import numpy as np
import matplotlib.pyplot as plt


def compute_true_ranges(poses, landmarks):
    """
    Compute ground truth range measurements
    """
    measurements = []

    for k, (x,y,_) in enumerate(poses):
        for j, (lx,ly) in enumerate(landmarks):
            r = np.sqrt((x - lx)**2 + (y-ly)**2)
            measurements.append([k, j, r])
    print(measurements)
    return np.array(measurements, dtype=float)

def compute_true_ranges_variable(poses, landmarks, max_range=12.0, measure_every=2, dropout_prob=0.0, rng=None):
    """
    Ground truth measurement with system variables
    ------------------------------------------------
    max_range: Range values greater than (value) meters will not be sensed
    measure_every: Use (value) greater than 1 to decrease sampling rate by that factor
    dropout_prob: Random number generator simulates dropout with probability == (value)
    rng: Random number generation seed
    """
    if rng is None:
        rng = np.random.default_rng(0)
    measurements = []

    for k, (x,y,_) in enumerate(poses):
        if k % measure_every != 0:
            continue

        for j, (lx, ly) in enumerate(landmarks):
            r = np.sqrt((x - lx)**2 + (y - ly)**2)

            if r > max_range:
                continue

            if rng.uniform() < dropout_prob:
                continue

            measurements.append([k,j,r])
    print(measurements[1])

    return np.array(measurements, dtype = float)


def add_range_noise(true_ranges, sigma_r=0.1, rng=None):
    """
    Add gaussian noise to range measurements
    -----------------------------------------
    sigma_r: Noise Variance value. (Value) larger than 0 will increase noise
    """
    if rng is None:
        rng = np.random.default_rng(0)

    noisy = []
    for k, j, r in true_ranges:
        r_noisy = max(r + rng.normal(0.0, sigma_r), 1e-3)
        noisy.append([int(k), int(j), r_noisy])
    print(noisy[1])
    return np.array(noisy, dtype=float)

def simulate_rssi_from_range(true_ranges, A=-55.0, n=2.5, sigma_rssi=2.0, rng=None):
    """
    Calculate path loss heuristc that converts distance (m) to RSSI value
    -----------------------------------------------------------------
    A: RSSI value at 1 meter
    n: Environmental disturbance factor. 
        Free space:        n = 2.0
        Indoor/office:     n = 2.5 - 3.0
    sigma_rssi: RSSI signal noise variance
    rng: Random number generation seed
    """
    if rng is None:
        rng = np.random.default_rng(0)

    rssi_measurements = []

    for k, j, d in true_ranges:
        d = max(d, 1e-3)
        rssi = A - 10.0 * n * np.log10(d) + rng.normal(0.0, sigma_rssi)
        rssi_measurements.append([k, j, rssi])

    return np.array(rssi_measurements, dtype=float)

def rssi_to_range(rssi_measurements, A = -55.0, n = 2.5):
    """
    Calculate path loss heuristc that converts RSSI to distance (m)
    -----------------------------------------------------------------
    A: RSSI value at 1 meter
    n: Environmental disturbance factor. 
        Free space:        n = 2.0
        Indoor/office:     n = 2.5 - 3.0
    sigma_rssi: RSSI signal noise variance
    rng: Random number generation seed
    """
    pseudo_ranges = []

    for k,j,rssi in rssi_measurements:
        d_hat = 10**((A - rssi)/(10.0 * n))
        pseudo_ranges.append([k,j,d_hat])
    return np.array(pseudo_ranges, dtype=float)

