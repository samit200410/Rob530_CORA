# config.py

BASE_KWARGS = dict(
    sigma_dx=0.02,
    sigma_dy=0.02,
    sigma_dtheta=0.01,
    sigma_range=0.1,
    A=-45.0,
    n=2.0,
    sigma_rssi=0.1,
    max_range=12.0,
    measure_every=2,
    dropout_prob=0.0,
    seed=0,
)

SIGMA_RSSI_VALUES = [0.5, 1, 2, 3, 4,5, 6,7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]

SIGMA_RANGE_VALUES = [0.5, 1, 2, 3, 4,5, 6,7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]


MEASURE_EVERY_VALUES = [1, 2, 4, 5, 10]

MAX_RANGE_VALUES = [2.5, 4.0, 6.0, 8.0, 12.0]

DROPOUT_VALUES = [0.0, 0.1, 0.2, 0.4, 0.6]