# Rob530_CORA
Last Edited Apr 17, 2026.

UMich ROB530 Project by Samit Mohapatra, Kevin Chang, Nicolas Betancur, and Holden Halucha

This project investigates Range Aided SLAM using RSSI as a proxy for distance, and the CORA framework developed by Dr. Alan Papalia as the estimation back end.

This repository provides a user with tools necessary for authoring CORA legible range aided datasets, as well as firmware for creating your own real world ESP32 RSSI RA-SLAM Experiment!

Repository Contents:

ESP32 Firmware:
-
- PlatformIO project for flashing and running ESP32 devices in a Range Sensing network.
- Features include a client node and up to 5 beacon nodes, all transmitting on different frequency bands.
- Requires PlatformIO extension in VSCode
- The .ini file configures how the hardware is flashed. Make sure to use the correct USB ports for all of the ESP32s.

CORA Solver
-
- C++ library for running CORA on Range Aided SLAM datasets.
- For setup instructions, visit https://github.com/MarineRoboticsGroup/cora

RA-Research Library
-
- Simulation tools for authoring Range Aided SLAM datasets.
- Datasets can be generated as RSSI-to-distance with noise injected into the RSSI measurement, or as noisy range data where noise is injected into the range measurement.
