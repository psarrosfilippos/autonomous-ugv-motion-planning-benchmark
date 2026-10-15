# Autonomous UGV Motion Planning Benchmark

A Python simulation framework for benchmarking motion planning algorithms (A*, RRT, and Hybrid A*) on non-holonomic Autonomous Ground Vehicles (UGVs).

## Overview

The simulator tests path planning under two modes:
1. **Known Goal (Global Planning):** Finds the optimal kinematic path to target coordinates.
2. **Blind Exploration (Unknown Goal):** Navigates unknown, randomly generated environments using a simulated 2D LiDAR, a dynamic visitation heatmap to escape local minima, and a tiered recovery (fail-safe) system.

## Algorithms Implemented

* **A\*:** Fast 8-connected grid search (used primarily for baseline and rapid fail-safe recovery).
* **RRT:** Sampling-based planner exploring continuous state-space with steering angle constraints.
* **Hybrid A\*:** Kinematically feasible path generation considering continuous coordinates, vehicle orientation (yaw), and turn penalties.


### Prerequisites

* Python 3.8+
* Tkinter (included with standard Python on Windows/macOS, or install via `sudo apt install python3-tk` on Linux)


### Author
Filippos Psarros

MSc Candidate in Unmanned Autonomous and Remote-Controlled Systems (University of West Attica)

BSc in Informatics and Telecommunications
