# Autonomous UGV Motion Planning Benchmark

A Python simulation framework for benchmarking motion planning algorithms (A*, RRT, and Hybrid A*) on non-holonomic Autonomous Ground Vehicles (UGVs).

## Overview

The simulator benchmarks path planning in procedural maze environments under two execution modes:
1. **Known Goal (Global Planning):** Computes optimal trajectories to target coordinates while respecting vehicle dimensions (3.5m x 1.8m) and Ackermann kinematics.
2. **Blind Exploration (Unknown Goal):** Navigates unknown maps under partial visibility using simulated 2D LiDAR (8m range), dynamic obstacle inflation (2.2m buffer), visitation heatmaps to escape local minima, and tiered failsafe recovery.

## Algorithms Implemented

* **A\* (Discrete Search):** 8-connected grid planner; mathematically optimal in distance, used primarily as an instant recovery mechanism.
* **RRT (Sampling-Based):** Explores continuous state-space with steering angle constraints and fallback logic to avoid stagnation.
* **Hybrid A\* (Kinematic Optimization):** Explores 3D state-space (x, y, theta) using vehicle kinematic equations and steering change penalties to output smooth, drivable paths.

## Key System Features

* **Map Traversability:** Automated map generation enforcing a minimum 5.0m (1.4x vehicle length) clearance between obstacles.
* **Sensory Perception:** Simulated 2D LiDAR with Fog-of-War obstacle revelation and 2.2m obstacle inflation margins.
* **Recovery Logic:** Tiered fail-safe engine comprising Micro-Push (1.5m inspection), Heatmap memory penalties, Backtracking (25 historical steps), and Origin Unstuck reset.

## Prerequisites

* Python 3.8+
* Dependencies: `numpy`, `matplotlib`
* UI: `tkinter` (pre-installed on Windows/macOS, or install via `sudo apt install python3-tk` on Linux)

## Author

**Filippos Psarros**
* MSc Candidate in Unmanned Autonomous and Remote-Controlled Systems (University of West Attica)
* BSc in Informatics and Telecommunications (Computational Systems)
