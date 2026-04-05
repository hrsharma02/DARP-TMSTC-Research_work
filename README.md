# DARP-TMSTC-Research_work
# DARP‑TMSTC: Energy‑Efficient Multi‑Robot Coverage with Workload Balancing and Turn Minimization

[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

This repository contains the complete simulation code, experimental results, and figures for the paper:

**"Energy‑Efficient Multi‑robot Coverage Path Planning with Workload Balancing and Turn Minimization"**  
*(To be published – citation will be added)*

The framework integrates the **Divide Areas Algorithm for Optimal Multi‑Robot Coverage (DARP)** with **Turn‑Minimized Spanning Tree Coverage (TMSTC)** to achieve energy‑aware, balanced, and scalable coverage in obstacle‑rich environments.

## Key Features

- **336 systematic experiments** across 4 grid sizes (20×20 to 50×50), 3 obstacle densities (10%, 12%, 15%), and robot teams of 2‑5.
- Comparison with **6 state‑of‑the‑art methods**: Grid, MSTC, Boustrophedon, Random, MARL proxy, Auction‑based proxy.
- **Physics‑based energy model** (turn cost = 3.2 × straight cell cost) derived from differential‑drive robot dynamics.
- **Sign test statistical validation** (p < 0.001) showing DARP‑TMSTC outperforms every baseline in all 48 distinct configurations.
- **Complete coverage** in all runs, execution times < 5 seconds for most scenarios.

## Repository Structure
