# UAV Fleet Optimization Using Genetic Algorithm

## Project Overview
This project implements a Genetic Algorithm (GA) to optimize a fleet of Unmanned Aerial Vehicles (UAVs) for disaster management and surveillance. The simulation models a hazardous grid environment and evolves UAV flight plans to improve coverage, target detection, energy efficiency, flight time, and safety.

## Objectives
- Maximize grid coverage and high-priority target detection.
- Reduce energy consumption and flight time.
- Minimize exposure to hazardous areas.
- Apply penalties for constraint violations.
- Compare Genetic Algorithm results with a random-search baseline.

## Environment and Configuration
- Grid size: 50 × 50
- UAV fleet: 3 drones
- Waypoints per drone: 5
- Population size: 40
- Generations: 80
- Mutation rate: 0.15
- Tournament selection size: 3
- Elite count: 2

## Fitness Function
The fitness function combines the following objectives:

| Objective | Weight |
|---|---:|
| Coverage | 0.25 |
| Target detection | 0.25 |
| Energy efficiency | 0.15 |
| Time efficiency | 0.10 |
| Safety | 0.25 |

Penalties are applied for obstacle/no-fly-zone violations, excessive flight distance, and time-limit violations.

## Requirements
- Python 3
- NumPy
- Matplotlib
- Pandas
- Seaborn

## Installation and Execution
1. Clone or download this repository.
2. Open a terminal in the project directory.
3. Install dependencies:

   `pip install -r requirements.txt`

4. Run the simulation:

   `python main.py`

The simulation should generate result files and visualizations according to the implementation and configuration in `main.py`.

## Results
The project evaluates fitness convergence, evolved UAV paths, coverage, and performance compared with a random-search baseline. Result files and plots should be included in the repository when generated.

## Limitations
The simulation is a simplified grid-based model. Penalty-based constraint handling may not guarantee that every evolved path is feasible. Results depend on the random seed, parameters, and environment configuration.

## Author
BSCS — Bahria University Lahore Campus
