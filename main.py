
import numpy as np
import matplotlib.pyplot as plt
import random
import copy

from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

# ============================================================
# 1. PROJECT CONFIGURATION
# ============================================================

random.seed(42)
np.random.seed(42)

GRID_SIZE = 50
NUM_DRONES = 3
NUM_WAYPOINTS = 5

POPULATION_SIZE = 40
GENERATIONS = 80
MUTATION_RATE = 0.15
TOURNAMENT_SIZE = 3
ELITE_COUNT = 2

MAX_FLIGHT_DISTANCE = 100
MAX_MISSION_TIME = 60
SENSOR_RADIUS = 3

# Grid cell types
# 0 = Safe, 1 = Obstacle  No-fly zone
# 2 = Hazard, 3 = High-priority target

grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)

grid[1018, 1015] = 1
grid[2532, 3038] = 1
grid[510, 3542] = 1

grid[2025, 515] = 2
grid[3542, 2028] = 2

grid[510, 510] = 3
grid[3035, 512] = 3
grid[3844, 3845] = 3

drone_starts = [(2, 2), (2, 25), (2, 45)]

# Only free cells count toward general area coverage.
free_cells = np.sum(grid != 1)
target_cells = np.sum(grid == 3)

print(Environment created, grid.shape)
print(Number of drones, NUM_DRONES)
print(High-priority target cells, target_cells)


# ============================================================
# 2. CHROMOSOME REPRESENTATION
# ============================================================

# One chromosome = one complete mission for all drones.
# Each drone has waypoints, speeds and sensor power settings.

class DronePlan
    OOP representation of one drone's mission plan.

    def __init__(self, waypoints, speeds, sensor_power)
        self.waypoints = waypoints
        self.speeds = speeds
        self.sensor_power = sensor_power

    # Dictionary-style access keeps the existing GA operators compatible.
    def __getitem__(self, key)
        return getattr(self, key)

    def __setitem__(self, key, value)
        setattr(self, key, value)

    def to_dict(self)
        return {
            waypoints self.waypoints,
            speeds self.speeds,
            sensor_power self.sensor_power,
        }


class Chromosome
    One candidate solution containing plans for the entire UAV fleet.

    def __init__(self, drone_plans)
        self.drone_plans = drone_plans

    # List-style access keeps existing evaluationselection code compatible.
    def __iter__(self)
        return iter(self.drone_plans)

    def __getitem__(self, index)
        return self.drone_plans[index]

    def __len__(self)
        return len(self.drone_plans)

    def to_dict(self)
        return [plan.to_dict() for plan in self.drone_plans]


def create_chromosome()
    drone_plans = []

    for start in drone_starts
        plan = DronePlan(
            waypoints=[
                (random.randint(0, GRID_SIZE - 1),
                 random.randint(0, GRID_SIZE - 1))
                for _ in range(NUM_WAYPOINTS)
            ],
            speeds=[
                random.randint(1, 5)
                for _ in range(NUM_WAYPOINTS)
            ],
            sensor_power=[
                random.randint(0, 2)
                for _ in range(NUM_WAYPOINTS)
            ],
        )
        drone_plans.append(plan)

    return Chromosome(drone_plans)


def create_population(size=POPULATION_SIZE)
    return [create_chromosome() for _ in range(size)]


# ============================================================
# 3. PATH AND SENSOR COVERAGE HELPERS
# ============================================================

def segment_cells(start, end)
    Sample a straight flight segment to check grid cells.
    x1, y1 = start
    x2, y2 = end

    distance = np.hypot(x2 - x1, y2 - y1)
    steps = max(2, int(np.ceil(distance  2)))

    xs = np.rint(np.linspace(x1, x2, steps + 1)).astype(int)
    ys = np.rint(np.linspace(y1, y2, steps + 1)).astype(int)

    # Remove consecutive duplicate grid cells.
    cells = []
    for x, y in zip(xs, ys)
        point = (int(x), int(y))
        if not cells or cells[-1] != point
            cells.append(point)

    return cells


def mark_sensor_coverage(coverage, point, power)
    Mark cells surveyed by a sensor at a waypoint.
    if power = 0
        return

    x, y = point
    radius = SENSOR_RADIUS  (0.6 + 0.4  power)

    x_min = max(0, int(np.floor(x - radius)))
    x_max = min(GRID_SIZE - 1, int(np.ceil(x + radius)))
    y_min = max(0, int(np.floor(y - radius)))
    y_max = min(GRID_SIZE - 1, int(np.ceil(y + radius)))

    for xx in range(x_min, x_max + 1)
        for yy in range(y_min, y_max + 1)
            if np.hypot(xx - x, yy - y) = radius
                if grid[yy, xx] != 1
                    coverage[yy, xx] = 1


# ============================================================
# 4. MULTI-OBJECTIVE FITNESS FUNCTION
# ============================================================

def evaluate_fitness(chromosome, return_details=False)
    coverage = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)

    total_distance = 0.0
    total_time = 0.0
    total_energy = 0.0
    total_samples = 0
    hazard_samples = 0

    obstacle_violations = 0
    distance_violations = 0
    time_violations = 0

    drone_distances = []
    drone_times = []

    for i, plan in enumerate(chromosome)
        start = drone_starts[i]
        current = start

        drone_distance = 0.0
        drone_time = 0.0
        drone_energy = 0.0

        # Include starting position in the flight path.
        mark_sensor_coverage(coverage, current, 1)

        for j, waypoint in enumerate(plan[waypoints])
            speed = plan[speeds][j]
            power = plan[sensor_power][j]

            distance = float(np.hypot(
                waypoint[0] - current[0],
                waypoint[1] - current[1]
            ))

            cells = segment_cells(current, waypoint)
            values = [grid[y, x] for x, y in cells]

            total_samples += len(values)
            hazard_samples += sum(value == 2 for value in values)

            # Hard constraints are enforced through penalties.
            if any(value == 1 for value in values)
                obstacle_violations += 1

            # Flight time and energy estimates.
            segment_time = distance  speed
            segment_energy = distance  (
                1.0 + 0.10  speed  2
            ) + 2.0  power

            drone_distance += distance
            drone_time += segment_time
            drone_energy += segment_energy

            mark_sensor_coverage(coverage, waypoint, power)
            current = waypoint

        # Return to the starting location after the mission.
        distance = float(np.hypot(
            current[0] - start[0],
            current[1] - start[1]
        ))

        cells = segment_cells(current, start)
        values = [grid[y, x] for x, y in cells]

        total_samples += len(values)
        hazard_samples += sum(value == 2 for value in values)

        if any(value == 1 for value in values)
            obstacle_violations += 1

        drone_distance += distance
        drone_time += distance  3.0
        drone_energy += distance  (1.0 + 0.10  3.0  2)

        if drone_distance  MAX_FLIGHT_DISTANCE
            distance_violations += 1

        if drone_time  MAX_MISSION_TIME
            time_violations += 1

        total_distance += drone_distance
        total_time += drone_time
        total_energy += drone_energy

        drone_distances.append(drone_distance)
        drone_times.append(drone_time)

    # General area coverage.
    covered_cells = np.sum((coverage == 1) & (grid != 1))
    area_coverage = covered_cells  free_cells

    # Target detection rate.
    detected_targets = np.sum((coverage == 1) & (grid == 3))
    detection_rate = detected_targets  max(target_cells, 1)

    # Normalized objective scores, all between 0 and 1.
    energy_efficiency = 1.0 - min(
        total_energy  (NUM_DRONES  MAX_FLIGHT_DISTANCE  2.0),
        1.0
    )

    time_efficiency = 1.0 - min(
        total_time  (NUM_DRONES  MAX_MISSION_TIME),
        1.0
    )

    hazard_rate = hazard_samples  max(total_samples, 1)
    safety_score = 1.0 - hazard_rate

    # Weighted multi-objective fitness.
    base_fitness = (
        0.25  area_coverage
        + 0.25  detection_rate
        + 0.15  energy_efficiency
        + 0.10  time_efficiency
        + 0.25  safety_score
    )

    # Penalties for violating constraints.
    penalty = (
        0.12  obstacle_violations
        + 0.30  distance_violations
        + 0.30  time_violations
    )

    fitness = base_fitness - penalty

    details = {
        fitness fitness,
        area_coverage area_coverage,
        detection_rate detection_rate,
        energy_efficiency energy_efficiency,
        time_efficiency time_efficiency,
        safety_score safety_score,
        total_distance total_distance,
        total_time total_time,
        total_energy total_energy,
        obstacle_violations obstacle_violations,
        distance_violations distance_violations,
        time_violations time_violations,
        coverage coverage,
        detected_targets int(detected_targets),
        drone_distances drone_distances,
        drone_times drone_times
    }

    if return_details
        return details

    return fitness


# ============================================================
# 5. SELECTION TOURNAMENT SELECTION
# ============================================================

def tournament_selection(population, tournament_size=TOURNAMENT_SIZE)
    candidates = random.sample(
        population,
        min(tournament_size, len(population))
    )

    return copy.deepcopy(
        max(candidates, key=evaluate_fitness)
    )


# ============================================================
# 6. CUSTOM CROSSOVER
# ============================================================

def crossover(parent1, parent2)
    child1 = copy.deepcopy(parent1)
    child2 = copy.deepcopy(parent2)

    # Swap a section of waypoints and corresponding settings.
    for d in range(NUM_DRONES)
        cut = random.randint(1, NUM_WAYPOINTS - 1)

        for key in [waypoints, speeds, sensor_power]
            child1[d][key][cut], child2[d][key][cut] = (
                copy.deepcopy(parent2[d][key][cut]),
                copy.deepcopy(parent1[d][key][cut])
            )

    return child1, child2


# ============================================================
# 7. MUTATION
# ============================================================

def mutate(chromosome, mutation_rate=MUTATION_RATE)
    for plan in chromosome
        for j in range(NUM_WAYPOINTS)

            # Spatial mutation perturb waypoint coordinates.
            if random.random()  mutation_rate
                x, y = plan[waypoints][j]

                x = int(np.clip(
                    x + random.randint(-5, 5), 0, GRID_SIZE - 1
                ))
                y = int(np.clip(
                    y + random.randint(-5, 5), 0, GRID_SIZE - 1
                ))

                plan[waypoints][j] = (x, y)

            # Speed mutation.
            if random.random()  mutation_rate
                plan[speeds][j] = random.randint(1, 5)

            # Sensor power mutation.
            if random.random()  mutation_rate
                plan[sensor_power][j] = random.randint(0, 2)

    return chromosome


# ============================================================
# 8. GENETIC ALGORITHM
# ============================================================

def run_ga(
    population_size=POPULATION_SIZE,
    generations=GENERATIONS,
    mutation_rate=MUTATION_RATE,
    tournament_size=TOURNAMENT_SIZE
)
    population = create_population(population_size)

    best_history = []
    average_history = []

    best_chromosome = None
    best_fitness = float(-inf)

    for generation in range(generations)
        scores = [evaluate_fitness(c) for c in population]

        generation_best_index = int(np.argmax(scores))
        generation_best = scores[generation_best_index]

        if generation_best  best_fitness
            best_fitness = generation_best
            best_chromosome = copy.deepcopy(
                population[generation_best_index]
            )

        best_history.append(best_fitness)
        average_history.append(float(np.mean(scores)))

        # Elitism preserve the strongest solutions.
        ranked = sorted(
            population,
            key=evaluate_fitness,
            reverse=True
        )

        elite_count = min(ELITE_COUNT, population_size)
        new_population = copy.deepcopy(ranked[elite_count])

        while len(new_population)  population_size
            parent1 = tournament_selection(
                population, tournament_size
            )
            parent2 = tournament_selection(
                population, tournament_size
            )

            child1, child2 = crossover(parent1, parent2)

            mutate(child1, mutation_rate)
            mutate(child2, mutation_rate)

            new_population.append(child1)

            if len(new_population)  population_size
                new_population.append(child2)

        population = new_population

    return {
        best_chromosome best_chromosome,
        best_fitness best_fitness,
        best_history best_history,
        average_history average_history
    }


# ============================================================
# 9. RUN THE MAIN SIMULATION
# ============================================================

print(nRunning Genetic Algorithm...)

result = run_ga()

best_chromosome = result[best_chromosome]
best_details = evaluate_fitness(
    best_chromosome, return_details=True
)

print(nSIMULATION COMPLETE)
print(Best fitness, round(best_details[fitness], 4))
print(Area coverage, round(
    best_details[area_coverage]  100, 2
), %)
print(Target detection, round(
    best_details[detection_rate]  100, 2
), %)
print(Energy efficiency, round(
    best_details[energy_efficiency]  100, 2
), %)
print(Time efficiency, round(
    best_details[time_efficiency]  100, 2
), %)
print(Safety score, round(
    best_details[safety_score]  100, 2
), %)
print(Total estimated distance, round(
    best_details[total_distance], 2
))
print(Total estimated mission time, round(
    best_details[total_time], 2
))
print(Obstacle violations, best_details[obstacle_violations])
print(Flight-distance violations,
      best_details[distance_violations])
print(Mission-time violations, best_details[time_violations])


# ============================================================
# 10. FITNESS CONVERGENCE GRAPH
# ============================================================

plt.figure(figsize=(9, 5))

plt.plot(
    result[best_history],
    label=Best fitness
)

plt.plot(
    result[average_history],
    label=Average fitness
)

plt.xlabel(Generation)
plt.ylabel(Fitness)
plt.title(GA Fitness Convergence)
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(fitness_convergence.png, dpi=200)
plt.show()


# ============================================================
# 11. VISUALIZE THE EVOLVED DRONE PATHS
# ============================================================

colors = ListedColormap([
    lightgreen, black, orange, red
])

plt.figure(figsize=(10, 8))
plt.imshow(
    grid, cmap=colors, origin=lower,
    interpolation=nearest
)

path_colors = [blue, purple, cyan]

for i, plan in enumerate(best_chromosome)
    points = [drone_starts[i]] + plan[waypoints] + [
        drone_starts[i]
    ]

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    plt.plot(
        xs, ys, marker=o, linewidth=2,
        color=path_colors[i], label=fDrone {i+1}
    )

    plt.scatter(
        drone_starts[i], marker=, s=180,
        color=path_colors[i], edgecolors=white
    )

legend_items = [
    Patch(color=lightgreen, label=Safe),
    Patch(color=black, label=Obstacle),
    Patch(color=orange, label=Hazard),
    Patch(color=red, label=Target)
]

plt.legend(handles=legend_items + [
    plt.Line2D([0], [0], color=path_colors[i],
               label=fDrone {i+1})
    for i in range(NUM_DRONES)
], bbox_to_anchor=(1.02, 1), loc=upper left)

plt.title(Evolved UAV Flight Paths)
plt.xlabel(X Coordinate)
plt.ylabel(Y Coordinate)
plt.tight_layout()
plt.savefig(evolved_paths.png, dpi=200)
plt.show()


# ============================================================
# 12. COVERAGE HEATMAP
# ============================================================

plt.figure(figsize=(8, 7))

plt.imshow(
    best_details[coverage],
    origin=lower,
    cmap=YlOrRd,
    interpolation=nearest
)

plt.colorbar(label=Surveyed Cell (1 = Covered))
plt.title(UAV Area Coverage Heatmap)
plt.xlabel(X Coordinate)
plt.ylabel(Y Coordinate)
plt.tight_layout()
plt.savefig(coverage_heatmap.png, dpi=200)
plt.show()


# ============================================================
# 13. RANDOM SEARCH BASELINE
# ============================================================

print(nRunning random-search baseline...)

# Same number of candidate evaluations as the GA's
# population evaluations across its generations.
random_evaluations = POPULATION_SIZE  GENERATIONS

random_best = None
random_best_fitness = float(-inf)
random_history = []

for i in range(random_evaluations)
    candidate = create_chromosome()
    score = evaluate_fitness(candidate)

    if score  random_best_fitness
        random_best_fitness = score
        random_best = copy.deepcopy(candidate)

    random_history.append(random_best_fitness)

print(GA best fitness, round(result[best_fitness], 4))
print(Random-search best fitness,
      round(random_best_fitness, 4))

plt.figure(figsize=(9, 5))

plt.plot(
    result[best_history],
    label=Genetic Algorithm
)

random_per_generation = [
    random_history[
        min((i + 1)  POPULATION_SIZE - 1,
            len(random_history) - 1)
    ]
    for i in range(GENERATIONS)
]

plt.plot(
    random_per_generation,
    label=Random Search
)

plt.xlabel(Generation-equivalent evaluation block)
plt.ylabel(Best fitness so far)
plt.title(GA vs Random Search)
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(ga_vs_random_search.png, dpi=200)
plt.show()


# ============================================================
# 14. HYPERPARAMETER SENSITIVITY ANALYSIS
# ============================================================

print(nRunning hyperparameter sensitivity experiments...)

experiments = [
    (Small population, 20, 0.15, 3),
    (Large population, 60, 0.15, 3),
    (Low mutation, 30, 0.05, 3),
    (High mutation, 30, 0.35, 3),
    (Low selection pressure, 30, 0.15, 2),
    (High selection pressure, 30, 0.15, 5)
]

experiment_names = []
experiment_scores = []

# Fewer generations keep the comparison reasonably quick.
for name, pop_size, mutation, tournament in experiments
    exp_result = run_ga(
        population_size=pop_size,
        generations=25,
        mutation_rate=mutation,
        tournament_size=tournament
    )

    experiment_names.append(name)
    experiment_scores.append(exp_result[best_fitness])

    print(name, -,
          round(exp_result[best_fitness], 4))

plt.figure(figsize=(11, 5))

plt.bar(experiment_names, experiment_scores)

plt.ylabel(Best fitness)
plt.title(Hyperparameter Sensitivity Analysis)
plt.xticks(rotation=25, ha=right)
plt.grid(axis=y, alpha=0.3)
plt.tight_layout()
plt.savefig(sensitivity_analysis.png, dpi=200)
plt.show()


# ============================================================
# 15. SAVE RESULTS AND DISPLAY SUMMARY
# ============================================================

import csv
import json

# Save numerical convergence data.
with open(fitness_history.csv, w, newline=) as file
    writer = csv.writer(file)
    writer.writerow([
        Generation, Best Fitness, Average Fitness
    ])

    for i, (best, avg) in enumerate(zip(
        result[best_history], result[average_history]
    ))
        writer.writerow([i + 1, best, avg])

# Save the best chromosome in a readable format.
with open(best_mission.json, w) as file
    json.dump(best_chromosome.to_dict(), file, indent=2)

# Save experiment scores.
with open(sensitivity_results.csv, w, newline=) as file
    writer = csv.writer(file)
    writer.writerow([Experiment, Best Fitness])
    writer.writerows(zip(
        experiment_names, experiment_scores
    ))

print(nALL CODING EXPERIMENTS FINISHED)
print(Generated files)
print(- fitness_convergence.png)
print(- evolved_paths.png)
print(- coverage_heatmap.png)
print(- ga_vs_random_search.png)
print(- sensitivity_analysis.png)
print(- fitness_history.csv)
print(- best_mission.json)
print(- sensitivity_results.csv)
