# UAV Deployment and Path Planning using a Genetic Algorithm

## Project overview
This project uses a Genetic Algorithm (GA) to evolve a UAV mission plan on a 20 x 20 grid. A chromosome contains:
- A list of waypoints, including a fixed start and destination
- UAV speed (1 to 5)
- Sensor power (20 to 100)

The fitness function rewards target detection and penalizes distance, flight time, energy use, hazards, obstacle collisions, and exceeding the maximum flight range.

## Files
- `main.py`: starts the program and displays the results
- `environment.py`: grid, obstacles, targets, hazards, start, and destination
- `drone.py`: chromosome representation and random chromosome creation
- `genetic_algorithm.py`: fitness, population, tournament selection, crossover, mutation, GA loop, plots, and random-search comparison
- `requirements.txt`: required Python packages

## Requirements
Python 3.10 or later is recommended. Install the packages with:

```bash
py -m pip install -r requirements.txt
```

## Run
From the project folder, run:

```bash
py main.py
```

The program prints fitness for each generation, prints the best evolved chromosome, displays a fitness graph, an evolved path plot, a coverage heatmap, and compares the GA result with random search.

## Genetic Algorithm settings
The default settings are:
- Population size: 40
- Generations: 60
- Mutation rate: 0.15
- Elite count: 2
- Tournament selection: best of 3 randomly selected candidates

To test sensitivity, change one setting at a time in `main.py`, run the program again, and record the best fitness and runtime. For example, compare population sizes 20, 40, and 80, or mutation rates 0.05, 0.15, and 0.30.

## Fitness function
The fitness score is calculated as:

`100 * detected_targets - distance - 1.5 * flight_time - 0.5 * energy - 100 * obstacle_hits - 25 * hazard_hits`

If total route distance exceeds the maximum flight range (70), an additional penalty is applied. Higher fitness is better.

## Notes and limitations
This is an educational grid simulation. Waypoints are randomly generated and the fitness function penalizes route segments that pass through obstacle cells; it does not perform a full geometric route-repair algorithm. The sensor model uses a simplified circular detection radius (`sensor_power / 20`). The energy and time equations are simplified estimates, not real UAV engineering models. A single random-search comparison is illustrative; repeat runs for stronger conclusions.

