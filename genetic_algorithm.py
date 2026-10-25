import math
import random
import matplotlib.pyplot as plt
import numpy as np
from drone import Drone


class GeneticAlgorithm:
    def __init__(self, environment, population_size=40, generations=60,
                 mutation_rate=0.15, elite_count=2):
        self.environment = environment
        self.population_size = population_size
        self.generations = generations
        self.mutation_rate = mutation_rate
        self.elite_count = elite_count
        self.best_fitness_history = []
        self.average_fitness_history = []
        self.best_drone = None

    def distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def sample_segment(self, point1, point2):
        """Return approximate grid points between two waypoints."""
        x1, y1 = point1
        x2, y2 = point2
        steps = max(abs(x2 - x1), abs(y2 - y1), 1)
        points = []

        for step in range(steps + 1):
            ratio = step / steps
            x = round(x1 + (x2 - x1) * ratio)
            y = round(y1 + (y2 - y1) * ratio)
            point = (x, y)
            if not points or points[-1] != point:
                points.append(point)

        return points

    def get_route_points(self, drone):
        route_points = []

        for i in range(len(drone.path) - 1):
            segment_points = self.sample_segment(drone.path[i], drone.path[i + 1])

            for point in segment_points:
                if not route_points or route_points[-1] != point:
                    route_points.append(point)

        return route_points

    def calculate_fitness(self, drone):
        route_points = self.get_route_points(drone)

        total_distance = 0
        for i in range(len(drone.path) - 1):
            total_distance += self.distance(drone.path[i], drone.path[i + 1])

        # Target is detected if any sampled route point is within sensor range.
        detection_range = drone.sensor_power / 20.0
        detected_targets = 0

        for target in self.environment.targets:
            target_found = False
            for point in route_points:
                if self.distance(target, point) <= detection_range:
                    target_found = True
                    break
            if target_found:
                detected_targets += 1

        # Count sampled points that cross obstacles or hazards.
        obstacle_hits = 0
        hazard_hits = 0

        for point in route_points:
            if self.environment.is_obstacle(point):
                obstacle_hits += 1
            if point in self.environment.hazards:
                hazard_hits += 1

        flight_time = total_distance / max(drone.speed, 1)
        energy = total_distance * (1 + drone.speed * 0.1) + (
            drone.sensor_power * flight_time / 100.0
        )

        fitness = (
            detected_targets * 100
            - total_distance * 1.0
            - flight_time * 1.5
            - energy * 0.5
            - obstacle_hits * 100
            - hazard_hits * 25
        )

        if total_distance > self.environment.max_flight_range:
            fitness -= (total_distance - self.environment.max_flight_range) * 5

        return fitness

    def create_population(self):
        population = []
        for i in range(self.population_size):
            population.append(Drone.create_random(self.environment))
        return population

    def select_parent(self, population):
        # Tournament selection: choose the best of three random candidates.
        candidates = random.sample(population, min(3, len(population)))
        selected = candidates[0]

        for drone in candidates:
            if self.calculate_fitness(drone) > self.calculate_fitness(selected):
                selected = drone

        return selected

    def crossover(self, parent1, parent2):
        # Combine waypoints from two parents at a random crossover point.
        child_path = [self.environment.start]
        middle_count = len(parent1.path) - 2
        split = random.randint(1, middle_count)

        for i in range(1, split + 1):
            child_path.append(parent1.path[i])

        for i in range(split + 1, len(parent2.path) - 1):
            child_path.append(parent2.path[i])

        child_path.append(self.environment.destination)

        child_speed = random.choice([parent1.speed, parent2.speed])
        child_sensor = random.choice([parent1.sensor_power, parent2.sensor_power])

        return Drone(child_path, child_speed, child_sensor)

    def mutate(self, drone):
        mutated = drone.copy()

        # Change intermediate waypoint coordinates
        for i in range(1, len(mutated.path) - 1):
            if random.random() < self.mutation_rate:
                x = random.randint(0, self.environment.size - 1)
                y = random.randint(0, self.environment.size - 1)
                mutated.path[i] = (x, y)

        # Sometimes change speed and sensor power too
        if random.random() < self.mutation_rate:
            mutated.speed = random.randint(1, 5)

        if random.random() < self.mutation_rate:
            mutated.sensor_power = random.randint(20, 100)

        return mutated

    def run(self):
        population = self.create_population()

        for generation in range(self.generations):
            scored_population = []

            for drone in population:
                score = self.calculate_fitness(drone)
                scored_population.append((score, drone))

            scored_population.sort(key=lambda item: item[0], reverse=True)

            best_score = scored_population[0][0]
            average_score = sum(item[0] for item in scored_population) / len(scored_population)

            self.best_fitness_history.append(best_score)
            self.average_fitness_history.append(average_score)
            self.best_drone = scored_population[0][1].copy()

            print(
                "Generation", generation + 1,
                "| Best Fitness:", round(best_score, 2),
                "| Average Fitness:", round(average_score, 2)
            )

            # Keep the best chromosomes unchanged (elitism)
            new_population = []
            for i in range(min(self.elite_count, len(scored_population))):
                new_population.append(scored_population[i][1].copy())

            while len(new_population) < self.population_size:
                parent1 = self.select_parent(population)
                parent2 = self.select_parent(population)
                child = self.crossover(parent1, parent2)
                child = self.mutate(child)
                new_population.append(child)

            population = new_population

        # Find best chromosome in the final population too
        final_drone = max(population, key=self.calculate_fitness)
        if self.calculate_fitness(final_drone) > self.calculate_fitness(self.best_drone):
            self.best_drone = final_drone.copy()

        return self.best_drone

    def plot_fitness(self):
        plt.figure(figsize=(9, 5))
        plt.plot(self.best_fitness_history, label="Best Fitness")
        plt.plot(self.average_fitness_history, label="Average Fitness")
        plt.title("Fitness Across Generations")
        plt.xlabel("Generation")
        plt.ylabel("Fitness Score")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()

    def plot_path(self, drone):
        grid = self.environment.grid
        plt.figure(figsize=(8, 8))
        plt.imshow(grid, cmap="Greys", origin="upper")

        # Draw route segments
        x_values = [point[0] for point in drone.path]
        y_values = [point[1] for point in drone.path]
        plt.plot(x_values, y_values, marker="o", linewidth=2, label="Evolved UAV Path")

        for x, y in self.environment.hazards:
            plt.scatter(x, y, color="orange", marker="s", s=100, label="Hazard")
        for x, y in self.environment.targets:
            plt.scatter(x, y, color="red", marker="*", s=180, label="Target")

        plt.scatter(*self.environment.start, color="blue", marker="o", s=100, label="Start")
        plt.scatter(*self.environment.destination, color="green", marker="X", s=100, label="Destination")

        plt.title("Best Evolved UAV Path")
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.xlim(-0.5, self.environment.size - 0.5)
        plt.ylim(self.environment.size - 0.5, -0.5)
        plt.grid(True, alpha=0.25)
        handles, labels = plt.gca().get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        plt.legend(unique.values(), unique.keys(), loc="upper left", bbox_to_anchor=(1, 1))
        plt.tight_layout()
        plt.show()

    def plot_coverage_heatmap(self, drone):
        coverage = np.zeros((self.environment.size, self.environment.size))
        route_points = self.get_route_points(drone)
        radius = drone.sensor_power / 20.0

        for x in range(self.environment.size):
            for y in range(self.environment.size):
                cell = (x, y)
                for route_point in route_points:
                    if self.distance(cell, route_point) <= radius:
                        coverage[y, x] = 1
                        break

        plt.figure(figsize=(8, 8))
        plt.imshow(coverage, cmap="YlOrRd", origin="upper", alpha=0.8)
        plt.imshow(self.environment.grid, cmap="Greys", origin="upper", alpha=0.35)

        for x, y in self.environment.targets:
            plt.scatter(x, y, color="blue", marker="*", s=180, label="Target")

        plt.title("Coverage Heatmap of Best Evolved Path")
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.xlim(-0.5, self.environment.size - 0.5)
        plt.ylim(self.environment.size - 0.5, -0.5)
        plt.grid(True, alpha=0.2)
        handles, labels = plt.gca().get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        plt.legend(unique.values(), unique.keys(), loc="upper left", bbox_to_anchor=(1, 1))
        plt.tight_layout()
        plt.show()

    def compare_with_random_search(self, random_trials=100):
        random_best = None
        random_best_score = float("-inf")

        for i in range(random_trials):
            drone = Drone.create_random(self.environment)
            score = self.calculate_fitness(drone)

            if score > random_best_score:
                random_best_score = score
                random_best = drone

        ga_score = self.calculate_fitness(self.best_drone)

        print("\nComparison with Random Search")
        print("Random Search Best Fitness:", round(random_best_score, 2))
        print("Genetic Algorithm Best Fitness:", round(ga_score, 2))

        if ga_score > random_best_score:
            print("Result: Genetic Algorithm found a better solution in this run.")
        elif ga_score < random_best_score:
            print("Result: Random Search found a better solution in this run.")
        else:
            print("Result: Both methods found the same fitness score.")

        return random_best, random_best_score
