import time
from environment import Environment
from genetic_algorithm import GeneticAlgorithm


class TestGeneticAlgorithm(GeneticAlgorithm):

    def __init__(self, *args, tournament_size=3, **kwargs):
        super().__init__(*args, **kwargs)
        self.tournament_size = tournament_size

    def select_parent(self, population):
        import random

        size = min(self.tournament_size, len(population))
        candidates = random.sample(population, size)

        selected = candidates[0]

        for drone in candidates:
            if self.calculate_fitness(drone) > self.calculate_fitness(selected):
                selected = drone

        return selected


def run_experiment(population_size, mutation_rate, tournament_size):

    env = Environment()

    ga = TestGeneticAlgorithm(
        environment=env,
        population_size=population_size,
        generations=30,
        mutation_rate=mutation_rate,
        elite_count=2,
        tournament_size=tournament_size
    )

    start_time = time.time()

    best_drone = ga.run()

    end_time = time.time()

    best_fitness = ga.calculate_fitness(best_drone)
    execution_time = end_time - start_time

    return best_fitness, execution_time


def main():

    experiments = [
        ("Population 20", 20, 0.15, 3),
        ("Population 40", 40, 0.15, 3),
        ("Population 80", 80, 0.15, 3),

        ("Mutation 0.05", 40, 0.05, 3),
        ("Mutation 0.15", 40, 0.15, 3),
        ("Mutation 0.30", 40, 0.30, 3),

        ("Tournament 2", 40, 0.15, 2),
        ("Tournament 3", 40, 0.15, 3),
        ("Tournament 5", 40, 0.15, 5)
    ]

    print("\nHYPERPARAMETER EXPERIMENT RESULTS")
    print("-" * 65)

    for name, population, mutation, tournament in experiments:

        print("\nRunning:", name)

        fitness, seconds = run_experiment(
            population,
            mutation,
            tournament
        )

        print("Experiment:", name)
        print("Best Fitness:", round(fitness, 2))
        print("Execution Time:", round(seconds, 2), "seconds")


if __name__ == "__main__":
    main()