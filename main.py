import random
from environment import Environment
from genetic_algorithm import GeneticAlgorithm


def main():
    # Fixed seed makes a run reproducible. Change or remove this line for different results.
    random.seed(42)

    environment = Environment()

    print("Starting UAV Genetic Algorithm...")
    ga = GeneticAlgorithm(
        environment=environment,
        population_size=40,
        generations=60,
        mutation_rate=0.15,
        elite_count=2
    )

    best_drone = ga.run()

    print("\nBest Evolved UAV Chromosome")
    best_drone.show()
    print("Best Fitness:", round(ga.calculate_fitness(best_drone), 2))

    ga.plot_fitness()
    ga.plot_path(best_drone)
    ga.plot_coverage_heatmap(best_drone)

    ga.compare_with_random_search(random_trials=100)


if __name__ == "__main__":
    main()
