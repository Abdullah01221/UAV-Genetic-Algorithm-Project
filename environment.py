import numpy as np
import matplotlib.pyplot as plt


class Environment:
    def __init__(self):
        self.size = 20
        self.grid = np.zeros((self.size, self.size))

        # Obstacles: grid[y, x] = 1
        self.grid[4, 4] = 1
        self.grid[4, 5] = 1
        self.grid[4, 6] = 1
        self.grid[5, 6] = 1
        self.grid[6, 6] = 1

        self.grid[10, 12] = 1
        self.grid[11, 12] = 1
        self.grid[12, 12] = 1

        self.start = (0, 0)
        self.destination = (19, 19)

        # High-priority targets to detect
        self.targets = [(5, 15), (14, 4), (16, 12)]

        # Hazard cells are risky, but are not solid obstacles
        self.hazards = [(8, 8), (8, 9), (9, 8), (15, 15)]

        self.max_flight_range = 70

    def is_inside(self, point):
        x, y = point
        return 0 <= x < self.size and 0 <= y < self.size

    def is_obstacle(self, point):
        x, y = point
        if not self.is_inside(point):
            return True
        return self.grid[y, x] == 1

    def show(self):
        plt.figure(figsize=(8, 8))
        plt.imshow(self.grid, cmap="Greys", origin="upper")
        for x, y in self.hazards:
            plt.scatter(x, y, color="orange", marker="s", s=100, label="Hazard")
        for x, y in self.targets:
            plt.scatter(x, y, color="red", marker="*", s=180, label="Target")
        plt.scatter(*self.start, color="blue", marker="o", s=100, label="Start")
        plt.scatter(*self.destination, color="green", marker="X", s=100, label="Destination")
        plt.title("UAV Mission Environment")
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.xlim(-0.5, self.size - 0.5)
        plt.ylim(self.size - 0.5, -0.5)
        plt.grid(True, alpha=0.25)
        # Show each legend label once
        handles, labels = plt.gca().get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        plt.legend(unique.values(), unique.keys(), loc="upper left", bbox_to_anchor=(1, 1))
        plt.tight_layout()
        plt.show()
