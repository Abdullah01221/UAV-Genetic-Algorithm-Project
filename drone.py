import random


class Drone:
    """
    One chromosome/solution for the UAV mission.
    A chromosome contains a list of waypoints, speed, and sensor power.
    """

    def __init__(self, path, speed, sensor_power):
        self.path = path
        self.speed = speed
        self.sensor_power = sensor_power

    @staticmethod
    def create_random(environment, waypoint_count=8):
        path = [environment.start]

        # Random intermediate waypoints
        for i in range(waypoint_count):
            x = random.randint(0, environment.size - 1)
            y = random.randint(0, environment.size - 1)
            path.append((x, y))

        path.append(environment.destination)

        speed = random.randint(1, 5)
        sensor_power = random.randint(20, 100)

        return Drone(path, speed, sensor_power)

    def copy(self):
        return Drone(self.path[:], self.speed, self.sensor_power)

    def show(self):
        print("Path:", self.path)
        print("Speed:", self.speed)
        print("Sensor Power:", self.sensor_power)
