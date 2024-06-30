import math

class DistanceSensorheight:
    def __init__(self, sensor_direction, distance=0):
        self.distance = distance  # the distance from an obstacle
        self.direction = sensor_direction  # up, down

    # update the current distance from an obstacle using the map and drone's position
    def update_values(self,drone_z_level,drone_radius, floor_level, ceiling_level, obstacles):
        max_range = 120  # 3 meters in pixels (3 meters / 0.025 meters per pixel)
        self.distance = max_range

        if self.direction == "up":
            # Check distance to ceiling
            distance_to_ceiling = ceiling_level - drone_z_level
            self.distance = min(distance_to_ceiling, max_range)

        elif self.direction == "down":
            # Check distance to floor
            distance_to_floor = drone_z_level - floor_level
            self.distance = min(distance_to_floor, max_range)

            # Check for obstacles in the downward direction:
            for obstacle in obstacles:
                # if is colliding with obstacle in x,y axis:
                _, _, _, _, height = obstacle
                if height < drone_z_level:  # Obstacle is below the drone
                    distance_to_obstacle = drone_z_level - height
                    self.distance = min(distance_to_obstacle, self.distance)
