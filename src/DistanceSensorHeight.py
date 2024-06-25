import math

class DistanceSensorheight:
    def __init__(self, sensor_direction, distance=0):
        self.distance = distance  # the distance from an obstacle
        self.direction = sensor_direction  # up, down

    # update the current distance from an obstacle using the map and drone's position
    def update_values(self,drone_z_level,drone_radius, floor_level, ceiling_level, obstacles): # TODO: stopped here, fix this func
        max_range = 120  # 3 meters in pixels (3 meters / 0.025 meters per pixel)
        self.distance = max_range

        if self.direction == "up":
            # Check distance to ceiling
            distance_to_ceiling = ceiling_level - drone_z_level
            self.distance = min(distance_to_ceiling, max_range) # * 2.5 # 1 pixel = 2.5 cm
            # Check for obstacles in the upward direction
            # for obstacle in obstacles:
            #     _, _, _, _, height = obstacle
            #     if height > drone_z_level:  # Obstacle is above the drone
            #         distance_to_obstacle = height - drone_z_level
            #         self.distance = min(distance_to_obstacle, self.distance)

        elif self.direction == "down":
            # Check distance to floor
            distance_to_floor = drone_z_level - floor_level
            self.distance = min(distance_to_floor, max_range)
            # Check for obstacles in the downward direction
            for obstacle in obstacles:
                # if is colliding with obstacle in x,y axis
                _, _, _, _, height = obstacle
                if height < drone_z_level:  # Obstacle is below the drone
                    distance_to_obstacle = drone_z_level - height
                    self.distance = min(distance_to_obstacle, self.distance) # * 2.5 # 1 pixel = 2.5 cm