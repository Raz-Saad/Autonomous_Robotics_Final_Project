class OpticalFlow:
    def __init__(self):
        self.acceleration = 1  # Acceleration rate in meters per second squared
        self.max_speed = 3  # Maximum speed in meters per second
        self.current_speed = 0  # Current speed in meters per second

    def update_speed_acceleration(self):
        # Update the speed based on acceleration until it reaches the maximum speed
        if self.current_speed < self.max_speed:
            self.current_speed += self.acceleration

    def update_speed_deceleration(self):
        # Update the speed based on acceleration until it reaches 0
        if self.current_speed > 0:
            self.current_speed -= self.acceleration    

    def get_current_speed(self):
        return self.current_speed    

    def reset_sensor(self):
        self.current_speed = 0