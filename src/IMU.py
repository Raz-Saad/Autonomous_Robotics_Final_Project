
class IMU:
    def __init__(self, start_orientation = 0):
        self.drone_orientation = start_orientation
    
    def update_orientation (self,updated_orientation):
        self.drone_orientation = updated_orientation