class PIDController:
    def __init__(self, p, i, d, max_i):
        self.P = p
        self.I = i
        self.D = d
        self.max_i = max_i
        self.integral = 0
        self.last_error = 0
        self.first_run = True

    def constrain(self, value, max_value, min_value):
        if value > max_value:
            return max_value
        elif value < min_value:
            return min_value
        else:
            return value

    def update(self, error, dt):
        if self.first_run:
            self.last_error = error
            self.first_run = False

        self.integral += self.I * error * dt
        diff = (error - self.last_error) / dt if dt != 0 else 0
        const_integral = self.constrain(self.integral, self.max_i, -self.max_i)
        control_out = self.P * error + self.D * diff + const_integral
        self.last_error = error
        return control_out
    
    def update_P_value(self,value):
        self.P += value
        
    def update_I_value(self,value):
        self.I += value

    def update_D_value(self,value):
        self.D += value        