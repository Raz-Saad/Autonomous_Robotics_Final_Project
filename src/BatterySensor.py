
class BatterySensor:

    def __init__(self , battery_precentage=100):
        self.battery_precentage = battery_precentage 
        # we update the value 10 times per sec , and the battrey life is 480 secs so we will decrease this value by 1 tick per update
        self.amount_of_decisecond_drone_can_fly = 4800 # 480 sec * 10
        self.remaining_ticks_to_battrey_life = self.amount_of_decisecond_drone_can_fly 
    '''
    this function is being called 10 times per second, in each call we decrese the remaining ticks by 1
    and update the battery precentage accordingly
    '''
    def update_battrey_precentage(self):
        if(self.battery_precentage != 0):
            self.remaining_ticks_to_battrey_life -= 1
            self.battery_precentage = (self.remaining_ticks_to_battrey_life / self.amount_of_decisecond_drone_can_fly) * 100 # update the battery precentage
        
    def get_battrey_precentage(self):
        return self.battery_precentage
    
    def reset_battrey(self):
        self.battery_precentage = 100
        self.remaining_ticks_to_battrey_life = self.amount_of_decisecond_drone_can_fly
