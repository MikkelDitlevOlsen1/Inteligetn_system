
import numpy as np
class Robot_cleaner:
    def __init__(self, name, battery_capacity, pos3d=(0, 0, 0)):
        self.name = name
        self.battery_capacity = battery_capacity
        self.battery_level = battery_capacity
        self.is_cleaning = False
        self.is_charging = False
        self.on_window = None
        self.this_id=id(self)
        self.pos3d = np.array(pos3d)    

    
