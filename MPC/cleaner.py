
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
        self.states = [self.pos3d, self.battery_level, self.is_cleaning, self.is_charging, self.on_window]
        self.last_update_time = 0.0
    def update_states(self):
        self.states = [self.pos3d, self.battery_level, self.is_cleaning, self.is_charging, self.on_window]
