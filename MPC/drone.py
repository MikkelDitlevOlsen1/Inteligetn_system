import numpy as np
from cleaner import Robot_cleaner

class Transport_drone:
    def __init__(self, init_state):
        self.pos3d = np.array(init_state[0:3], dtype=np.float64)  # (x, y, z) position
        self.orentation = np.array(init_state[3:6], dtype=np.float64)  # [vx, vy, vz]
        self.load = None  #| Robot_cleaner  # current load being carried by the drone (None if no load)
        self.battery_capacity = 100.0  # maximum battery capacity
        self.battery_level = 100.0  # current battery level
        self.ucupied = False  # whether the drone is currently occupied with an action
        self.is_moving = False  # whether the drone is currently moving
        
        self.states = [self.pos3d, self.load, self.battery_level]

    def update_states(self):
        self.states = [self.pos3d, self.load, self.battery_level]