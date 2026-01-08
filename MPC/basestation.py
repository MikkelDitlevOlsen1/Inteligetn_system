import numpy as np
from cleaner import Robot_cleaner

class Base_station:
    def __init__(self, pos3d, ):
        self.pos3d = np.array(pos3d, dtype=np.float64)  # (x, y, z) position of the base station
        