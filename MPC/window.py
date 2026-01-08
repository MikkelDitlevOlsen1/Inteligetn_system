import numpy as np
class Window:
    def __init__(self, pos3d, width, height, state,cleaing_time, name=None):
        self.name=name
        self.pos3d = np.array(pos3d, dtype=np.float64)  # (x, y, z) position of the window
        self.width = width
        self.height = height
        self.state = state  # 'clean' or 'dirty'
        self.cleaning_time = cleaing_time  # time required to clean the window
        self.this_id = id(self)

