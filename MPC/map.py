import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from drone import Transport_drone
from cleaner import Robot_cleaner
from basestation import Base_station
from window import Window
from matplotlib.animation import FuncAnimation

class Map:
    def __init__(self, base_station: Base_station, drone: Transport_drone, cleaners: list[Robot_cleaner], windows: list[Window]):
        self.base_station = base_station
        self.drone = drone
        self.cleaners = cleaners
        self.windows = windows
        self.time = 0.0  # simulation time

    def visualize(self):
        # 3D scatter of all map objects: base station, drone, cleaners, windows
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')

        # Base station
        try:
            bx, by, bz = tuple(self.base_station.pos3d)
            ax.scatter(bx, by, bz, c='black', marker='s', s=80, label='Base Station')
            ax.text(bx, by, bz, ' Base', color='black')
        except Exception:
            pass

        # Drone
        try:
            dx, dy, dz = tuple(self.drone.pos3d)
            ax.scatter(dx, dy, dz, c='red', marker='^', s=60, label='Drone')
            ax.text(dx, dy, dz, ' Drone', color='red')
        except Exception:
            pass

        # Cleaners
        cx_list = []
        cy_list = []
        cz_list = []
        for cleaner in self.cleaners:
            pos = cleaner.pos3d
            if pos is None:
                continue
            try:
                x, y, z = tuple(pos)
            except Exception:
                continue
            cx_list.append(x)
            cy_list.append(y)
            cz_list.append(z)
            # label each cleaner
            ax.text(x, y, z, f' {getattr(cleaner, "name", "C")}', color='blue')
        if cx_list:
            ax.scatter(cx_list, cy_list, cz_list, c='blue', marker='o', s=40, label='Cleaners')

        # Windows
        wx_list = []
        wy_list = []
        wz_list = []
        for window in self.windows:
            pos = getattr(window, 'pos3d', None)
            if pos is None:
                continue
            try:
                x, y, z = tuple(pos)
            except Exception:
                continue
            wx_list.append(x)
            wy_list.append(y)
            wz_list.append(z)
            # color by state if available
            state = getattr(window, 'state', None)
            lbl = f'W({state})' if state is not None else 'Window'
            ax.text(x, y, z, f' {lbl}', color='green')
        if wx_list:
            ax.scatter(wx_list, wy_list, wz_list, c='green', marker='x', s=40, label='Windows')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        ax.grid(True)
        plt.tight_layout()
        plt.show()


def random_map_generater(num_cleaners, num_windows):
    base_station = Base_station(pos3d=(0, 0, 0))

    drone = Transport_drone(init_state=[0, 0, 0, 0, 0, 0])

    cleaners = []
    for i in range(num_cleaners):
        cleaner = Robot_cleaner(name=f"{i+1}", battery_capacity=100, pos3d=(0, 0, 0))
        cleaners.append(cleaner)

    windows = []
    for i in range(num_windows):
        pos3d = (np.random.uniform(-50, 50), np.random.uniform(-50, 50), np.random.uniform(10, 100))
        width = np.random.uniform(1, 5)
        height = np.random.uniform(1, 5)
        state = np.random.choice(['clean', 'dirty'])
        cleaning_time = np.random.uniform(5, 20)  # time required to clean the window
        window = Window(pos3d=pos3d, width=width, height=height, state=state, cleaing_time=cleaning_time,name=f"{i+1}")
        windows.append(window)

    return Map(base_station=base_station, drone=drone, cleaners=cleaners, windows=windows)


if __name__ == "__main__":
    test_map = random_map_generater(num_cleaners=5, num_windows=10)
