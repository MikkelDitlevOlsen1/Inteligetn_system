import numpy as np
from drone import Transport_drone
from cleaner import Robot_cleaner
from basestation import Base_station
from window import Window

# Matplotlib imports deferred to avoid numpy compatibility issues
matplotlib_available = False
class Map:
    def __init__(self, base_station: Base_station, drone: Transport_drone, cleaners: list[Robot_cleaner], windows: list[Window]):
        self.base_station = base_station
        self.max_cleaners = 4
        self.max_windows = 50
        self.drone = drone
        self.cleaners = cleaners
        self.windows = windows
        self.time = 0.0  # simulation time
        self.cleaning_processes : list = [None for _ in self.cleaners]  # track cleaning processes
        self.cleaner_suction_consumption=0.2 # battery consumption rate when cleaner is cleaning (units per time)
        self.states = []
        self.update_states()

    
    def is_done(self):
        """Check if all windows are clean or any cleaner has 0 battery."""
        all_clean = all(window.state == 'clean' for window in self.windows)
        any_dead_cleaner = any(cleaner.battery_level <= 0.0 for cleaner in self.cleaners)
        return all_clean or any_dead_cleaner
    
    def update_states(self):
        #time and drone states info
        self.states = [self.time ,self.drone.battery_level]
        self.states.extend(self.drone.pos3d.tolist())
        if self.drone.load is None:
            drone_load_list=[0 for _ in range(self.max_cleaners)]
        else:
            drone_load_list=[1 if cleaner.this_id == self.drone.load.this_id else 0 for cleaner in self.cleaners]
            drone_load_list.extend([0 for _ in range(self.max_cleaners - len(drone_load_list))])
        self.states.extend(drone_load_list)

        # Base station states info
        self.states.extend(self.base_station.pos3d.tolist())
        # Cleaners states info
        for cleaner in self.cleaners:
            self.states.extend(cleaner.pos3d.tolist() + [cleaner.battery_level, 1 if cleaner.is_cleaning else 0, 1 if cleaner.is_charging else 0])

            if cleaner.on_window is not None:
                cleaner_on_window_list=[1 if window.this_id == cleaner.on_window else 0 for window in self.windows]
                cleaner_on_window_list.extend([0 for _ in range(self.max_windows - len(cleaner_on_window_list))])
                self.states.extend(cleaner_on_window_list)
            else:
                self.states.extend([0 for _ in range(self.max_windows)])

        # dummy data for cleners not there 
        for _ in range(self.max_cleaners - len(self.cleaners)):
            self.states.extend([0.0, 0.0, 0.0, 0.0 ,0.0 ,0.0])
            self.states.extend([0 for _ in range(self.max_windows)])
        
        # Windows states info
        for window in self.windows:
            self.states.extend(window.pos3d.tolist() + [window.cleaning_time, 1 if window.state == 'clean' else 0])
        
        # dummy data for windows not there
        for _ in range(self.max_windows - len(self.windows)):
            self.states.extend([0.0, 0.0, 0.0, 0.0,0.0])




    def update_cleaning_processes(self, print_this=False):
        for i, process in enumerate(self.cleaning_processes):
            if process is not None:
                if process.end_time <= self.time:
                    process.when_done(self)
                    if print_this:
                        print(f"Cleaner {i} finished cleaning window {process} at time {self.time:.2f}")
                    self.cleaning_processes[i] = None
                else:
                    process.when_runed_time(self, self.time)
                    if print_this:
                        print(f"Cleaner {i} is cleaning window {process}, time left: {process.end_time - self.time:.2f}")
        
        for i, cleaner in enumerate(self.cleaners):
            if cleaner.on_window is not None:
                cleaner.battery_level -= self.cleaner_suction_consumption * (self.time - cleaner.last_update_time)
                if cleaner.battery_level < 0.0:
                    cleaner.battery_level = 0.0
                cleaner.last_update_time = self.time    
                if print_this:
                    pass
                    #print(f"Cleaner {i} cleaning window {cleaner.on_window.name}, battery level: {cleaner.battery_level:.2f}")
    
    def visualize(self):
        # Import matplotlib only when needed
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
        except ImportError:
            print("Error: matplotlib is required for visualization")
            return
        
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

    def visualize_from_map_states(self, map_states, max_cleaners=4, max_windows=50):
        """
        Visualize map from a flat state vector.
        
        State structure:
        - Time (1)
        - Drone battery (1)
        - Drone pos3d (3)
        - Drone load indicator (max_cleaners)
        - Base station pos3d (3)
        - For each cleaner (max_cleaners):
            - pos3d (3), battery (1), is_cleaning (1), is_charging (1), on_window indicator (max_windows)
        - For each window (max_windows):
            - pos3d (3), cleaning_time (1), is_clean (1)
        """
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
        except ImportError:
            print("Error: matplotlib is required for visualization")
            return
        
        # Parse the state vector
        idx = 0
        # Time and drone info
        time = map_states[idx]
        idx += 1
        drone_battery = map_states[idx]
        idx += 1
        drone_pos = map_states[idx:idx+3]
        idx += 3
        drone_load = map_states[idx:idx+max_cleaners]
        idx += max_cleaners
        
        # Base station
        base_pos = map_states[idx:idx+3]
        idx += 3
        
        # Cleaners
        cleaners_data = []
        for i in range(max_cleaners):
            cleaner_pos = map_states[idx:idx+3]
            idx += 3
            battery = map_states[idx]
            idx += 1
            is_cleaning = map_states[idx]
            idx += 1
            is_charging = map_states[idx]
            idx += 1
            on_window = map_states[idx:idx+max_windows]
            idx += max_windows
            
            # Only include if battery is non-zero (cleaner exists)
            if battery > 1:
                cleaners_data.append({
                    'pos': cleaner_pos,
                    'battery': battery,
                    'is_cleaning': is_cleaning,
                    'is_charging': is_charging,
                    'id': i+1
                })
        
        # Windows
        windows_data = []
        for i in range(max_windows):
            window_pos = map_states[idx:idx+3]
            idx += 3
            cleaning_time = map_states[idx]
            idx += 1
            is_clean = map_states[idx]
            idx += 1
            
            # Only include if position is non-zero (window exists)
            if any(abs(p) > 1e-6 for p in window_pos):
                windows_data.append({
                    'pos': window_pos,
                    'cleaning_time': cleaning_time,
                    'is_clean': is_clean,
                    'id': i+1
                })
        
        # Create 3D plot
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot base station
        ax.scatter(base_pos[0], base_pos[1], base_pos[2], 
                c='black', marker='s', s=100, label='Base Station')
        ax.text(base_pos[0], base_pos[1], base_pos[2], ' Base', color='black')
        
        # Plot drone
        ax.scatter(drone_pos[0], drone_pos[1], drone_pos[2], 
                c='red', marker='^', s=80, label='Drone')
        ax.text(drone_pos[0], drone_pos[1], drone_pos[2], 
                f' Drone (B:{drone_battery:.1f})', color='red')
        
        # Plot cleaners
        if cleaners_data:
            cx = [c['pos'][0] for c in cleaners_data]
            cy = [c['pos'][1] for c in cleaners_data]
            cz = [c['pos'][2] for c in cleaners_data]
            ax.scatter(cx, cy, cz, c='blue', marker='o', s=50, label='Cleaners')
            for c in cleaners_data:
                status = 'Cleaning' if c['is_cleaning'] else ('Charging' if c['is_charging'] else 'Idle')
                ax.text(c['pos'][0], c['pos'][1], c['pos'][2], 
                        f" C{c['id']}({status},B:{c['battery']:.1f})", 
                        color='blue', fontsize=8)
        
        # Plot windows
        if windows_data:
            wx = [w['pos'][0] for w in windows_data]
            wy = [w['pos'][1] for w in windows_data]
            wz = [w['pos'][2] for w in windows_data]
            
            # Color by state
            colors = ['green' if w['is_clean'] else 'orange' for w in windows_data]
            ax.scatter(wx, wy, wz, c=colors, marker='x', s=60, label='Windows')
            for w in windows_data:
                state_str = 'Clean' if w['is_clean'] else 'Dirty'
                ax.text(w['pos'][0], w['pos'][1], w['pos'][2], 
                        f" W{w['id']}({state_str})", 
                        color='green' if w['is_clean'] else 'orange', fontsize=8)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Map State at Time: {time:.2f}')
        ax.legend()
        ax.grid(True)
        plt.tight_layout()
        plt.show()


    def animate_map_states_list(self,map_states_list, actions_list, dt=0.5, max_cleaners=4, max_windows=50, block=True):
        """Live-update visualization for a list of map state vectors.

        Args:
            map_states_list: iterable of flat state vectors (see ``visualize_from_map_states`` structure).
            dt: pause duration between frames in seconds.
            max_cleaners: max cleaners encoded per state vector.
            max_windows: max windows encoded per state vector.
            block: when True, keep the window open after the last frame.
        """
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  # needed for 3D projection
        except ImportError:
            print("Error: matplotlib is required for visualization")
            return

        def parse_state(map_states):
            idx = 0

            # Time and drone info
            time = map_states[idx]
            idx += 1
            drone_battery = map_states[idx]
            idx += 1
            drone_pos = map_states[idx:idx+3]
            idx += 3
            drone_load = map_states[idx:idx+max_cleaners]
            idx += max_cleaners

            # Base station
            base_pos = map_states[idx:idx+3]
            idx += 3

            # Cleaners
            cleaners_data = []
            for i in range(max_cleaners):
                cleaner_pos = map_states[idx:idx+3]
                idx += 3
                battery = map_states[idx]
                idx += 1
                is_cleaning = map_states[idx]
                idx += 1
                is_charging = map_states[idx]
                idx += 1
                on_window = map_states[idx:idx+max_windows]
                idx += max_windows

                # Only include if battery is non-zero (cleaner exists)
                if battery > 1:
                    cleaners_data.append({
                        'pos': cleaner_pos,
                        'battery': battery,
                        'is_cleaning': is_cleaning,
                        'is_charging': is_charging,
                        'id': i+1
                    })

            # Windows
            windows_data = []
            for i in range(max_windows):
                window_pos = map_states[idx:idx+3]
                idx += 3
                cleaning_time = map_states[idx]
                idx += 1
                is_clean = map_states[idx]
                idx += 1

                if any(abs(p) > 1e-6 for p in window_pos):
                    windows_data.append({
                        'pos': window_pos,
                        'cleaning_time': cleaning_time,
                        'is_clean': is_clean,
                        'id': i+1
                    })

            return {
                'time': time,
                'drone_battery': drone_battery,
                'drone_pos': drone_pos,
                'drone_load': drone_load,
                'base_pos': base_pos,
                'cleaners': cleaners_data,
                'windows': windows_data,
            }

        plt.ion()
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        for map_states, actions in zip(map_states_list, actions_list):
            ax.cla()
            parsed = parse_state(map_states)

            base_pos = parsed['base_pos']
            drone_pos = parsed['drone_pos']
            drone_battery = parsed['drone_battery']
            cleaners_data = parsed['cleaners']
            windows_data = parsed['windows']
            current_time = parsed['time']
            
            # Base station
            ax.scatter(base_pos[0], base_pos[1], base_pos[2], c='black', marker='s', s=100, label='Base Station')
            ax.text(base_pos[0], base_pos[1], base_pos[2], ' Base', color='black')

            # Drone
            ax.scatter(drone_pos[0], drone_pos[1], drone_pos[2], c='red', marker='^', s=80, label='Drone')
            ax.text(drone_pos[0], drone_pos[1], drone_pos[2], f' Drone (B:{drone_battery:.1f})', color='red')

            # Cleaners
            if cleaners_data:
                cx = [c['pos'][0] for c in cleaners_data]
                cy = [c['pos'][1] for c in cleaners_data]
                cz = [c['pos'][2] for c in cleaners_data]
                ax.scatter(cx, cy, cz, c='blue', marker='o', s=50, label='Cleaners')
                for c in cleaners_data:
                    status = 'Cleaning' if c['is_cleaning'] else ('Charging' if c['is_charging'] else 'Idle')
                    ax.text(c['pos'][0], c['pos'][1], c['pos'][2], f" C{c['id']}({status},B:{c['battery']:.1f})", color='blue', fontsize=8)

            # Windows
            if windows_data:
                wx = [w['pos'][0] for w in windows_data]
                wy = [w['pos'][1] for w in windows_data]
                wz = [w['pos'][2] for w in windows_data]
                colors = ['green' if w['is_clean'] else 'orange' for w in windows_data]
                ax.scatter(wx, wy, wz, c=colors, marker='x', s=60, label='Windows')
                for w in windows_data:
                    state_str = 'Clean' if w['is_clean'] else 'Dirty'
                    ax.text(w['pos'][0], w['pos'][1], w['pos'][2], f" W{w['id']}({state_str})", color='green' if w['is_clean'] else 'orange', fontsize=8)

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(f'Map State at Time: {current_time:.2f} \n Action: {actions}')
            ax.legend()
            ax.grid(True)
            plt.tight_layout()
            plt.pause(dt)

        if block:
            plt.ioff()
            plt.show()
        else:
            plt.pause(0.001)




def random_map_generater(num_cleaners, num_windows):
    base_station = Base_station(pos3d=(0, 0, 0))

    drone = Transport_drone(init_state=[0, 0, 0, 0, 0, 0])

    cleaners = []
    for i in range(num_cleaners):
        cleaner = Robot_cleaner(name=f"{i+1}", battery_capacity=120, pos3d=(0, 0, 0))
        cleaners.append(cleaner)

    windows = []
    for i in range(num_windows):
        pos3d = (np.random.uniform(-50, 50), np.random.uniform(-50, 50), np.random.uniform(10, 100))
        width = np.random.uniform(1, 5)
        height = np.random.uniform(1, 5)
        state = np.random.choice(['dirty'])
        cleaning_time = np.random.uniform(5, 10)  # time required to clean the window
        window = Window(pos3d=pos3d, width=width, height=height, state=state, cleaing_time=cleaning_time,name=f"{i+1}")
        windows.append(window)

    return Map(base_station=base_station, drone=drone, cleaners=cleaners, windows=windows)

def map_1(num_cleaners,num_windows):
    base_station = Base_station(pos3d=(0, 0, 0))

    drone = Transport_drone(init_state=[0, 0, 0, 0, 0, 0])

    cleaners = []
    for i in range(num_cleaners):
        cleaner = Robot_cleaner(name=f"{i+1}", battery_capacity=120, pos3d=(0, 0, 0))
        cleaners.append(cleaner)

    windows = []
    for i in range(num_windows):
        pos3d = (10, np.random.uniform(0, 50), np.random.uniform(10, 100))
        width = np.random.uniform(1, 5)
        height = np.random.uniform(1, 5)
        state = np.random.choice(['dirty'])
        cleaning_time = np.random.uniform(5, 10)  # time required to clean the window
        window = Window(pos3d=pos3d, width=width, height=height, state=state, cleaing_time=cleaning_time,name=f"{i+1}")
        windows.append(window)
    
    return Map(base_station=base_station, drone=drone, cleaners=cleaners, windows=windows)

if __name__ == "__main__":
    test_map = random_map_generater(num_cleaners=5, num_windows=10)
