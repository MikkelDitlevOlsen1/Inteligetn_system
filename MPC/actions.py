from window import Window
from basestation import Base_station
from drone import Transport_drone
from cleaner import Robot_cleaner
import numpy as np
class actions:
    def __init__(self, name, duration, energy_cost ,target : Robot_cleaner| Transport_drone):
        self.name = name  # Name of the action
        self.duration = duration  # Duration of the action in seconds to sucsede
        self.energy_cost = energy_cost  # Energy cost of the action in joules
        self.target = target  # Target involved in the action the one that loses power
        self.start_time = 0  # Time when the action starts
        self.end_time = self.start_time + duration  # Time when the action ends
        
    def step_action(self, current_time, delta_t):
        self.aplay_action_power_in_deltat(delta_t)
        self.aplay_action_movment(delta_t)
        self.when_start()
        if self.is_done(current_time):
            self.when_done()
            return True

        return False

    def full_action(self , delta_t):
        self.when_start()
        repatisions=int(self.duration/delta_t)
        for _ in range(repatisions):
            self.aplay_action_power_in_deltat(delta_t)
            self.aplay_action_movment(delta_t)
        self.when_done()
        return repatisions

    
    def set_start_time(self, start_time):
        self.start_time = start_time
        self.end_time = start_time + self.duration
    
    def aplay_action_power(self):
        if self.duration == 0: #
            return
        self.target.battery_level -= self.energy_cost / self.target.battery_capacity * 100.0
        if self.target.battery_level < 0:
            self.target.battery_level = 0.0

    def aplay_action_power_in_deltat(self, delta_t):
        if self.duration == 0: #in idelsateS
            return
        energy_used = (self.energy_cost / self.duration) * delta_t
        self.target.battery_level -= energy_used / self.target.battery_capacity * 100.0
        if self.target.battery_level < 0:
            self.target.battery_level = 0.0

    def just_started(self, current_time):
        return current_time == self.start_time 

    def is_done(self, current_time):
        return current_time >= self.end_time
    
    def is_canceled(self, current_time):
        pass

    def when_start(self):
        print(f"Starting action : {self}")

    def when_done(self):
        print(f"Done whit action : {self}")
        

    def aplay_action_movment(self, delta_t):
        pass

    def is_alowed(self):
        pass

    def __str__(self):
        return self.name
    
class fly_to_window(actions):
    def __init__(self, window : Window, drone: Transport_drone , speed=5.0 , power_consumption=200.0):
        super().__init__('fly_to_window', 0, 0, drone)
        self.window = window  # ID of the target window
        self.drone = drone  # Reference to the drone performing the action
        self.speed = speed # Speed of the drone in m/s
        self.power_consumption = power_consumption  # Power consumption in watts/joules per second
        self.duration = self.calculate_duration()
        self.energy_cost = self.calculate_energy_cost()

    def __str__(self):
        return f"{self.name} {self.window.name}"
       
    def calculate_duration(self):
        distance = np.linalg.norm(self.drone.pos3d - self.window.pos3d)
        return distance / self.speed

    def calculate_energy_cost(self):
        return self.duration * self.power_consumption

    def aplay_action_movment(self, delta_t):
        direction = self.window.pos3d - self.drone.pos3d
        distance_to_travel = self.speed * delta_t
        distance_to_window = np.linalg.norm(direction)
        if distance_to_travel >= distance_to_window:
            self.drone.pos3d = self.window.pos3d.copy()
        else:
            direction_normalized = direction / distance_to_window
            self.drone.pos3d += direction_normalized * distance_to_travel

        if self.drone.load is not None:
            #print(self.drone.load)
            self.drone.load.pos3d = self.drone.pos3d.copy()

    def when_done(self):
        super().when_done()
        self.drone.is_moving = False

    def when_start(self):
        self.drone.is_moving = True

    def is_alowed(self):
        if np.array_equal(self.drone.pos3d, self.window.pos3d):
            return False
        return self.drone.ucupied is False
    
class clean_window(actions):
    def __init__(self, cleaner: Robot_cleaner , power_consumption=100.0):
        super().__init__('clean_window', 0, 0, cleaner)
        self.cleaner = cleaner  # Reference to the cleaner performing the action
        self.power_consumption = power_consumption  # Power consumption in watts/joules per second
        if self.cleaner.on_window is None:
            self.window = None
            self.duration = 0
            self.energy_cost = 0
        else:
            self.window = self.cleaner.on_window  # Target window to be cleaned
            self.duration = self.cleaner.on_window.cleaning_time
            self.energy_cost = self.calculate_energy_cost()
        
    def calculate_energy_cost(self):
        return self.duration * self.power_consumption
    
    def when_start(self):
        super().when_done()

        self.cleaner.is_cleaning = True

    def when_done(self):
        self.cleaner.is_cleaning = False
        self.window.state = 'clean'
    
    def is_alowed(self):
        
        if self.window is None:
            return False
        if self.cleaner.battery_level <= 40.0:
            return False
        if not np.array_equal(self.cleaner.pos3d, self.window.pos3d):
            print("there is a error if you see this message ")
            return False
        return self.window.state == 'dirty'
    
class pickup_cleaner(actions):
    def __init__(self, cleaner: Robot_cleaner, drone: Transport_drone, power_consumption=50.0 , pickup_duration=10.0):
        super().__init__('pickup_cleaner', 0, 0, drone)
        self.cleaner = cleaner  # ID of the target cleaner
        self.drone = drone  # Reference to the drone performing the action
        self.power_consumption = power_consumption  # Power consumption in watts/joules per second
        self.duration = pickup_duration  # Fixed duration for pickup action
        self.energy_cost = self.calculate_energy_cost()
        
    def calculate_energy_cost(self):
        return self.duration * self.power_consumption

    def when_done(self):
        super().when_done()
        self.cleaner.is_cleaning = False
        self.drone.load = self.cleaner
        self.cleaner.pos3d = self.drone.pos3d.copy()
        self.cleaner.on_window = None
        self.drone.ucupied = False

    def when_start(self):
        self.drone.ucupied = True

    def is_alowed(self):
        if self.drone.ucupied:
            return False
        if not np.array_equal(self.drone.pos3d, self.cleaner.pos3d): #TODO change to be close to instet of the same
            return False
        if self.cleaner.on_window is None:
            return False
        return self.drone.load is None 
    
    def __str__(self):
        return f"{self.name} {self.cleaner.name}"
    
class dropoff_cleaner(actions):
    def __init__(self, drone: Transport_drone ,window: Window,   power_consumption=50.0 , drop_duration=10.0):
        super().__init__('dropoff_cleaner', 0, 0, drone)
        self.drone = drone  # Reference to the drone performing the action
        self.window = window  # Target window to drop off the cleaner
        self.power_consumption = power_consumption  # Power consumption in watts/joules per second
        self.duration = drop_duration  # Fixed duration for dropoff action
        self.energy_cost = self.calculate_energy_cost()
        if self.drone.load is not None:
            self.cleaner = self.drone.load
        
    def calculate_energy_cost(self):
        return self.duration * self.power_consumption
    
    def when_done(self):
        super().when_done()

        self.drone.load = None
        self.cleaner.pos3d = self.window.pos3d.copy()
        self.cleaner.on_window = self.window
        self.drone.ucupied = False
        self.cleaner = None

    def when_start(self):
        self.drone.ucupied = True

    def is_alowed(self):
        if self.drone.ucupied:
            return False
        if not np.array_equal(self.drone.pos3d, self.window.pos3d): #TODO change to be close to instet of the same
            return False
        if self.drone.load is None:
            return False
        self.cleaner = self.drone.load
        return True
    

    def __str__(self):
        return f"{self.name} {self.cleaner.name}"
    
class pickup_clean_at_base(actions):
    def __init__(self, cleaner: Robot_cleaner, drone: Transport_drone, base_station: Base_station, power_consumption=50.0, pickup_duration=2.0):
        super().__init__('pickup_at_base_cleaner', 0, 0, drone)
        self.cleaner = cleaner
        self.drone = drone
        self.base_station = base_station
        self.power_consumption = power_consumption
        self.duration =pickup_duration
        self.energy_cost = self.calculate_energy_cost()

    def calculate_energy_cost(self):
        return self.duration * self.power_consumption

    def when_start(self):
        
        self.drone.ucupied = True

    def when_done(self):
        super().when_done()

        self.drone.load = self.cleaner
        self.cleaner.pos3d = self.drone.pos3d.copy()
        self.cleaner.on_window = None
        self.drone.ucupied = False

    def is_alowed(self):
        if self.drone.ucupied:
            return False
        if not np.array_equal(self.drone.pos3d, self.base_station.pos3d):
            return False
        if not np.array_equal(self.cleaner.pos3d, self.base_station.pos3d):
            return False
        if self.drone.load is not None:
            return False
        return True
    
    def __str__(self):
        return f"{self.name} {self.cleaner.name}"

class dropoff_clean_at_base(actions):
    def __init__(self, drone: Transport_drone, base_station: Base_station, power_consumption=50.0 , drop_duration=2.0):
        super().__init__('dropoff_clean_at_base', 0, 0, drone)
        self.drone = drone
        self.base_station = base_station
        self.power_consumption = power_consumption
        self.duration = drop_duration
        self.energy_cost = self.calculate_energy_cost()
        if self.drone.load is not None:
            self.cleaner = self.drone.load

    def calculate_energy_cost(self):
        return self.duration * self.power_consumption

    def when_start(self):
        self.drone.ucupied = True

    def when_done(self):
        super().when_done()

        self.drone.load = None
        self.cleaner.pos3d = self.base_station.pos3d.copy()
        self.cleaner.on_window = None
        self.drone.ucupied = False
        self.cleaner = None
    
    def is_alowed(self):
        if self.drone.ucupied:
            return False
        if not np.array_equal(self.drone.pos3d, self.base_station.pos3d):
            return False
        if self.drone.load is None:
            return False
        self.cleaner = self.drone.load
        return True
       
class return_to_base(actions):
    def __init__(self, base_station: Base_station, drone: Transport_drone , speed=5.0 , power_consumption=200.0):
        super().__init__('return_to_base', 0, 0, drone)
        self.base_station = base_station  # ID of the base station
        self.drone = drone  # Reference to the drone performing the action
        self.speed = speed # Speed of the drone in m/s
        self.power_consumption = power_consumption  # Power consumption in watts/joules per second
        self.duration = self.calculate_duration()
        self.energy_cost = self.calculate_energy_cost()
        
    def calculate_duration(self):
        distance = np.linalg.norm(self.drone.pos3d - self.base_station.pos3d)
        return distance / self.speed

    def calculate_energy_cost(self):
        return self.duration * self.power_consumption
    

    def aplay_action_movment(self, delta_t):
        direction = self.base_station.pos3d - self.drone.pos3d
        distance_to_travel = self.speed * delta_t
        distance_to_window = np.linalg.norm(direction)
        
        if distance_to_travel >= distance_to_window:
            self.drone.pos3d = self.base_station.pos3d.copy()
        else:
            direction_normalized = direction / distance_to_window
            self.drone.pos3d += direction_normalized * distance_to_travel

        if self.drone.load is not None:
            self.drone.load.pos3d = self.drone.pos3d.copy()
        
    def is_alowed(self):
        if self.drone.ucupied:
            return False
        return True
    
class charge_cleaner(actions):
    def __init__(self, cleaner: Robot_cleaner,base_station: Base_station, charge_rate=10.0 ):
        super().__init__('charge_cleaner', 0, 0, cleaner)
        self.cleaner = cleaner  # ID of the target cleaner
        self.charge_rate = charge_rate  #joules per second
        self.base_station = base_station
        self.duration = self.calculate_duration()
        self.energy_cost = self.calculate_energy_cost()
        
    def calculate_duration(self):
        energy_needed = self.cleaner.battery_capacity - self.cleaner.battery_level / 100.0 * self.cleaner.battery_capacity
        return energy_needed / self.charge_rate

    def calculate_energy_cost(self):
        return -self.duration * self.charge_rate  # Negative because charging adds energy
    
    def when_start(self):
        self.cleaner.is_charging = True
    
    def when_done(self):
        #super().when_done()

        self.cleaner.is_charging = False

    def is_alowed(self):
        return np.array_equal(self.cleaner.pos3d, self.base_station.pos3d)

class charge_drone(actions):
    def __init__(self, drone: Transport_drone, base_station: Base_station, charge_rate=20.0 ):
        super().__init__('charge_drone', 0, 0, drone)
        self.drone = drone  # ID of the target drone
        self.charge_rate = charge_rate  #joules per second
        self.base_station = base_station
        self.duration = self.calculate_duration()
        self.energy_cost = self.calculate_energy_cost()
        
    def calculate_duration(self):
        energy_needed = self.drone.battery_capacity - self.drone.battery_level / 100.0 * self.drone.battery_capacity
        duration=energy_needed / self.charge_rate

        return duration if duration > 2.0 else 2.0  # Minimum duration of 2 seconds

    def calculate_energy_cost(self):
        return -self.duration * self.charge_rate  # Negative because charging adds energy
    
    def when_start(self):
        pass

    def when_done(self):
        super().when_done()
        

    def is_alowed(self):
        if self.drone.ucupied:
            return False
        if self.drone.load:
            return False
        return np.array_equal(self.drone.pos3d, self.base_station.pos3d)
    

class null_action(actions):
    def __init__(self, target: Transport_drone| Robot_cleaner):
        super().__init__('null_action', 0, 0, target)
        
    def is_alowed(self):
        return True
    
    def when_done(self):
        pass

    def when_start(self):
        pass    
    

