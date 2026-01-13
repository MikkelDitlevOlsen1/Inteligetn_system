


import numpy as np
import copy
from map import Map
from window import Window
from basestation import Base_station
from drone import Transport_drone
from cleaner import Robot_cleaner


class MapAction:
    """Base class for actions that operate on Map state and run to completion."""
    
    def __init__(self, name):
        self.name = name
    
    def is_allowed(self, map_state: Map) -> bool:
        """Check if this action is allowed given the current map state."""
        raise NotImplementedError("Subclasses must implement is_allowed()")
    
    def run(self, map_state: Map) -> Map:
        """
        Execute the action to completion and return a copy of the map with updated state.
        
        Args:
            map_state: Current map state
            
        Returns:
            A deep copy of the map with updated drone states, cleaner states, 
            window states, and time
        """
        raise NotImplementedError("Subclasses must implement run()")
    
    def __str__(self):
        return self.name


class FlyToWindow(MapAction):
    """Drone flies to a target window."""
    
    def __init__(self, window_index: int, speed=5.0, power_consumption=200.0):
        super().__init__(f'fly_to_window_{window_index}')
        self.window_index = window_index
        self.speed = speed
        self.power_consumption = power_consumption
    
    def is_allowed(self, map_state: Map) -> bool:
        """Check if drone can fly to the window."""
        if self.window_index >= len(map_state.windows):
            return False
        
        window = map_state.windows[self.window_index]
        drone = map_state.drone
        
        # Already at the window
        if np.array_equal(drone.pos3d, window.pos3d):
            return False
        
        return True
    
    def run(self, map_state: Map) -> Map:
        """Execute full flight to window."""
        # Create deep copy of the map
        new_map = copy.deepcopy(map_state)
        
        window = new_map.windows[self.window_index]
        drone = new_map.drone
        
        # Calculate duration and energy cost
        distance = np.linalg.norm(drone.pos3d - window.pos3d)
        duration = distance / self.speed
        energy_cost = duration * self.power_consumption
        
        # Update drone position
        drone.pos3d = window.pos3d.copy()
        drone.is_moving = False
        
        # Update load position if carrying something
        if drone.load is not None:
            drone.load.pos3d = drone.pos3d.copy()
        
        # Update battery
        energy_used_percent = energy_cost / drone.battery_capacity * 100.0
        drone.battery_level -= energy_used_percent
        if drone.battery_level < 0:
            drone.battery_level = 0.0
        
        # Update time
        new_map.time += duration
        
        return new_map


class PickupCleaner(MapAction):
    """Drone picks up a cleaner from a window."""
    
    def __init__(self, cleaner_index: int, power_consumption=50.0, pickup_duration=10.0):
        super().__init__(f'pickup_cleaner_{cleaner_index}')
        self.cleaner_index = cleaner_index
        self.power_consumption = power_consumption
        self.duration = pickup_duration
    
    def is_allowed(self, map_state: Map) -> bool:
        """Check if drone can pick up the cleaner."""
        if self.cleaner_index >= len(map_state.cleaners):
            return False
        
        cleaner = map_state.cleaners[self.cleaner_index]
        drone = map_state.drone
        
        # Drone must be at cleaner's position
        if not np.array_equal(drone.pos3d, cleaner.pos3d):
            return False
        
        # Drone must not be carrying anything
        if drone.load is not None:
            return False
        
        # Drone must not be occupied
        if drone.ucupied:
            return False
                # Cleaner must not be cleaning
        if cleaner.is_cleaning:
            return False
        
        # Cleaner must be on a window
        if cleaner.on_window is None:
            return False
        
        return True
    
    def run(self, map_state: Map) -> Map:
        """Execute pickup action."""
        new_map = copy.deepcopy(map_state)
        
        cleaner = new_map.cleaners[self.cleaner_index]
        drone = new_map.drone
        
        # Calculate energy cost
        energy_cost = self.duration * self.power_consumption
        
        # Pick up cleaner
        cleaner.is_cleaning = False
        drone.load = cleaner
        cleaner.pos3d = drone.pos3d.copy()
        cleaner.on_window = None
        
        # Update battery
        energy_used_percent = energy_cost / drone.battery_capacity * 100.0
        drone.battery_level -= energy_used_percent
        if drone.battery_level < 0:
            drone.battery_level = 0.0
        
        # Update time
        new_map.time += self.duration
        
        return new_map


class DropoffCleaner(MapAction):
    """Drone drops off a cleaner at a window."""
    
    def __init__(self, window_index: int, power_consumption=50.0, drop_duration=10.0):
        super().__init__(f'dropoff_cleaner_at_window_{window_index}')
        self.window_index = window_index
        self.power_consumption = power_consumption
        self.duration = drop_duration
    
    def is_allowed(self, map_state: Map) -> bool:
        """Check if drone can drop off cleaner at window."""
        if self.window_index >= len(map_state.windows):
            return False
        
        window = map_state.windows[self.window_index]
        drone = map_state.drone
        
        # Drone must be at window position
        if not np.array_equal(drone.pos3d, window.pos3d):
            return False
        
        # Drone must be carrying a cleaner
        if drone.load is None:
            return False
        
        # Drone must not be occupied
        if drone.ucupied:
            return False
        
        if window.state == 'clean':
            return False
        
        return True
    
    def run(self, map_state: Map) -> Map:
        """Execute dropoff action."""
        new_map = copy.deepcopy(map_state)
        window = new_map.windows[self.window_index]
        #print("window pos:", window.pos3d)
        drone = new_map.drone
        
        # Calculate energy cost
        energy_cost = self.duration * self.power_consumption
        # Drop off cleaner
        cleaner = drone.load
        cleaner.pos3d = window.pos3d.copy()
        #print("here")
        cleaner.on_window = window
        drone.load = None

        # Update battery
        energy_used_percent = energy_cost / drone.battery_capacity * 100.0
        drone.battery_level -= energy_used_percent
        if drone.battery_level < 0:
            drone.battery_level = 0.0
        
        # Update time
        new_map.time += self.duration
        
        return new_map


class PickupCleanerAtBase(MapAction):
    """Drone picks up a cleaner from the base station."""
    
    def __init__(self, cleaner_index: int, power_consumption=50.0, pickup_duration=2.0):
        super().__init__(f'pickup_cleaner_at_base_{cleaner_index}')
        self.cleaner_index = cleaner_index
        self.power_consumption = power_consumption
        self.duration = pickup_duration
    
    def is_allowed(self, map_state: Map) -> bool:
        """Check if drone can pick up cleaner at base."""
        if self.cleaner_index >= len(map_state.cleaners):
            return False
        
        cleaner = map_state.cleaners[self.cleaner_index]
        drone = map_state.drone
        base = map_state.base_station
        
        # Drone must not be occupied
        if drone.ucupied:
            return False
        
        # Drone must be at base station
        if not np.array_equal(drone.pos3d, base.pos3d):
            return False
        
        # Cleaner must be at base station
        if not np.array_equal(cleaner.pos3d, base.pos3d):
            return False
        

        
        # Drone must not be carrying anything
        if drone.load is not None:
            return False

        return True
    
    def run(self, map_state: Map) -> Map:
        """Execute pickup at base action."""
        new_map = copy.deepcopy(map_state)
        
        cleaner = new_map.cleaners[self.cleaner_index]
        drone = new_map.drone
        
        # Calculate energy cost
        energy_cost = self.duration * self.power_consumption
        
        # Pick up cleaner
        drone.load = cleaner
        cleaner.pos3d = drone.pos3d.copy()
        cleaner.on_window = None
        
        # Update battery
        energy_used_percent = energy_cost / drone.battery_capacity * 100.0
        drone.battery_level -= energy_used_percent
        if drone.battery_level < 0:
            drone.battery_level = 0.0
        
        # Update time
        new_map.time += self.duration
        
        return new_map


class DropoffCleanerAtBase(MapAction):
    """Drone drops off cleaner at the base station."""
    
    def __init__(self, power_consumption=50.0, drop_duration=2.0):
        super().__init__('dropoff_cleaner_at_base')
        self.power_consumption = power_consumption
        self.duration = drop_duration
    
    def is_allowed(self, map_state: Map) -> bool:
        """Check if drone can drop off cleaner at base."""
        drone = map_state.drone
        base = map_state.base_station
        
        # Drone must not be occupied
        if drone.ucupied:
            return False
        
        # Drone must be at base station
        if not np.array_equal(drone.pos3d, base.pos3d):
            return False
        
        # Drone must be carrying a cleaner
        if drone.load is None:
            return False
        
        return True
    
    def run(self, map_state: Map) -> Map:
        """Execute dropoff at base action."""
        new_map = copy.deepcopy(map_state)
        
        drone = new_map.drone
        base = new_map.base_station
        
        # Calculate energy cost
        energy_cost = self.duration * self.power_consumption
        
        # Drop off cleaner
        cleaner = drone.load
        drone.load = None
        cleaner.pos3d = base.pos3d.copy()
        cleaner.on_window = None
        
        # Update battery
        energy_used_percent = energy_cost / drone.battery_capacity * 100.0
        drone.battery_level -= energy_used_percent
        if drone.battery_level < 0:
            drone.battery_level = 0.0
        
        # Update time
        new_map.time += self.duration
        
        return new_map


class ReturnToBase(MapAction):
    """Drone returns to base station."""
    
    def __init__(self, speed=5.0, power_consumption=200.0):
        super().__init__('return_to_base')
        self.speed = speed
        self.power_consumption = power_consumption
    
    def is_allowed(self, map_state: Map) -> bool:
        """Check if drone can return to base."""
        drone = map_state.drone
        base = map_state.base_station
        
        # Already at base
        if np.array_equal(drone.pos3d, base.pos3d):
            return False
        
        # Drone must not be occupied
        if drone.ucupied:
            return False
        
        return True
    
    def run(self, map_state: Map) -> Map:
        """Execute return to base action."""
        new_map = copy.deepcopy(map_state)
        
        drone = new_map.drone
        base = new_map.base_station
        
        # Calculate duration and energy cost
        distance = np.linalg.norm(drone.pos3d - base.pos3d)
        duration = distance / self.speed
        energy_cost = duration * self.power_consumption
        
        # Update drone position
        drone.pos3d = base.pos3d.copy()
        drone.is_moving = False
        
        # Update load position if carrying something
        if drone.load is not None:
            drone.load.pos3d = drone.pos3d.copy()
        
        # Update battery
        energy_used_percent = energy_cost / drone.battery_capacity * 100.0
        drone.battery_level -= energy_used_percent
        if drone.battery_level < 0:
            drone.battery_level = 0.0
        
        # Update time
        new_map.time += duration
        
        return new_map


class ChargeDrone(MapAction):
    """Drone charges at base station."""
    
    def __init__(self, charge_rate=20.0):
        super().__init__('charge_drone')
        self.charge_rate = charge_rate
    
    def is_allowed(self, map_state: Map) -> bool:
        """Check if drone can charge."""
        drone = map_state.drone
        base = map_state.base_station
        
        # Drone must be at base station
        if not np.array_equal(drone.pos3d, base.pos3d):
            return False
        
        # Drone must not be occupied
        if drone.ucupied:
            return False
        
        # Drone must not be carrying anything
        if drone.load is not None:
            return False
        
        # Battery must not be full
        if drone.battery_level >= 99.9:
            return False
        
        return True
    
    def run(self, map_state: Map) -> Map:
        """Execute charging action."""
        new_map = copy.deepcopy(map_state)
        
        drone = new_map.drone
        
        # Calculate duration to fully charge
        current_energy = drone.battery_level / 100.0 * drone.battery_capacity
        energy_needed = drone.battery_capacity - current_energy
        duration = energy_needed / self.charge_rate
        
        # Minimum duration of 2 seconds
        if duration < 2.0:
            duration = 2.0
        
        # Charge to full
        drone.battery_level = 100.0
        
        # Update time
        new_map.time += duration
        
        return new_map


class NullAction(MapAction):
    """No-op action that does nothing."""
    
    def __init__(self):
        super().__init__('null_action')
    
    def is_allowed(self, map_state: Map) -> bool:
        """Always allowed."""
        return True
    
    def run(self, map_state: Map) -> Map:
        """Return map unchanged."""
        return copy.deepcopy(map_state)





class DropCleanerOffAtWindow(MapAction):
    """Drone flies to a window and drops off a cleaner at that window in one action."""
    def __init__(self, window_index: int, speed=5.0, fly_power=200.0, drop_power=50.0, drop_duration=10.0):
        super().__init__(f'drop_cleaner_off_at_window_{window_index}')
        self.window_index = window_index
        self.speed = speed
        self.fly_power = fly_power
        self.drop_power = drop_power
        self.drop_duration = drop_duration

    def is_allowed(self, map_state: Map) -> bool:
        # Check if both flying to window and dropping off cleaner are allowed
        if self.window_index >= len(map_state.windows):
            return False
        window = map_state.windows[self.window_index]
        drone = map_state.drone
        # Must be carrying a cleaner
        if drone.load is None:
            return False
        # Drone must not be occupied
        if drone.ucupied:
            return False
        # Window must not be clean
        if window.state == 'clean':
            return False
        return True

    def run(self, map_state: Map) -> Map:
        # Deep copy map
        new_map = copy.deepcopy(map_state)
        window = new_map.windows[self.window_index]
        drone = new_map.drone

        # 1. Fly to window if not already there
        if not np.array_equal(drone.pos3d, window.pos3d):
            distance = np.linalg.norm(drone.pos3d - window.pos3d)
            duration = distance / self.speed
            energy_cost = duration * self.fly_power
            drone.pos3d = window.pos3d.copy()
            drone.is_moving = False
            # Move load with drone
            if drone.load is not None:
                drone.load.pos3d = drone.pos3d.copy()
            # Battery
            energy_used_percent = energy_cost / drone.battery_capacity * 100.0
            drone.battery_level -= energy_used_percent
            if drone.battery_level < 0:
                drone.battery_level = 0.0
            # Time
            new_map.time += duration

        # 2. Drop off cleaner
        # Only drop if at window
        if np.array_equal(drone.pos3d, window.pos3d) and drone.load is not None:
            energy_cost = self.drop_duration * self.drop_power
            cleaner = drone.load
            cleaner.pos3d = window.pos3d.copy()
            cleaner.on_window = window
            drone.load = None
            # Battery
            energy_used_percent = energy_cost / drone.battery_capacity * 100.0
            drone.battery_level -= energy_used_percent
            if drone.battery_level < 0:
                drone.battery_level = 0.0
            # Time
            new_map.time += self.drop_duration

        return new_map
    


# Composite action: Fly to cleaner and pick it up
class PickupCleanerByFlying(MapAction):
    """Drone flies to a cleaner and picks it up in one action."""
    def __init__(self, cleaner_index: int, speed=5.0, fly_power=200.0, pickup_power=50.0, pickup_duration=10.0):
        super().__init__(f'pickup_cleaner_by_flying_{cleaner_index}')
        self.cleaner_index = cleaner_index
        self.speed = speed
        self.fly_power = fly_power
        self.pickup_power = pickup_power
        self.pickup_duration = pickup_duration

    def is_allowed(self, map_state: Map) -> bool:
        if self.cleaner_index >= len(map_state.cleaners):
            return False
        cleaner = map_state.cleaners[self.cleaner_index]
        drone = map_state.drone

        # Drone must not be carrying anything
        if drone.load is not None:
            return False

        # Cleaner must not be cleaning
        if cleaner.is_cleaning:
            return False
                
        return True

    def run(self, map_state: Map) -> Map:
        new_map = copy.deepcopy(map_state)
        cleaner = new_map.cleaners[self.cleaner_index]
        drone = new_map.drone

        # 1. Fly to cleaner if not already there
        if not np.array_equal(drone.pos3d, cleaner.pos3d):
            distance = np.linalg.norm(drone.pos3d - cleaner.pos3d)
            duration = distance / self.speed
            energy_cost = duration * self.fly_power
            drone.pos3d = cleaner.pos3d.copy()
            drone.is_moving = False
            # Battery
            energy_used_percent = energy_cost / drone.battery_capacity * 100.0
            drone.battery_level -= energy_used_percent
            if drone.battery_level < 0:
                drone.battery_level = 0.0
            new_map.time += duration

        # 2. Pick up cleaner
        # Only pick up if at cleaner's position and not carrying anything
        if np.array_equal(drone.pos3d, cleaner.pos3d) and drone.load is None:
            energy_cost = self.pickup_duration * self.pickup_power
            cleaner.is_cleaning = False
            drone.load = cleaner
            cleaner.pos3d = drone.pos3d.copy()
            cleaner.on_window = None
            # Battery
            energy_used_percent = energy_cost / drone.battery_capacity * 100.0
            drone.battery_level -= energy_used_percent
            if drone.battery_level < 0:
                drone.battery_level = 0.0
            new_map.time += self.pickup_duration

        return new_map 
    
    
# Composite action: Fly to base and charge drone

class FlyToBaseAndCharge(MapAction):
    """Drone flies to the base station, drops off load if carrying, and charges to full in one action."""
    def __init__(self, speed=5.0, fly_power=200.0, charge_rate=20.0, drop_power=50.0, drop_duration=2.0):
        super().__init__('fly_to_base_and_charge')
        self.speed = speed
        self.fly_power = fly_power
        self.charge_rate = charge_rate
        self.drop_power = drop_power
        self.drop_duration = drop_duration

    def is_allowed(self, map_state: Map) -> bool:
        drone = map_state.drone
        # Battery must not be full
        if drone.battery_level >= 99.9:
            return False
        return True

    def run(self, map_state: Map) -> Map:
        new_map = copy.deepcopy(map_state)
        drone = new_map.drone
        base = new_map.base_station

        # 1. Fly to base if not already there
        if not np.array_equal(drone.pos3d, base.pos3d):
            distance = np.linalg.norm(drone.pos3d - base.pos3d)
            duration = distance / self.speed
            energy_cost = duration * self.fly_power
            drone.pos3d = base.pos3d.copy()
            drone.is_moving = False
            # Move load with drone if carrying
            if drone.load is not None:
                drone.load.pos3d = drone.pos3d.copy()
            # Battery
            energy_used_percent = energy_cost / drone.battery_capacity * 100.0
            drone.battery_level -= energy_used_percent
            if drone.battery_level < 0:
                drone.battery_level = 0.0
            new_map.time += duration

        # 2. Drop off cleaner at base if carrying
        if drone.load is not None:
            cleaner = drone.load
            # Drop off at base
            cleaner.pos3d = base.pos3d.copy()
            drone.load = None

            # Battery
            energy_cost = self.drop_duration * self.drop_power
            energy_used_percent = energy_cost / drone.battery_capacity * 100.0
            drone.battery_level -= energy_used_percent
            if drone.battery_level < 0:
                drone.battery_level = 0.0
            new_map.time += self.drop_duration

        # 3. Charge drone (if not already full)
        if drone.battery_level < 99.9:
            current_energy = drone.battery_level / 100.0 * drone.battery_capacity
            energy_needed = drone.battery_capacity - current_energy
            duration = energy_needed / self.charge_rate
            if duration < 2.0:
                duration = 2.0
            drone.battery_level = 100.0
            new_map.time += duration

        return new_map
    
class DropOffCleanerAtBaseByFlying(MapAction):
    """Drone flies to base, drops off cleaner if carrying, in one action."""
    def __init__(self, speed=5.0, fly_power=200.0, drop_power=50.0, drop_duration=2.0):
        super().__init__('drop_off_cleaner_at_base_by_flying')
        self.speed = speed
        self.fly_power = fly_power
        self.drop_power = drop_power
        self.drop_duration = drop_duration

    def is_allowed(self, map_state: Map) -> bool:
        drone = map_state.drone

        # Must be carrying a cleaner
        if drone.load is None:
            return False
        return True

    def run(self, map_state: Map) -> Map:
        new_map = copy.deepcopy(map_state)
        drone = new_map.drone
        base = new_map.base_station

        # 1. Fly to base if not already there
        if not np.array_equal(drone.pos3d, base.pos3d):
            distance = np.linalg.norm(drone.pos3d - base.pos3d)
            duration = distance / self.speed
            energy_cost = duration * self.fly_power
            drone.pos3d = base.pos3d.copy()
            drone.is_moving = False
            # Move load with drone
            if drone.load is not None:
                drone.load.pos3d = drone.pos3d.copy()
            # Battery
            energy_used_percent = energy_cost / drone.battery_capacity * 100.0
            drone.battery_level -= energy_used_percent
            if drone.battery_level < 0:
                drone.battery_level = 0.0
            new_map.time += duration

        # 2. Drop off cleaner at base
        if np.array_equal(drone.pos3d, base.pos3d) and drone.load is not None:
            energy_cost = self.drop_duration * self.drop_power
            cleaner = drone.load
            cleaner.pos3d = base.pos3d.copy()
            drone.load = None
            # Battery
            energy_used_percent = energy_cost / drone.battery_capacity * 100.0
            drone.battery_level -= energy_used_percent
            if drone.battery_level < 0:
                drone.battery_level = 0.0
            new_map.time += self.drop_duration

        return new_map