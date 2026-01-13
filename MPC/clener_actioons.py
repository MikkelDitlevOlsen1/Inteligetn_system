import copy
import numpy as np
from map_actions import MapAction, NullAction
from map import Map


class CleanWindowAction(MapAction):
    """Cleaner scrubs its current window to completion."""

    def __init__(self, cleaner_index: int, power_consumption: float = 100.0):
        super().__init__(f"clean_window_{cleaner_index}")
        self.cleaner_index = cleaner_index
        self.power_consumption = power_consumption
        self.larsted_update_time = 0.0
        self.end_time = 0.0 # to be set when started
    
    def when_started(self , update_Map : Map):
        if self.cleaner_index >= len(update_Map.cleaners):   
            return False
        update_Map.cleaners[self.cleaner_index].is_cleaning = True     
        #print(f"Cleaner {self.cleaner_index} started cleaning.")   
        self.end_time=update_Map.cleaners[self.cleaner_index].on_window.cleaning_time + update_Map.time
        self.is_cleaning = True

    def when_done(self , update_Map : Map):
        if self.cleaner_index >= len(update_Map.cleaners):   
            return False
        #print("here")
        #print(f"Cleaner {self.cleaner_index} finished cleaning.")
        update_Map.cleaners[self.cleaner_index].is_cleaning = False
        update_Map.cleaners[self.cleaner_index].on_window.state = 'clean'
        self.when_runed_time(update_Map , self.end_time)
        self.is_cleaning = False

    def when_runed_time(self , update_Map : Map , time):
        run_time = time - self.larsted_update_time
        self.larsted_update_time = time
        if self.cleaner_index >= len(update_Map.cleaners):   
            return False
        cleaner = update_Map.cleaners[self.cleaner_index]
        energy_used = self.power_consumption * run_time  
        battery_drain_pct = (energy_used / cleaner.battery_capacity) * 100.0
        cleaner.battery_level = max(0.0, cleaner.battery_level - battery_drain_pct)


    def is_allowed(self, map_state: Map) -> bool:
        if self.cleaner_index >= len(map_state.cleaners):
            return False
        cleaner = map_state.cleaners[self.cleaner_index]
        window = cleaner.on_window
        # Must be on a window and at the same position
        if window is None:
            return False
        
        if not np.array_equal(cleaner.pos3d, window.pos3d):
            return False

        # Require sufficient battery to start cleaning
        if cleaner.battery_level <= 40.0:
            return False
        
        if cleaner.is_charging:
            return False
        
        if cleaner.is_cleaning:
            return False
        
        # Only clean dirty windows
        return window.state == "dirty"

 

class ChargeCleanerAction(MapAction):
    """Cleaner charges to full at the base station."""

    def __init__(self, cleaner_index: int, charge_rate: float = 10.0, min_duration: float = 2.0):
        super().__init__(f"charge_cleaner_{cleaner_index}")
        self.cleaner_index = cleaner_index
        self.charge_rate = charge_rate
        self.min_duration = min_duration
        self.larsted_update_time = 0.0
        self.end_time = 0.0  # to be set when started
    
    def when_started(self, update_Map: Map):
        if self.cleaner_index >= len(update_Map.cleaners):
            return False
        cleaner = update_Map.cleaners[self.cleaner_index]
        current_energy = cleaner.battery_level / 100.0 * cleaner.battery_capacity
        energy_needed = cleaner.battery_capacity - current_energy
        duration = energy_needed / self.charge_rate if self.charge_rate > 0 else 0.0
        if duration < self.min_duration:
            duration = self.min_duration
        update_Map.cleaners[self.cleaner_index].is_charging = True
        self.end_time = duration + update_Map.time

    def when_done(self, update_Map: Map):
        if self.cleaner_index >= len(update_Map.cleaners):
            return False
        update_Map.cleaners[self.cleaner_index].is_charging = False
        update_Map.cleaners[self.cleaner_index].battery_level = 100.0
        self.when_runed_time(update_Map, self.end_time)

    def when_runed_time(self, update_Map: Map, time):
        run_time = time - self.larsted_update_time
        self.larsted_update_time = time
        if self.cleaner_index >= len(update_Map.cleaners):
            return False
        cleaner = update_Map.cleaners[self.cleaner_index]
        # Charging adds energy (negative consumption)
        energy_added = self.charge_rate * run_time
        battery_gain_pct = (energy_added / cleaner.battery_capacity) * 100.0
        cleaner.battery_level = min(100.0, cleaner.battery_level + battery_gain_pct)

    def is_allowed(self, map_state: Map) -> bool:
        if self.cleaner_index >= len(map_state.cleaners):
            return False
        cleaner = map_state.cleaners[self.cleaner_index]
        base = map_state.base_station

        # Must be at base to charge
        if not np.array_equal(cleaner.pos3d, base.pos3d):
            return False
        
        if cleaner.is_charging:
            return False

        if cleaner.is_cleaning:
            return False
        
        # Skip if already full
        if cleaner.battery_level >= 99.9:
            return False

        # Cannot charge while cleaning
        return cleaner.is_cleaning is False


class CleanerNullAction(NullAction):
    """No-op cleaner action."""

    def __init__(self):
        super().__init__()
        self.larsted_update_time = 0.0
        self.end_time = 0.0
    
    def when_started(self, update_Map: Map):
        self.end_time = update_Map.time
        return True

    def when_done(self, update_Map: Map):
        return True
    
    def when_runed_time(self, update_Map: Map, time):
        self.larsted_update_time = time
        return True
