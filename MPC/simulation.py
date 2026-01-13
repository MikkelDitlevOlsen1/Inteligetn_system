from map import Map,random_map_generater
import matplotlib.pyplot as plt
from actions import *
import random
import time
class simulation:
    def __init__(self, map: Map , sleep_time=0.1 , real_time = False):

        self.real_time = real_time
        self.sleep_time = sleep_time
        self.map = map
        self.time = 0.0  # simulation time
        self.curent_actions_drone_action = null_action(self.map.drone)  # list of ongoing actions
        self.clener_actions = [null_action(cleaner) for cleaner in self.map.cleaners]
        self.all_clener_actions = []  # list of all possible actions
        self.all_drone_actions = []  # list of all possible actions
        self.alowed_drone_actions = [] # list of currently allowed actions
        self.alowed_clener_actions = [] # list of lists of currently allowed actions
        
        #paremeteres for simulation
        self.pickup_dropoff_duration = 2.0  # seconds
        self.drone_speed = 5.0  # units per second
        self.charging_rate_drone = 20.0  # joules per second
        self.charging_rate_cleaner = 10.0  # joules per second
        self.flying_power_consumption = 1.0  # joules per unit distance #when its flying
        self.cleaning_power_consumption = 0.5  # joules per second #when its cleaning
        self.constant_drone_power_consumption = 0.1  # joules per second #when its in the air
        self.carying_power_consumption = 0.2  # joules per second #when its carrying a cleaner
        self.constant_cleaner_power_consumption = 0.05  # joules per second #when its on the window
        self.dropof_pickup_comsumption = 1.0  # joules per action #when its picking up or dropping off a cleaner
        self.dt = 1.0  # time step for simulation in seconds

        self.creat_all_actions()
        self.update_alowed_actions()
    
    def update_alowed_actions(self):
        self.alowed_drone_actions = []
        self.alowed_clener_actions = []
        for action_list in self.all_clener_actions:
            allowed_actions_for_cleaner = []

            for action in action_list:
                if action.is_alowed():
                    allowed_actions_for_cleaner.append(action)
            self.alowed_clener_actions.append(allowed_actions_for_cleaner)

        for action in self.all_drone_actions:
            if action.is_alowed() == True:
                self.alowed_drone_actions.append(action)

    def creat_all_actions(self):
        self.all_drone_actions = [null_action(self.map.drone)]
        self.all_clener_actions = [[null_action(cleaner)] for cleaner in self.map.cleaners]
        # Create charge_drone action
        charge_drone_action = charge_drone(drone=self.map.drone, base_station=self.map.base_station, charge_rate=self.charging_rate_drone)
        self.all_drone_actions.append(charge_drone_action)

        fly_to_base_action = return_to_base(drone=self.map.drone, base_station=self.map.base_station, speed=self.drone_speed, power_consumption=self.flying_power_consumption)
        self.all_drone_actions.append(fly_to_base_action)

        #

        dropoff_clean_at_base_action = dropoff_clean_at_base(drone=self.map.drone, base_station=self.map.base_station, power_consumption=self.dropof_pickup_comsumption, drop_duration=self.pickup_dropoff_duration)
        self.all_drone_actions.append(dropoff_clean_at_base_action)     

        # Create charge_cleaner actions for each cleaner
        for i, cleaner in enumerate(self.map.cleaners):
            charge_cleaner_action = charge_cleaner(cleaner=cleaner, base_station=self.map.base_station, charge_rate=self.charging_rate_cleaner)
            self.all_clener_actions[i].append(charge_cleaner_action)

        # Create clean_window actions for each cleaner
        for i, cleaner in enumerate(self.map.cleaners):
            clean_window_action = clean_window(cleaner=cleaner, power_consumption=self.cleaning_power_consumption)
            self.all_clener_actions[i].append(clean_window_action)

        #creat fly_to_window actions for each window
        for window in self.map.windows:
            fly_to_window_action = fly_to_window(drone=self.map.drone, window=window, speed=self.drone_speed, power_consumption=self.flying_power_consumption)
            self.all_drone_actions.append(fly_to_window_action)
           
            dropoff_cleaner_action = dropoff_cleaner(drone=self.map.drone, window=window, power_consumption=self.dropof_pickup_comsumption, drop_duration=self.pickup_dropoff_duration)
            self.all_drone_actions.append(dropoff_cleaner_action)

        
        #creat pickup_cleaner and dropoff_cleaner actions for each cleaner
        for cleaner in self.map.cleaners:
            pickup_cleaner_action = pickup_cleaner(drone=self.map.drone, cleaner=cleaner ,power_consumption=self.dropof_pickup_comsumption , pickup_duration=self.pickup_dropoff_duration)
            self.all_drone_actions.append(pickup_cleaner_action)

            pickup_clean_at_base_action = pickup_clean_at_base(cleaner=cleaner, drone=self.map.drone, base_station=self.map.base_station, power_consumption=self.dropof_pickup_comsumption, pickup_duration=self.pickup_dropoff_duration)
            self.all_drone_actions.append(pickup_clean_at_base_action)



    def step(self, dt: float):
        self.time += dt
        self.map.time = self.time
        
        if self.curent_actions_drone_action.step_action(self.time, dt):
            self.curent_actions_drone_action = null_action(self.map.drone)

        for action in self.clener_actions:
            if action.step_action(self.time, dt):
                action = null_action(action.target)
            


    
    def update_drone_action(self, action: actions):
        if isinstance(action, null_action):
            return True
        elif action in self.alowed_drone_actions:
            self.curent_actions_drone_action = action
            action.set_start_time(self.time)
            return True
    
    def update_clener_action(self,  actions: list[actions]):

        for i, action in enumerate(actions):
            if isinstance(action, null_action):
                continue
            if action in self.alowed_clener_actions[i]:
                self.clener_actions[i] = action
                action.set_start_time(self.time)
                return True
    
    def run_in_time(self, run_time: float):
        while self.time < run_time:
            #self.visualize()
            #print(f"Simulation time: {self.time:.2f} seconds")
            print(f"active drone action: {self.curent_actions_drone_action}")
            #print("alowed drone actions: ", ", ".join([cls.__str__() for cls in self.alowed_drone_actions]))
            #print("-----------------------------")
            self.step(self.dt)
            self.creat_all_actions()
            self.update_alowed_actions()
        
            drone_action=self.chose_drone_action()
            cleaner_actions=self.chose_clener_action()
            self.update_drone_action(drone_action)
            self.update_clener_action(cleaner_actions)
            if self.real_time:
                time.sleep(self.dt)
            else:
                time.sleep(self.sleep_time)

            #input("Press Enter to continue to next step...")
            #print(len(self.map.cleaners))

    def chose_drone_action(self) -> actions:
        # 90% chance to choose the first allowed action, else random
        if isinstance(self.curent_actions_drone_action, null_action):
             return random.choice(self.alowed_drone_actions[1:])
        if self.alowed_drone_actions:
            if random.random() < 0.99:
                return self.alowed_drone_actions[0]
            else:
                return random.choice(self.alowed_drone_actions)
        return null_action(self.map.drone)
    
    def chose_clener_action(self) -> list[actions]:
        # 90% chance to choose the first allowed action for each cleaner, else random
        chosen_actions = []
        for i, allowed_actions in enumerate(self.alowed_clener_actions):
            if allowed_actions:
                if random.random() < 0.9:
                    chosen_actions.append(allowed_actions[0])
                else:
                    chosen_actions.append(random.choice(allowed_actions))
            else:
                chosen_actions.append(null_action(self.map.cleaners[i]))
        return chosen_actions
      
    def smart_cleaner_action(self) -> list[actions]:
        chosen_actions = []
        for i, allowed_actions in enumerate(self.alowed_clener_actions):
            for action in allowed_actions:
                if isinstance(action, clean_window):
                    chosen_actions.append(action) #prioritize cleaning
                    break
                elif isinstance(action, charge_cleaner):
                    chosen_actions.append(action) #second prioritize charging
                    break
            else:
                chosen_actions.append(allowed_actions[0])  # default to first action if no clean or charge found
        return chosen_actions

    def smart_alowed_action(self) -> list[actions]:
        self.alowed_drone_actions = []
        for action in self.all_drone_actions:
            if isinstance(action, fly_to_window):
                totel_comsumtioon=action.power_consumption + self.constant_drone_power_consumption 



    


    def visualize(self):
        self.map.visualize()


if __name__ == "__main__":
    test_map = random_map_generater(num_cleaners=2, num_windows=2)
    #test_map.cleaners[0].pos3d = test_map.windows[0].pos3d  # Move first cleaner to first window for testing
    sim = simulation(test_map,)
    sim.run_in_time(100.0)
    sim.visualize()