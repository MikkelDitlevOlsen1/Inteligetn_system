import time
import random
from turtle import done
from typing import List
import numpy as np
from map import Map, random_map_generater, map_1
from map_actions import (
    DropOffCleanerAtBaseByFlying,
    MapAction,
    NullAction,
    DropCleanerOffAtWindow,
    PickupCleanerByFlying,
    FlyToBaseAndCharge,
    FlyToClener,
)
from clener_actioons import (
    CleanWindowAction,
    ChargeCleanerAction,
    CleanerNullAction,
)


class MapSimulation:
    def get_dqn_state(self, map_state: Map = None):
        """
        Returns a flat numpy array state for DQN:
        [drone_x, drone_y, drone_z, drone_battery, drone_has_load (1/0),
         cleaner1_x, cleaner1_y, cleaner1_z, cleaner1_battery, cleaner1_is_cleaning (1/0), ...
         window1_x, window1_y, window1_z, window1_is_clean (1/0), ...]
        """
        if map_state is None:
            map_state = self.map

        state = map_state.states.copy()
        # print(state)
        return np.array(state, dtype=np.float32)

    def compute_reward(self, prev_map : Map, new_map :Map):
        # Assuming last N values are window clean states (0 or 1)
        cleaned = 0
        reward=0
        for prev, new in zip(prev_map.windows, new_map.windows):
            cleaned += int(prev.state == 'dirty' and new.state == 'clean')
        reward += 100 * cleaned
        
        # Penalty for any cleaner battery depleted
        if any(cleaner.battery_level <= 0.0 for cleaner in new_map.cleaners):
                reward -= 100000
        
        if any(cleaner.battery_level <= 20.0 for cleaner in new_map.cleaners):
                reward -= 10

        if new_map.drone.battery_level <= 5.0:
                reward -= 2000
        
        for prev, new in zip(prev_map.cleaners, new_map.cleaners):
            if prev.on_window is None and new.on_window is not None:
                reward += 100  # bonus for placing cleaner on window

            if prev.on_window is not None and new.on_window is None:
                reward += 100  # bonus for removing cleaner on window
            
            #if new.on_window is not None and new.is_cleaning is False:
                #reward -= 2  # bonus for keeping cleaner on window

            
            #if prev.battery_level == new.battery_level and new.on_window is None:
                #reward -= 1  # penalty for idle cleaner not on window 
            
            pass
        
        reward += (prev_map.time - new_map.time) * 0.1  # time penalty, adjust index if needed
        return reward

    def rl_step(self, action_idx, map_state: Map = None):
        """Take an action by index, return new state, reward, done."""
        if map_state is None:
            map_state = self.map


        # Build and filter allowed actions
        drone_candidates = self.new_build_drone_actions(map_state)
        allowed_drone = self._allowed(drone_candidates, map_state)
        allowed_drone = self.advance_allowedv2(allowed_drone, map_state)
        # Clamp action_idx
        if not allowed_drone:
            chosen_action = NullAction()
        else:
            action_idx = max(0, min(action_idx, len(allowed_drone) - 1))
            chosen_action = allowed_drone[action_idx]
        #print(f"Allowed drone actions: {[str(a) for a in allowed_drone]}")
        #print(f"Chosen action: {chosen_action}")
        new_map = self.apply_action(chosen_action, map_state)
        new_state = self.get_dqn_state(new_map)

        reward = self.compute_reward(map_state, new_map)
        # Done if all windows clean or any cleaner battery is 0
        done = new_map.is_done()
        self.map = new_map  # Update internal map state
        if done:
            print(f"Episode done. {self.map.time} , cleaned windows: {sum(1 for w in self.map.windows if w.state=='clean')} / {len(self.map.windows)}")

        return new_state, reward, done, new_map
    """Simulation that applies full-length map actions to completion."""



    def __init__(self, map_state: Map, real_time: bool = False, sleep_time: float = 0.0):
        self.map = map_state
        self.real_time = real_time
        self.sleep_time = sleep_time
        self.action_log = []
        # Tunable parameters
        self.drone_speed = 10.0
        self.flying_power_consumption = 1.5
        self.pickup_drop_power = 1.0
        self.pickup_drop_duration = 2.0
        self.charging_rate_drone = 5.0
        self.charging_rate_cleaner = 2.0

        self.cleaning_power_consumption = 1.3  # joules per second #when its cleaning

            #paremeteres for simulation<
        """
        self.pickup_dropoff_duration = 2.0  # seconds
        self.drone_speed = 5.0  # units per second
        self.charging_rate_drone = 20.0  # joules per second
        self.charging_rate_cleaner = 10.0  # joules per second
        self.flying_power_consumption = 1.0  # joules per unit distance #when its flying
        self.constant_drone_power_consumption = 0.1  # joules per second #when its in the air
        self.carying_power_consumption = 0.2  # joules per second #when its carrying a cleaner
        self.constant_cleaner_power_consumption = 0.05  # joules per second #when its on the window
        self.dropof_pickup_comsumption = 1.0  # joules per action #when its picking up or dropping off a cleaner
        """


    def new_build_drone_actions(self, map_state: Map = None):
        if map_state is None:
            map_state = self.map
        actions = [
            NullAction(),
            #ReturnToBase(speed=self.drone_speed, power_consumption=self.flying_power_consumption),
            FlyToBaseAndCharge(charge_rate=self.charging_rate_drone, speed=self.drone_speed, fly_power=self.flying_power_consumption ,drop_power=self.pickup_drop_power,drop_duration=self.pickup_drop_duration),
            DropOffCleanerAtBaseByFlying(speed=self.drone_speed, fly_power=self.flying_power_consumption, drop_power=self.pickup_drop_power, drop_duration=self.pickup_drop_duration),
        ]

        # Per-window actions
        #for w_idx, _ in enumerate(map_state.windows):
            #actions.append(DropCleanerOffAtWindow(window_index=w_idx, drop_power=self.pickup_drop_power, drop_duration=self.pickup_drop_duration, fly_power=self.flying_power_consumption, speed=self.drone_speed, cleaning_power=self.cleaning_power_consumption))
        
        #creat the drop cleaner of at window but for the 5 closesest to base that is not cleaned
        base_pos = map_state.base_station.pos3d
        closest_windows = sorted(
            [w for w in map_state.windows if w.state != "clean"],
            key=lambda w: np.linalg.norm(base_pos - w.pos3d)
        )[:3]
        for window in closest_windows:
            w_idx = map_state.windows.index(window)
            actions.append(DropCleanerOffAtWindow(window_index=w_idx, drop_power=self.pickup_drop_power, drop_duration=self.pickup_drop_duration, fly_power=self.flying_power_consumption, speed=self.drone_speed, cleaning_power=self.cleaning_power_consumption))

        # Per-cleaner actions
        for c_idx, _ in enumerate(map_state.cleaners):
            actions.append(PickupCleanerByFlying(cleaner_index=c_idx, pickup_power=self.pickup_drop_power, pickup_duration=self.pickup_drop_duration, fly_power=self.flying_power_consumption, speed=self.drone_speed))
            #actions.append(PickupCleanerAtBase(cleaner_index=c_idx, power_consumption=self.pickup_drop_power, pickup_duration=2.0))
            actions.append(FlyToClener(cleaner_index=c_idx, speed=self.drone_speed, fly_power=self.flying_power_consumption))
        return actions

    def _build_cleaner_actions(self, map_state: Map = None):
        if map_state is None:
            map_state = self.map
        cleaner_actions = []
        for c_idx, _ in enumerate(map_state.cleaners):
            cleaner_actions.append([
                CleanWindowAction(cleaner_index=c_idx, power_consumption= self.cleaning_power_consumption ),
                ChargeCleanerAction(cleaner_index=c_idx, charge_rate=self.charging_rate_cleaner, min_duration=2.0),
            ])
        return cleaner_actions

    # ---- Selection helpers ----
    def _allowed(self, actions, map_state: Map = None):
        if map_state is None:
            map_state = self.map
        return [a for a in actions if a.is_allowed(map_state)]
    

    def advance_allowedv2(self,  actions, map_state: Map = None):
        alowed_actions = []
        for alowed_action in actions:
            #if isinstance(alowed_action, FlyToBaseAndCharge):
             #   alowed_actions.append(alowed_action)
            #else:
            new_state = self.apply_action(alowed_action, map_state, drone_only=False)
            #print(f"New state drone battery level: {new_state.drone.battery_level}")
            drone_candidates = self.new_build_drone_actions(new_state)
            #drone_candidates=self._allowed(drone_candidates , new_state)
            allow_action=True

            for action in drone_candidates:
                if isinstance(action, FlyToBaseAndCharge):
                    new_state2 = self.apply_action(action, new_state, drone_only=True)
                    if new_state2.drone.battery_level < 10.0:
                        allow_action=False
                        break
                    
                if isinstance(action, PickupCleanerByFlying):
                    if action.is_allowed(new_state):
                        new_state2 = self.apply_action(action, new_state, drone_only=False)
                        dont=False
                        for cleaner in new_state2.cleaners:
                            if cleaner.battery_level < 10.0:
                                dont=True         
                        if dont:   
                            allow_action=False
                            break
            if allow_action:
                alowed_actions.append(alowed_action)
                    

        return alowed_actions

    def advance_allowedv3(self,  actions, map_state: Map = None):
        alowed_actions = []
        for alowed_action in actions:
            #if isinstance(alowed_action, FlyToBaseAndCharge):
             #   alowed_actions.append(alowed_action)
            #else:
            new_state = self.apply_action(alowed_action, map_state, drone_only=False)
            #print(f"New state drone battery level: {new_state.drone.battery_level}")
            drone_candidates = self.new_build_drone_actions(new_state)
            allowed_actions = self._allowed(drone_candidates , new_state)
            #drone_candidates=self._allowed(drone_candidates , new_state)
            allow_action=True

            for action in allowed_actions:
                if isinstance(action, FlyToBaseAndCharge):
                    new_state2 = self.apply_action(action, new_state, drone_only=True)
                    if new_state2.drone.battery_level < 10.0:
                        allow_action=False
                        break
                       
            if allow_action:
                alowed_actions.append(alowed_action)
                    

        return alowed_actions

    def _choose_drone_action(self, allowed_actions):
        if not allowed_actions:
            return NullAction()
        # Prefer first non-null; otherwise random
        #non_null = [a for a in allowed_actions if not isinstance(a, NullAction)]

        return random.choice(allowed_actions)

    def choose_drone_action_depth(self, allowed_actions, map_state: Map = None, depth=1, first_call=True, call_number=None):
        if map_state is None:
            map_state = self.map
        if first_call:
            call_number=0

        if depth == 0:
            return self._choose_drone_action(allowed_actions), 0.0
        
        if not allowed_actions:
            print("No allowed actions in depth choice")
            return NullAction(), 0.0

        best_action = None
        best_reward = float('-inf')

        for action in allowed_actions:
            new_state = self.apply_action(action, map_state)
            drone_candidates = self.new_build_drone_actions(new_state)
            allowed_next_actions = self._allowed(drone_candidates, new_state)
            #allowed_next_actions = self.advance_allowedv2(allowed_next_actions, new_state)
            next_action, reward = self.choose_drone_action_depth(allowed_next_actions, new_state, depth - 1, first_call=False, call_number=call_number+1)
            this_action_reward = self.compute_reward(map_state, new_state)
            total_reward = this_action_reward + reward
            if first_call:
                #print(f"    Action: {action}, this action reward {this_action_reward} Total Reward: {total_reward}")
                pass
            else:
                if call_number is 1:
                    #print(f"    Sub-action: {action}, this action reward {this_action_reward} Total Reward: {total_reward}  cleaner battery before {[map_state.cleaners[i].battery_level for i in range(len(new_state.cleaners))]} cleaner_battery after: {[new_state.cleaners[i].battery_level for i in range(len(new_state.cleaners))]}")
                    pass
                elif call_number is 2:
                    #print(f"        sub_Sub-action: {action}, this action reward {this_action_reward} Total Reward: {total_reward}")
                    pass

            if total_reward > best_reward:
                best_reward = total_reward
                best_action = action
            elif total_reward == best_reward:
                if isinstance(best_action, NullAction):
                    best_action = action

        return best_action, best_reward


    def _choose_cleaner_action(self, allowed_actions_for_cleaner):
        # Priority: clean -> charge -> null
        for action in allowed_actions_for_cleaner:
            if isinstance(action, CleanWindowAction):
                return action
        for action in allowed_actions_for_cleaner:
            if isinstance(action, ChargeCleanerAction):
                return action
        return None
    # ---- Simulation loop ----

    def step(self, map_state: Map = None) -> Map:
        if map_state is None:
            map_state = self.map
        
        # Build action candidates fresh based on current map
        #drone_candidates = self._build_drone_actions(map_state)
        drone_candidates=self.new_build_drone_actions(map_state)
        allowed_drone = self._allowed(drone_candidates, map_state)
        #print(f"Allowed drone actions pre: {[str(a) for a in allowed_drone]}")

        #allowed_drone=self.advance_allowed(allowed_drone , map_state)
        #allowed_drone=self.advance_allowedv2(allowed_drone , map_state)

        #self.visualize()
        choose_drone_action_depth = True

        if choose_drone_action_depth:    
            chosen_drone, expected_reward = self.choose_drone_action_depth(allowed_drone, map_state, depth=4)
            #print(f"Chosen drone action by depth: {chosen_drone} with expected reward: {expected_reward}")
        else:
            allowed_drone=self.advance_allowedv2(allowed_drone , map_state)
            chosen_drone = self._choose_drone_action(allowed_drone)
        #chosen_drone = self.advance_choose_drone_action(allowed_drone)
        #for cleaner in map_state.cleaners:
                #print(f"Allowed drone  actions: {[str(a) for a in allowed_drone]}")
        print(f"Drone action: {chosen_drone}")
        map_state = self.apply_action(chosen_drone, map_state,print_this=False)
        log_info = {"map_state": map_state.states.copy(), "chosen_drone_action": str(chosen_drone)}
        self.action_log.append(log_info)

        #print(f"drone states: {map_state.drone.pos3d}, battery: {map_state.drone.battery_level}")
        if self.real_time:
            #time.sleep(self.sleep_time)
            pass
        for cleaner in map_state.cleaners:
            #print(f"Cleaner battery: {cleaner.battery_level}")
            if cleaner.on_window is not None:
                #print(f"Cleaner on window: {cleaner.on_window} cleaner battery: {cleaner.battery_level}")
                pass
        return map_state

    def run(self, steps: int = 20):
        done=False
        map_state = self.map
        for step in range(steps):
            map_state = self.step(map_state)
            self.map=map_state
            # Break if all windows are clean
            for window in map_state.windows:
                if not window.state == 'clean':
                    break
            else:
                print(f"All windows are clean. Ending simulation after {step} steps and {self.map.time}.")
                break
                
            for cleaner in map_state.cleaners:
                if cleaner.battery_level == 0.0:
                    done=True
                    break
            if done:
                print(f"A cleaner has run out of battery. Ending simulation after {step} steps and {self.map.time}.")
                break
        
        if step == steps -1:
            print(f"Reached maximum steps ({steps}). Ending simulation after {self.map.time}.")
            clened=sum(1 for window in map_state.windows if window.state=='clean')
            print(f"Total cleaned windows: {clened} / {len(map_state.windows)}")
            
    def visualize(self):
        self.map.visualize()

    def apply_action(self, action :MapAction, map_state: Map , drone_only=False , print_this=False) -> Map  :
        next_map = action.run(map_state)

        #print(f"drone states: {self.map.drone.pos3d}, battery: {self.map.drone.battery_level}")
        #print(self.map.cleaning_processes)
        #print(f"cleaner pos3d: {[cleaner.pos3d for cleaner in self.map.cleaners]}")
        #print(f"cleaners on window: {[cleaner.on_window for cleaner in self.map.cleaners]}")
        #print(f"cleaner states: {[cleaner.states for cleaner in self.map.cleaners]}")

        if not(drone_only):
            cleaner_candidates = self._build_cleaner_actions(next_map)
            for c_idx, cleaner_actions in enumerate(cleaner_candidates):
                allowed_cleaner = self._allowed(cleaner_actions, next_map)
                #print(f"Allowed cleaner {c_idx} actions: {[str(a) for a in allowed_cleaner]}")
                chosen_cleaner = self._choose_cleaner_action(allowed_cleaner)
                if chosen_cleaner is not None:
                    next_map.cleaning_processes[c_idx] = chosen_cleaner
                    chosen_cleaner.when_started(next_map)
                    if print_this:
                        if isinstance(chosen_cleaner, CleanWindowAction):
                            print(f"Cleaner {c_idx} cleaning window: {next_map.cleaners[c_idx].on_window}")
                    
            next_map.update_cleaning_processes(print_this)

        next_map.update_states()
        if print_this:
            print(next_map.cleaning_processes)
        return next_map

    def apply_list_of_actions(self, actions :list[MapAction], map_state: Map) -> Map  :
        next_map = map_state
        for action in actions:
            next_map = self.apply_action( action , next_map)
        return next_map

    def run_state_to_state(self, start_states , end_states , dt: float, action = None) -> List[List[float]]:
        """
        Interpolate between two Map states over time with dt steps.
        Boolean values (0 or 1) are held constant and only change at the last step.
        Continuous values are linearly interpolated.
        
        Args:
            start_states: Starting state array
            end_states: Ending state array
            dt: Time step for interpolation
            
        Returns:
            List of state arrays (map.states format) showing progression from start to end
        """
       
        start_array = np.array(start_states, dtype=np.float32)
        end_array = np.array(end_states, dtype=np.float32)
        
        # Calculate time difference between start and end
        time_diff = end_array[0] - start_array[0]
        
        if time_diff <= 0:
            return [start_array.tolist()]
        
        # Calculate number of steps needed
        num_steps = int(time_diff / dt) + 1
    
        new_action_list = []
        # Create interpolated states
        interpolated_states = []
        for step in range(num_steps):
            alpha = step / (num_steps - 1)
            interp_state = start_array * (1 - alpha) + end_array * alpha
            
            
            # Handle boolean values (assuming they are at fixed indices)
            for i in range(len(start_array)):
                if start_array[i] in [0, 1] and end_array[i] in [0, 1]:
                    interp_state[i] = start_array[i] if alpha < 1.0 else end_array[i]
            
            interpolated_states.append(interp_state.tolist())
            
            if action is not None:
                new_action_list.append(action)

        return interpolated_states, new_action_list
        
if __name__ == "__main__":
    random.seed(42) # acttion seed
    np.random.seed(42) # map generater seed
    test_map = random_map_generater(num_cleaners=2, num_windows=10)
    test_map = map_1(num_cleaners=2, num_windows=10 )
    sim = MapSimulation(test_map, real_time=False, sleep_time=0.0)
    sim.run(steps=400)
    #sim.visualize()
    #print(sim.get_dqn_state())'

    all_states = []
    all_actions_list = []
    dt=0.2
    speed=2
    for log in range(1, len(sim.action_log)):
        new_states_list,new_actuion_list = sim.run_state_to_state(sim.action_log[log-1]["map_state"], sim.action_log[log]["map_state"], dt=dt ,action=sim.action_log[log]["chosen_drone_action"])
        all_states.extend(new_states_list)    
        all_actions_list.extend(new_actuion_list)
    print(f"Total states for visualization: {len(all_states)}")
    print((all_states[-1][0]))
    #sim.map.visualize_from_map_states(all_states[-1]) 
    input("Press Enter to start animation...")
    sim.map.animate_map_states_list(all_states, all_actions_list, dt=dt/speed ,block=False )