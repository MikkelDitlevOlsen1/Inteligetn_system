import time
import random
import numpy as np
from map import Map, random_map_generater
from map_actions import (
    DropOffCleanerAtBaseByFlying,
    MapAction,
    NullAction,
    FlyToWindow,
    PickupCleaner,
    DropoffCleaner,
    PickupCleanerAtBase,
    DropoffCleanerAtBase,
    ReturnToBase,
    ChargeDrone,
    DropCleanerOffAtWindow,
    PickupCleanerByFlying,
    FlyToBaseAndCharge,
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
        drone = map_state.drone
        drone_pos = drone.pos3d.tolist() if hasattr(drone, 'pos3d') else [0,0,0]
        drone_battery = getattr(drone, 'battery_level', 0.0)
        drone_has_load = 1 if getattr(drone, 'load', None) is not None else 0

        state = []
        state.extend(drone_pos)
        state.append(drone_battery)
        state.append(drone_has_load)

        for cleaner in map_state.cleaners:
            cpos = cleaner.pos3d.tolist() if hasattr(cleaner, 'pos3d') else [0,0,0]
            cbatt = getattr(cleaner, 'battery_level', 0.0)
            cclean = 1 if getattr(cleaner, 'is_cleaning', False) else 0
            state.extend(cpos)
            state.append(cbatt)
            state.append(cclean)

        for window in map_state.windows:
            wpos = window.pos3d.tolist() if hasattr(window, 'pos3d') else [0,0,0]
            wclean = 1 if getattr(window, 'state', None) == 'clean' else 0
            state.extend(wpos)
            state.append(wclean)

        return np.array(state, dtype=np.float32)

    def compute_reward(self, prev_state, new_state):
        """Reward: +1000 for each new window cleaned, -1000 if any cleaner battery is 0, -1 per step."""
        prev_windows = prev_state[2]
        new_windows = new_state[2]
        cleaned = sum(n > p for p, n in zip(prev_windows, new_windows))
        reward = 1000 * cleaned
        # Penalty for any cleaner battery depleted
        if any(b == 0.0 for b in new_state[1]) and not any(b == 0.0 for b in prev_state[1]):
            reward -= 1000
        # Small step penalty
        reward -= 1
        return reward

    def rl_step(self, action_idx, map_state: Map = None):
        """Take an action by index, return new state, reward, done."""
        if map_state is None:
            map_state = self.map
        # Build and filter allowed actions
        drone_candidates = self._build_drone_actions(map_state)
        allowed_drone = self._allowed(drone_candidates, map_state)
        allowed_drone = self.advance_allowed(allowed_drone, map_state)
        # Clamp action_idx
        if not allowed_drone:
            chosen_action = NullAction()
        else:
            action_idx = max(0, min(action_idx, len(allowed_drone) - 1))
            chosen_action = allowed_drone[action_idx]
        prev_state = self.get_state(map_state)
        new_map = self.apply_action(chosen_action, map_state)
        new_state = self.get_state(new_map)
        reward = self.compute_reward(prev_state, new_state)
        # Done if all windows clean or any cleaner battery is 0
        done = all(w == 1 for w in new_state[2]) or any(b == 0.0 for b in new_state[1])
        return new_state, reward, done, new_map
    """Simulation that applies full-length map actions to completion."""

    def __init__(self, map_state: Map, real_time: bool = False, sleep_time: float = 0.05):
        self.map = map_state
        self.real_time = real_time
        self.sleep_time = sleep_time

        # Tunable parameters
        self.drone_speed = 5.0
        self.flying_power_consumption = 0.5
        self.pickup_drop_power = 1.0
        self.pickup_drop_duration = 2.0
        self.charging_rate_drone = 20.0
        self.charging_rate_cleaner = 10.0

        self.cleaning_power_consumption = 0.2  # joules per second #when its cleaning

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
    # ---- Action builders ----
    def _build_drone_actions(self, map_state: Map = None):
        if map_state is None:
            map_state = self.map
        actions = [
            NullAction(),
            ReturnToBase(speed=self.drone_speed, power_consumption=self.flying_power_consumption),
            ChargeDrone(charge_rate=self.charging_rate_drone),
            DropoffCleanerAtBase(power_consumption=self.pickup_drop_power, drop_duration=2.0),
        ]

        # Per-window actions
        for w_idx, _ in enumerate(map_state.windows):
            actions.append(FlyToWindow(window_index=w_idx, speed=self.drone_speed, power_consumption=self.flying_power_consumption))
            actions.append(DropoffCleaner(window_index=w_idx, power_consumption=self.pickup_drop_power, drop_duration=self.pickup_drop_duration))

        # Per-cleaner actions
        for c_idx, _ in enumerate(map_state.cleaners):
            actions.append(PickupCleaner(cleaner_index=c_idx, power_consumption=self.pickup_drop_power, pickup_duration=self.pickup_drop_duration))
            actions.append(PickupCleanerAtBase(cleaner_index=c_idx, power_consumption=self.pickup_drop_power, pickup_duration=2.0))

        return actions


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
        for w_idx, _ in enumerate(map_state.windows):
            actions.append(DropCleanerOffAtWindow(window_index=w_idx, drop_power=self.pickup_drop_power, drop_duration=self.pickup_drop_duration, fly_power=self.flying_power_consumption, speed=self.drone_speed))
        # Per-cleaner actions
        for c_idx, _ in enumerate(map_state.cleaners):
            actions.append(PickupCleanerByFlying(cleaner_index=c_idx, pickup_power=self.pickup_drop_power, pickup_duration=self.pickup_drop_duration, fly_power=self.flying_power_consumption, speed=self.drone_speed))
            #actions.append(PickupCleanerAtBase(cleaner_index=c_idx, power_consumption=self.pickup_drop_power, pickup_duration=2.0))

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
    
    def advance_allowed(self,  actions, map_state: Map = None):
        alowed_actions = []
        for alowed_action in actions:
            if isinstance(alowed_action, ChargeDrone) or isinstance(alowed_action,DropoffCleanerAtBase) or isinstance(alowed_action,ReturnToBase):
                alowed_actions.append(alowed_action)
            elif not isinstance(alowed_action, ReturnToBase):
                new_state = self.apply_action(alowed_action, map_state, drone_only=True)
                drone_candidates = self._build_drone_actions(new_state)
                #drone_candidates=self._allowed(drone_candidates , new_state)
                for action in drone_candidates:
                    if isinstance(action, ReturnToBase):
                        new_state = self.apply_action(action, new_state, drone_only=True)
                        if new_state.drone.battery_level > 10.0:
                            alowed_actions.append(alowed_action)
        return alowed_actions


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

                if isinstance(action, PickupCleanerByFlying):
                    if action.is_allowed(new_state):
                        new_state2 = self.apply_action(action, new_state, drone_only=False)
                        dont=False
                        for cleaner in new_state2.cleaners:
                            if cleaner.battery_level < 10.0:
                                dont=True         
                        if dont:   
                            allow_action=False
            if allow_action:
                alowed_actions.append(alowed_action)
                    

        return alowed_actions

    def _choose_drone_action(self, allowed_actions):
        if not allowed_actions:
            return NullAction()
        # Prefer first non-null; otherwise random
        non_null = [a for a in allowed_actions if not isinstance(a, NullAction)]

        return random.choice(non_null)

    def advance_choose_drone_action(self, allowed_actions):
        # Priority: dropoff at base -> charge -> return to base -> dropoff -> pickup -> flyto -> null
        non_null = [a for a in allowed_actions if not isinstance(a, NullAction)]

        for action in allowed_actions:
            if isinstance(action, PickupCleanerAtBase):
                return action
            if isinstance(action, DropoffCleaner):
                return action
            if isinstance(action, PickupCleaner):
                return action
        return random.choice(non_null)

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
        print(f"Allowed drone actions: {[str(a) for a in allowed_drone]}")

        #allowed_drone=self.advance_allowed(allowed_drone , map_state)
        allowed_drone=self.advance_allowedv2(allowed_drone , map_state)
        print(f"Allowed drone actions: {[str(a) for a in allowed_drone]}")

        
        #chosen_drone = self._choose_drone_action(allowed_drone)
        chosen_drone = self.advance_choose_drone_action(allowed_drone)
        for cleaner in map_state.cleaners:
            print(f"Cleaner battery level: {cleaner.battery_level}")
        print(f"Drone action: {chosen_drone}")
        map_state = self.apply_action(chosen_drone, map_state)
        
        #print(f"drone states: {map_state.drone.pos3d}, battery: {map_state.drone.battery_level}")
        if self.real_time:
            time.sleep(self.sleep_time)

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

            
    def visualize(self):
        self.map.visualize()

    def apply_action(self, action :MapAction, map_state: Map , drone_only=False) -> Map  :
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
                    #print(f"Cleaner {c_idx} action: {chosen_cleaner}")
            next_map.update_cleaning_processes()

        next_map.update_states()

        return next_map

    def apply_list_of_actions(self, actions :list[MapAction], map_state: Map) -> Map  :
        next_map = map_state
        for action in actions:
            next_map = self.apply_action( action , next_map)
        return next_map


if __name__ == "__main__":
    random.seed(42) # acttion seed
    np.random.seed(42) # map generater seed
    test_map = random_map_generater(num_cleaners=2, num_windows=5)
    sim = MapSimulation(test_map, real_time=False, sleep_time=0.0)
    sim.run(steps=300)
    sim.visualize()
