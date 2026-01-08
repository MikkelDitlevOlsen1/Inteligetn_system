import time
import math
from typing import Optional, List

import numpy as np

from map import Map, random_map_generater
from simulation import simulation
from live_visualizer import create_live_visualization
from actions import (
    actions,
    null_action,
    fly_to_window,
    dropoff_cleaner,
    pickup_cleaner,
    pickup_clean_at_base,
    dropoff_clean_at_base,
    return_to_base,
    charge_drone,
    clean_window,
    charge_cleaner,
)


class MPCController:
    """A simple MPC-style controller for choosing actions.

    Objective: Clean all windows as fast as possible while keeping the drone
    battery above a safety threshold.

    This is a pragmatic rolling-horizon heuristic that evaluates allowed actions
    at each step and chooses the next best move. It prefers transporting idle
    cleaners to nearest dirty windows, starting cleaning when possible, and
    charging when battery is low.
    """

    def __init__(
        self,
        sim: simulation,
        safety_battery_pct: float = 10.0,
        horizon_steps: int = 3,
    ) -> None:
        self.sim = sim
        self.map: Map = sim.map
        self.safety_battery_pct = safety_battery_pct
        self.horizon_steps = horizon_steps

        # Ensure action pools exist
        if not getattr(self.sim, "all_drone_actions", None):
            self.sim.creat_all_actions()

    # ------------------------------- Helpers ---------------------------------
    def _dirty_windows(self) -> List:
        return [w for w in self.map.windows if getattr(w, "state", "dirty") == "dirty"]

    def _nearest_dirty_window(self, pos3d: np.ndarray) -> Optional:
        dirty = self._dirty_windows()
        if not dirty:
            return None
        best_w = None
        best_d = float("inf")
        for w in dirty:
            d = float(np.linalg.norm(pos3d - w.pos3d))
            if d < best_d:
                best_d = d
                best_w = w
        return best_w

    def _battery_after_action_pct(self, target, action: actions) -> float:
        # Approximate battery after completing action based on energy_cost
        cap = getattr(target, "battery_capacity", 100.0)
        lvl = getattr(target, "battery_level", cap)
        # actions.aplay_action_power subtracts energy_cost / capacity * 100.0
        delta_pct = (action.energy_cost / cap) * 100.0 if cap > 0 else action.energy_cost
        return lvl - delta_pct

    def _is_action_safe_for_battery(self, target, action: actions) -> bool:
        return self._battery_after_action_pct(target, action) >= self.safety_battery_pct

    def _allowed_drone_actions(self) -> List[actions]:
        return [a for a in self.sim.all_drone_actions if a.is_alowed()]

    def _allowed_cleaner_actions(self, idx: int) -> List[actions]:
        return [a for a in self.sim.all_clener_actions[idx] if a.is_alowed()]

    def _choose_drone_action(self) -> actions:
        drone = self.map.drone
        allowed = self._allowed_drone_actions()
        if not allowed:
            return null_action(drone)

        # If battery is low, prefer charging or returning to base.
        if getattr(drone, "battery_level", 100.0) <= (self.safety_battery_pct + 5.0):
            charge_opts = [a for a in allowed if isinstance(a, charge_drone)]
            if charge_opts and self._is_action_safe_for_battery(drone, charge_opts[0]):
                return charge_opts[0]
            rtb_opts = [a for a in allowed if isinstance(a, return_to_base)]
            if rtb_opts and self._is_action_safe_for_battery(drone, rtb_opts[0]):
                return rtb_opts[0]
            return null_action(drone)

        # If carrying a cleaner, head to nearest dirty window and drop off.
        if getattr(drone, "load", None) is not None:
            target_w = self._nearest_dirty_window(drone.pos3d)
            if target_w is None:
                # No dirty windows left; return to base or stop flying.
                rtb_opts = [a for a in allowed if isinstance(a, return_to_base)]
                if rtb_opts and self._is_action_safe_for_battery(drone, rtb_opts[0]):
                    return rtb_opts[0]
                return null_action(drone)

            # Prefer flying to that window if allowed; else drop if already there.
            fly_opts = [a for a in allowed if isinstance(a, fly_to_window) and a.window.this_id == target_w.this_id]
            if fly_opts and self._is_action_safe_for_battery(drone, fly_opts[0]):
                return fly_opts[0]

            drop_opts = [a for a in allowed if isinstance(a, dropoff_cleaner) and a.window.this_id == target_w.this_id]
            if drop_opts and self._is_action_safe_for_battery(drone, drop_opts[0]):
                return drop_opts[0]

            # If specific target not allowed, pick any safe dropoff or fly action.
            for a in allowed:
                if isinstance(a, (fly_to_window, dropoff_cleaner)) and self._is_action_safe_for_battery(drone, a):
                    return a

        # Not carrying a cleaner: pick up the best cleaner to deliver.
        # Prefer cleaner at base (fast pickup).
        pickup_base_opts = [a for a in allowed if isinstance(a, pickup_clean_at_base)]
        pickup_base_opts = [a for a in pickup_base_opts if self._is_action_safe_for_battery(drone, a)]
        if pickup_base_opts:
            # Choose the cleaner with highest battery among pickup options.
            pickup_base_opts.sort(key=lambda a: getattr(a.cleaner, "battery_level", 100.0), reverse=True)
            return pickup_base_opts[0]

        # Else, pick up any idle cleaner (not cleaning/charging), then fly to nearest dirty window.
        pickup_opts = [a for a in allowed if isinstance(a, pickup_cleaner)]
        pickup_opts = [a for a in pickup_opts if self._is_action_safe_for_battery(drone, a)]
        if pickup_opts:
            pickup_opts.sort(key=lambda a: getattr(a.cleaner, "battery_level", 100.0), reverse=True)
            return pickup_opts[0]

        # If nothing actionable toward cleaning, consider charging preemptively when lowish.
        charge_opts = [a for a in allowed if isinstance(a, charge_drone)]
        if charge_opts and self._is_action_safe_for_battery(drone, charge_opts[0]) and drone.battery_level <= 30.0:
            return charge_opts[0]

        # As a fallback, return to base or do nothing.
        rtb_opts = [a for a in allowed if isinstance(a, return_to_base)]
        if rtb_opts and self._is_action_safe_for_battery(drone, rtb_opts[0]):
            return rtb_opts[0]
        return null_action(drone)

    def _choose_cleaner_action(self, idx: int) -> actions:
        cleaner = self.map.cleaners[idx]
        allowed = self._allowed_cleaner_actions(idx)
        if not allowed:
            return null_action(cleaner)

        # If on a window and window is dirty, start cleaning if safe.
        if getattr(cleaner, "on_window", None) is not None and getattr(cleaner.on_window, "state", "dirty") == "dirty":
            c_opts = [a for a in allowed if isinstance(a, clean_window)]
            for a in c_opts:
                if self._is_action_safe_for_battery(cleaner, a):
                    return a

        # If battery is low and charging is possible, prefer charging.
        if getattr(cleaner, "battery_level", 100.0) <= (self.safety_battery_pct + 5.0):
            ch_opts = [a for a in allowed if isinstance(a, charge_cleaner)]
            if ch_opts and self._is_action_safe_for_battery(cleaner, ch_opts[0]):
                return ch_opts[0]

        # Otherwise, idle when no meaningful action is available.
        return null_action(cleaner)

    def choose_and_apply_actions(self) -> None:
        """Compute actions and apply them to the simulation state."""
        # Drone decision
        drone_action = self._choose_drone_action()
        # Directly set the current drone action to avoid dependency on incomplete update methods
        self.sim.curent_actions_drone_action = drone_action

        # Cleaners decisions
        for i in range(len(self.map.cleaners)):
            cleaner_action = self._choose_cleaner_action(i)
            # Replace action for cleaner i
            self.sim.clener_actions[i] = cleaner_action


def run_mpc(sim: simulation, duration_s: float) -> None:
    """Run the simulation under MPC control for a given duration."""
    ctrl = MPCController(sim)
    end_time = sim.time + duration_s
    while sim.time < end_time:
        ctrl.choose_and_apply_actions()
        sim.step(sim.dt)
        if sim.real_time and sim.sleep_time > 0:
            time.sleep(sim.sleep_time)


def run_mpc_with_visualization(
    num_cleaners: int = 4,
    num_windows: int = 10,
    sim_duration: float = 120.0,
    update_interval_ms: int = 50,
) -> None:
    """Run MPC-driven simulation with live visualization."""
    test_map = random_map_generater(num_cleaners=num_cleaners, num_windows=num_windows)
    sim = simulation(map=test_map, sleep_time=update_interval_ms / 1000.0, real_time=True)
    viz = create_live_visualization(test_map, update_interval_ms=update_interval_ms)

    import threading

    t = threading.Thread(target=run_mpc, args=(sim, sim_duration), daemon=True)
    t.start()
    viz.show()


if __name__ == "__main__":
    # Quick start: run MPC with visualization.
    # Adjust parameters to your liking.
    run_mpc_with_visualization(
        num_cleaners=4,
        num_windows=10,
        sim_duration=300.0,
        update_interval_ms=50,
    )
