"""
Example of running a simulation with live visualization.
The map visualization updates in real-time as the simulation progresses.
"""

from map import random_map_generater
from simulation import simulation
from live_visualizer import create_live_visualization
import matplotlib.pyplot as plt
import threading
import time


def run_simulation_with_live_viz(num_cleaners=3, num_windows=5, sim_duration=100, update_interval_ms=200):
    """
    Run simulation with live visualization updating in real-time.
    
    Args:
        num_cleaners: Number of cleaner robots
        num_windows: Number of windows to clean
        sim_duration: Total simulation duration in seconds
        update_interval_ms: Visualization update interval in milliseconds
    """
    # Create map and simulation
    test_map = random_map_generater(num_cleaners=num_cleaners, num_windows=num_windows)
    sim = simulation(map=test_map,sleep_time=update_interval_ms / 1000.0)
    
    # Create live visualizer
    visualizer = create_live_visualization(test_map, update_interval_ms=update_interval_ms)
    

    # Start simulation thread
    sim_thread = threading.Thread(target=sim.run_in_time, args=(sim_duration,), daemon=True)
    sim_thread.start()
    
    # Show visualization (blocking)
    visualizer.show()
           

def run_simple_visualization(num_cleaners=3, num_windows=5, update_interval_ms=100):
    """
    Show live visualization without running simulation.
    (Useful for testing the visualization interface)
    """
    test_map = random_map_generater(num_cleaners=num_cleaners, num_windows=num_windows)
    visualizer = create_live_visualization(test_map, update_interval_ms=update_interval_ms)
    visualizer.show()


if __name__ == "__main__":
    # Option 1: Just visualize (no simulation)
    # run_simple_visualization(num_cleaners=3, num_windows=5)
    
    # Option 2: Run simulation with live visualization
    run_simulation_with_live_viz(
        num_cleaners=4,
        num_windows=10,
        sim_duration=1000,  # 60 seconds of simulation
        update_interval_ms=50  # Update visualization every 100ms
    )
