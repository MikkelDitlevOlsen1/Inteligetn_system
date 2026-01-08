import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from map import Map


class LiveVisualizer:
    """Live visualization that updates as the map object changes."""
    
    def __init__(self, map_obj: Map, update_interval_ms=100):
        """
        Initialize the live visualizer.
        
        Args:
            map_obj: The Map object to visualize
            update_interval_ms: Update interval in milliseconds
        """
        self.map = map_obj
        self.update_interval = update_interval_ms
        
        # Create figure and 3D axis
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Store scatter plot references for updating
        self.base_station_scatter = None
        self.drone_scatter = None
        self.cleaners_scatter = None
        self.windows_scatter = None
        
        # Store text annotations
        self.texts = []
        
        # Set up the plot
        self._setup_plot()
        
    def _setup_plot(self):
        """Initialize plot with labels and settings."""
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Live Map Visualization')
        self.ax.grid(True)
        
        # Set reasonable limits
        self.ax.set_xlim(-60, 60)
        self.ax.set_ylim(-60, 60)
        self.ax.set_zlim(0, 120)
        
    def _clear_texts(self):
        """Remove all text annotations from plot."""
        for text in self.texts:
            text.remove()
        self.texts = []
    
    def _update_plot(self, frame):
        """Update plot with current map state. Called by FuncAnimation."""
        # Clear previous scatter plots and texts
        if self.base_station_scatter:
            self.base_station_scatter.remove()
            self.base_station_scatter = None

        if self.drone_scatter:
            self.drone_scatter.remove()
            self.drone_scatter = None

        # Remove cleaner/window artists from previous frame
        if isinstance(self.cleaners_scatter, list):
            for sc in self.cleaners_scatter:
                try:
                    sc.remove()
                except Exception:
                    pass
            self.cleaners_scatter = []
        else:
            # Initialize for first use if it was None in __init__
            self.cleaners_scatter = []

        if isinstance(self.windows_scatter, list):
            for sc in self.windows_scatter:
                try:
                    sc.remove()
                except Exception:
                    pass
            self.windows_scatter = []
        else:
            # Initialize for first use if it was None in __init__
            self.windows_scatter = []

        self._clear_texts()
        
        # Base station
        try:
            bx, by, bz = tuple(self.map.base_station.pos3d)
            self.base_station_scatter = self.ax.scatter(bx, by, bz, c='black', marker='s', s=100, label='Base Station', depthshade=False)
            text = self.ax.text(bx, by, bz + 2, 'Base', color='black', fontsize=9)
            self.texts.append(text)
        except Exception as e:
            print(f"Error plotting base station: {e}")
        
        # Drone
        try:
            dx, dy, dz = tuple(self.map.drone.pos3d)
            drone_color = 'red' if not self.map.drone.ucupied else 'orange'
            self.drone_scatter = self.ax.scatter(dx, dy, dz, c=drone_color, marker='^', s=80, label='Drone', depthshade=False)
            status = "Idle" if not self.map.drone.ucupied else "Occupied"
            text = self.ax.text(dx, dy, dz + 2, f'Drone ({status})', color=drone_color, fontsize=9)
            self.texts.append(text)
            
            
        except Exception as e:
            print(f"Error plotting drone: {e}")
        
        # Cleaners

        cx_list = []
        cy_list = []
        cz_list = []
        cleaner_names = []
        cleaner_colors = []
        
        for cleaner in self.map.cleaners:
            pos = cleaner.pos3d
            if pos is None:
                continue
            
            x, y, z = tuple(pos)
            cx_list.append(x)
            cy_list.append(y)
            cz_list.append(z)
            
            # Color based on state
            if getattr(cleaner, 'is_charging', False):
                color = 'purple'
            elif getattr(cleaner, 'is_cleaning', False):
                color = 'cyan'
            else:
                color = 'blue'
            cleaner_colors.append(color)
            
            name = getattr(cleaner, 'name', 'C')
            cleaner_names.append(name)
            battery = getattr(cleaner, 'battery_level', 100)
            text = self.ax.text(x, y, z + 2, f'{name}({battery:.0f}%)', color=color, fontsize=8)
            self.texts.append(text)
        
        if cx_list:
            # Plot each cleaner with its color and store handles for removal
            for cx, cy, cz, color in zip(cx_list, cy_list, cz_list, cleaner_colors):
                sc = self.ax.scatter(cx, cy, cz, c=color, marker='o', s=60, depthshade=False)
                self.cleaners_scatter.append(sc)

        
        # Windows
        try:
            wx_list = []
            wy_list = []
            wz_list = []
            window_colors = []
            
            for window in self.map.windows:
                pos = getattr(window, 'pos3d', None)
                if pos is None:
                    continue
                
                x, y, z = tuple(pos)
                wx_list.append(x)
                wy_list.append(y)
                wz_list.append(z)
                
                # Color by state
                state = getattr(window, 'state', 'dirty')
                color = 'green' if state == 'clean' else 'red'
                window_colors.append(color)
                
                name = getattr(window, 'name', 'W')
                text = self.ax.text(x, y, z + 2, f'{name}({state[0].upper()})', color=color, fontsize=8)
                self.texts.append(text)
            
            if wx_list:
                # Plot each window with its color and store handles for removal
                for wx, wy, wz, color in zip(wx_list, wy_list, wz_list, window_colors):
                    sc = self.ax.scatter(wx, wy, wz, c=color, marker='x', s=80, depthshade=False)
                    self.windows_scatter.append(sc)
        except Exception as e:
            print(f"Error plotting windows: {e}")
        
        # Update title with current simulation time
        time_str = f"Time: {self.map.time:.1f}s" if hasattr(self.map, 'time') else "Time: N/A"
        self.ax.set_title(f'Live Map Visualization - {time_str}')
        
        return [self.base_station_scatter, self.drone_scatter] + self.texts if self.base_station_scatter else self.texts
    
    def show(self):
        """Display the live visualization."""
        anim = FuncAnimation(self.fig, self._update_plot, interval=self.update_interval, blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()
        return anim


def create_live_visualization(map_obj: Map, update_interval_ms=100):
    """
    Convenience function to create and show a live visualization.
    
    Args:
        map_obj: The Map object to visualize
        update_interval_ms: Update interval in milliseconds
    
    Returns:
        LiveVisualizer instance
    """
    visualizer = LiveVisualizer(map_obj, update_interval_ms)
    return visualizer


if __name__ == "__main__":
    # Example usage
    from map import random_map_generater
    
    test_map = random_map_generater(num_cleaners=3, num_windows=5)
    visualizer = create_live_visualization(test_map, update_interval_ms=100)
    visualizer.show()
