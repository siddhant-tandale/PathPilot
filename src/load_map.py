"""
Author: Sid Tandale
Program: Map Loader
Description: Loads a numeric grid map from file with error checking.
"""

# Imports
import numpy as np

# Map loading function
def load_map(filename):
    """
    Loads a grid map from a text file.
    If file not found, returns a default fallback map.
    """

    # Attempt to load the map
    try:
        grid = np.loadtxt(filename, dtype=int)

        if grid.ndim != 2:
            raise ValueError("Map must be 2D.")

    # Handle file not found
    except OSError:
        print("Error: File not found. Using fallback map.")
        grid = np.zeros((10, 10), dtype=int)
        grid[3:7, 5] = 1

    # Return the loaded or fallback map
    return grid