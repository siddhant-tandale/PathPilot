"""
Author: Sid Tandale
Program: Visualization Module
Description: Visualizes the grid map and the path using Matplotlib.
"""

# Imports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# Visualization function
def visualize(grid, path):
    """
    Visualizes the grid map and the path using Matplotlib.
    grid: 2D numpy array (0 = free, 1 = obstacle)
    """

    # Create a display copy of the grid
    disp = np.copy(grid)

    # Mark path cells with value 2
    for (x, y) in path:
        disp[y, x] = 2

    # Define color map
    cmap = ListedColormap([
        "white",   # free
        "black",   # obstacle
        "red"      # path
    ])

    plt.imshow(disp, cmap=cmap)
    plt.title("Grid Map with A* Path (RED)")
    plt.show()