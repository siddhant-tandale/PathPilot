# PathPilot

A* path planning on randomly generated 2D occupancy grids, written in Python.

PathPilot generates a grid map with randomly placed obstacles, plans a collision-free path from start to goal using A* with 8-connected movement, and visualizes the result with Matplotlib.

## How it works

- **Map generation** (`generate_map.py`): builds an N x N occupancy grid (default 50 x 50) at a configurable obstacle density (default 30%), keeps the start and goal cells free, and saves it to `map_data.txt`.
- **Planning** (`pathfinding.py`): A* search over 8-connected neighbors using a priority queue (`heapq`), an octile-distance heuristic for diagonal moves, and collision checks against the grid.
- **Visualization** (`visualization.py`): renders the obstacles and planned path with Matplotlib.
- **Entry point** (`main.py`): loads the map, runs the planner, and writes the path and summary metrics to `path_output.txt` and `path_results.txt`.

## Quick start

```bash
pip install -r requirements.txt
python src/generate_map.py
python src/main.py
```

## Next steps

- Compare A* against Dijkstra and RRT on the same maps
- Path smoothing for a differential-drive robot
- Benchmark planning time against map size and obstacle density
