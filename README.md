# PathPilot

PathPilot is a small Python project for generating grid maps and finding paths through them. It includes simple map generation, a pathfinding implementation, and basic visualization utilities used for experimenting with path planning algorithms.

**Key Features**
- **Map generation:** Create random or structured grid maps saved to `map_data.txt`.
- **Pathfinding:** Compute paths and output results to `path_output.txt` and `path_results.txt`.
- **Visualization:** Visual helpers in `visualization.py` to inspect maps and paths.

**Repository Structure**
- `src/`: Main source files.
	- `generate_map.py`: Tools to generate or convert map data.
	- `load_map.py`: Load a `map_data.txt` file into memory.
	- `pathfinding.py`: Pathfinding algorithm(s) used by the project.
	- `main.py`: Simple CLI-style entrypoint to run generation and pathfinding flows.
	- `visualization.py`: Rendering helpers for debugging and display.
	- `map_data.txt`, `path_output.txt`, `path_results.txt`: Example data files used by the project.

**Requirements**
- Python 3.8+ (recommended). Install dependencies if any are added to `requirements.txt`.

**Quick Start**
1. Create a virtual environment (recommended):

```
python3 -m venv .venv
source .venv/bin/activate
```

2. Install dependencies (if present):

```
pip install -r requirements.txt
```

3. Generate or edit a map and run the main program:

```
python src/generate_map.py    # create or update `src/map_data.txt`
python src/main.py           # runs pathfinding and outputs results
```

4. Inspect outputs:

- `src/path_output.txt`: step-by-step path output or raw solver output.
- `src/path_results.txt`: summarized path metrics and results.
- `src/map_data.txt`: current map used for runs.

**Development Notes**
- The code is intentionally small and educational — feel free to refactor or swap the pathfinding algorithm in `pathfinding.py`.
- If you add dependencies, update `requirements.txt` and the README installation steps.

**Contributing**
- Open an issue or submit a PR. Keep changes focused and include tests or examples when applicable.

**License & Contact**
- This repository does not include an explicit license file. Contact the maintainer for reuse permissions.

Enjoy exploring path planning!