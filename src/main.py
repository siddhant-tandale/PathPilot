"""
Course Number: ENGR 13300
Semester: Fall 2025

Description:
    A program that generates a random grid map with obstacles,
    loads the map, takes user input for start and goal points,
    implements the A* pathfinding algorithm with diagonal movement
    and collision checking, visualizes the path using Matplotlib,
    and saves the path to a text file.

Assignment Information:
    Assignment:     18
    Team ID:        LC5
    Author:         Siddhant Tandale, standale@purdue.edu
    Date:           09/29/2025

Contributors:
    ChatGPT

    My contributor(s) helped me:
    [Y] understand the assignment expectations without
        telling me how they will approach it.
    [Y] understand different ways to think about a solution
        without helping me plan my solution.
    [Y] think through the meaning of a specific error or
        bug present in my code without looking at my code.
    Note that if you helped somebody else with their code, you
    have to list that person as a contributor here as well.

Academic Integrity Statement:
    I have not used source code obtained from any unauthorized
    source, either modified or unmodified; nor have I provided
    another student access to my code.  The project I am
    submitting is my own original work.
"""



"""
Author: Sid Tandale
Program: Main Program
Description: Loads map, gets user input, runs A* pathfinding, visualizes and saves the path.
"""

# Imports
from load_map import load_map
from pathfinding import a_star, check_collision
from visualization import visualize

# Main function
def main():
    """
    Main execution flow.
    1. Load map from file.
    2. Get user input for start and goal points.
    3. Run A* pathfinding.
    4. Visualize and save the path.
    """

    # Load the map
    grid = load_map("map_data.txt")
    print("Map loaded successfully.")

    # Get user input for start and goal
    while True:
        try:
            sx, sy = map(int, input("Enter start x y: ").split())
            gx, gy = map(int, input("Enter goal x y: ").split())

            if check_collision(sx, sy, grid) or check_collision(gx, gy, grid):
                continue

            break

        except ValueError:
            print("Invalid input. Enter two integers.")

    # Run A* pathfinding
    path = a_star((sx, sy), (gx, gy), grid)

    # Visualize and save the path
    if path:
        print(f"Path found with {len(path)} steps.")
        visualize(grid, path)

        with open("path_output.txt", "w") as f:
            for p in path:
                f.write(f"{p}\n")
        print("Path saved to path_output.txt.")
    else:
        print("No valid path could be created.")


# Run main
if __name__ == "__main__":
    main()