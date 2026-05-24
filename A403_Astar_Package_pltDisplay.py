# A* Algorithm "pathfinding" package "plt" Demo


from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
import numpy as np

# 10x10 map
matrix = [
    [1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,1,1,1,0,1],
    [1,1,1,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,1,0,1],
    [1,1,1,0,1,1,1,1,0,0],
    [1,0,0,1,1,1,0,1,0,1],
    [1,0,1,0,0,0,0,1,0,1],
    [1,0,1,1,1,1,1,1,0,1],
    [0,0,0,0,1,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1]
]


# A* path search
grid = Grid(matrix=matrix)
start = grid.node(0, 0)
end = grid.node(9, 9)
finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
path, run_count = finder.find_path(start, end, grid)

# Print result report
print("===== A* Search Result Report =====")
print(f"Total checked nodes: {run_count}")
print(f"Path total steps: {len(path)}")
print("Path coordinate sequence:")
coord_list = [(p.x, p.y) for p in path]
print(coord_list)

# Extract coordinate
x = [p.x for p in path]
y = [p.y for p in path]

# QtAgg visualization
plt.figure(figsize=(6,6))
plt.imshow(matrix, cmap="gray_r", origin="upper")
plt.plot(x, y, "cyan", linewidth=2, label="Shortest Path")
plt.scatter(x[0], y[0], c="yellow", s=150, label="Start Point")
plt.scatter(x[-1], y[-1], c="red", s=150, label="End Point")
plt.legend()
plt.title("A* Path Planning")
plt.show()