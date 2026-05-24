# A* Algorithm "pathfinding" package GUI Demo

import tkinter as tk
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

# PARAMETERS
GRID_SIZE = 10
CELL_SIZE = 60
EMPTY = 0
OBSTACLE = 1
START = 2
END = 3
PATH = 4


class AStarGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("A* Pathfinding (pathfinding Library)")

        # Grid data
        self.grid = [[EMPTY for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        self.start_pos = None
        self.end_pos = None
        self.mode = "start"  # start, end, obs

        # Canvas
        self.canvas = tk.Canvas(root, width=GRID_SIZE * CELL_SIZE, height=GRID_SIZE * CELL_SIZE, bg="white")
        self.canvas.pack()

        # Buttons
        frame = tk.Frame(root)
        frame.pack(pady=5)
        tk.Button(frame, text="Set Start", command=lambda: self.set_mode("start")).grid(row=0, column=0, padx=5)
        tk.Button(frame, text="Set End", command=lambda: self.set_mode("end")).grid(row=0, column=1, padx=5)
        tk.Button(frame, text="Set Obstacle", command=lambda: self.set_mode("obs")).grid(row=0, column=2, padx=5)
        tk.Button(frame, text="Run A*", command=self.run_astar).grid(row=0, column=3, padx=5)
        tk.Button(frame, text="Clear", command=self.clear).grid(row=0, column=4, padx=5)

        self.canvas.bind("<Button-1>", self.on_click)
        self.draw_grid()

    def set_mode(self, mode):
        self.mode = mode

    def draw_grid(self):
        self.canvas.delete("all")
        color = {
            EMPTY: "white",
            OBSTACLE: "#222222",
            START: "#27ae60",
            END: "#f39c12",
            PATH: "#ffc0cb"
        }

        for row in range(GRID_SIZE):
            for col in range(GRID_SIZE):
                x1 = col * CELL_SIZE
                y1 = row * CELL_SIZE
                x2 = x1 + CELL_SIZE
                y2 = y1 + CELL_SIZE
                c = color[self.grid[row][col]]
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=c, outline="#cccccc")

    def on_click(self, e):
        col = e.x // CELL_SIZE
        row = e.y // CELL_SIZE
        if not (0 <= row < GRID_SIZE and 0 <= col < GRID_SIZE):
            return

        if self.mode == "start":
            if self.start_pos:
                old_r, old_c = self.start_pos
                self.grid[old_r][old_c] = EMPTY
            self.start_pos = (row, col)
            self.grid[row][col] = START

        elif self.mode == "end":
            if self.end_pos:
                old_r, old_c = self.end_pos
                self.grid[old_r][old_c] = EMPTY
            self.end_pos = (row, col)
            self.grid[row][col] = END

        elif self.mode == "obs":
            if self.grid[row][col] in (START, END):
                return
            self.grid[row][col] = OBSTACLE

        self.draw_grid()

    def clear(self):
        self.grid = [[EMPTY] * GRID_SIZE for _ in range(GRID_SIZE)]
        self.start_pos = None
        self.end_pos = None
        self.draw_grid()

    # -------------------------- CORE: A* from library --------------------------
    def run_astar(self):
        if not self.start_pos or not self.end_pos:
            return

        # Build matrix for pathfinding
        matrix = []
        for row in range(GRID_SIZE):
            mat_row = []
            for col in range(GRID_SIZE):
                mat_row.append(0 if self.grid[row][col] == OBSTACLE else 1)
            matrix.append(mat_row)

        grid = Grid(matrix=matrix)
        start_node = grid.node(self.start_pos[1], self.start_pos[0])
        end_node = grid.node(self.end_pos[1], self.end_pos[0])

        # A* finder with diagonal movement
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start_node, end_node, grid)

        # Draw result path
        for (x, y) in path:
            if self.grid[y][x] not in (START, END):
                self.grid[y][x] = PATH
        self.draw_grid()


if __name__ == "__main__":
    root = tk.Tk()
    app = AStarGUI(root)
    root.mainloop()