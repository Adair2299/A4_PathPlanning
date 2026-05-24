# A* Algorithm Hand-Coding GUI Demo

import tkinter as tk
import heapq
import time

GRID_SIZE = 10
CELL_SIZE = 60
STEP_DELAY = 0.5  # Time interval between each displaying step
SCALE = 10  # Side length of each grid cell
STRAIGHT_COST = 1 * SCALE
DIAG_COST = 1.4 * SCALE

EMPTY = 0
OBSTACLE = 1
START_NODE = 2
END_NODE = 3
EXPLORED = 4
FINAL_PATH = 5

class AStarPathGUI:
    def __init__(self, window):
        self.win = window
        self.win.title("A* Path Optimization")
        self.grid_data = [[EMPTY for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        self.start_pos = None
        self.end_pos = None
        self.current_mode = "start"
        self.g_cost = {}
        self.h_cost = {}

        self.canvas = tk.Canvas(window, width=GRID_SIZE*CELL_SIZE, height=GRID_SIZE*CELL_SIZE, bg="white")
        self.canvas.pack()

        btn_frame = tk.Frame(window)
        btn_frame.pack(pady=4)
        tk.Button(btn_frame, text="Set Start", command=lambda:self.set_mode("start")).grid(row=0, column=0, padx=2)
        tk.Button(btn_frame, text="Set End", command=lambda:self.set_mode("end")).grid(row=0, column=1, padx=2)
        tk.Button(btn_frame, text="Set Obstacle", command=lambda:self.set_mode("obs")).grid(row=0, column=2, padx=2)
        tk.Button(btn_frame, text="Start Search", command=self.execute_astar).grid(row=0, column=3, padx=2)
        tk.Button(btn_frame, text="Clear Board", command=self.reset_board).grid(row=0, column=4, padx=2)

        self.canvas.bind("<Button-1>", self.click_grid)
        self.draw_full_grid()

    def set_mode(self, mode):
        self.current_mode = mode

    def draw_full_grid(self):
        self.canvas.delete("all")
        color_dict = {
            EMPTY: "white",
            OBSTACLE: "#222222",
            START_NODE: "#27ae60",
            END_NODE: "#f39c12",
            EXPLORED: "#b0d8f5",
            FINAL_PATH: "#ffc0cb"
        }
        for row in range(GRID_SIZE):
            for col in range(GRID_SIZE):
                x1 = col * CELL_SIZE
                y1 = row * CELL_SIZE
                x2 = x1 + CELL_SIZE
                y2 = y1 + CELL_SIZE
                fill_color = color_dict[self.grid_data[row][col]]
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=fill_color, outline="#cccccc")

                if (row,col) in self.g_cost and self.grid_data[row][col] != OBSTACLE:
                    g = int(self.g_cost[(row,col)])
                    h = int(self.h_cost[(row,col)])
                    f = round(g + h,1)
                    self.canvas.create_text(x1+8, y1+8, text=str(g), font=("Arial",7), fill="red")
                    self.canvas.create_text(x2-8, y1+8, text=str(h), font=("Arial",7), fill="blue")
                    self.canvas.create_text(x1+8, y2-8, text=str(f), font=("Arial",7), fill="black")

    def click_grid(self, event):
        col = event.x // CELL_SIZE
        row = event.y // CELL_SIZE
        if not (0 <= row < GRID_SIZE and 0 <= col < GRID_SIZE):
            return

        if self.current_mode == "start":
            if self.start_pos:
                old_r, old_c = self.start_pos
                self.grid_data[old_r][old_c] = EMPTY
            self.start_pos = (row, col)
            self.grid_data[row][col] = START_NODE

        elif self.current_mode == "end":
            if self.end_pos:
                old_r, old_c = self.end_pos
                self.grid_data[old_r][old_c] = EMPTY
            self.end_pos = (row, col)
            self.grid_data[row][col] = END_NODE

        elif self.current_mode == "obs":
            if self.grid_data[row][col] in (START_NODE, END_NODE):
                return
            self.grid_data[row][col] = OBSTACLE
        self.draw_full_grid()

    def reset_board(self):
        self.grid_data = [[EMPTY]*GRID_SIZE for _ in range(GRID_SIZE)]
        self.start_pos = None
        self.end_pos = None
        self.g_cost = {}
        self.h_cost = {}
        self.draw_full_grid()

    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return SCALE * max(dx, dy) + (DIAG_COST - STRAIGHT_COST) * min(dx, dy)

    def execute_astar(self):
        if not self.start_pos or not self.end_pos:
            return
        open_list = []
        heapq.heappush(open_list, (0, self.start_pos))
        parent_record = dict()

        g_cost = {(r,c):float('inf') for r in range(GRID_SIZE) for c in range(GRID_SIZE)}
        g_cost[self.start_pos] = 0
        f_cost = {(r,c):float('inf') for r in range(GRID_SIZE) for c in range(GRID_SIZE)}
        f_cost[self.start_pos] = self.heuristic(self.start_pos, self.end_pos)

        move_dir = [(-1,0), (1,0), (0,-1), (0,1),
                    (-1,-1), (-1,1), (1,-1), (1,1)]
        reach_target = False

        while open_list:
            _, current = heapq.heappop(open_list)
            curr_r, curr_c = current

            if current == self.end_pos:
                reach_target = True
                break
            if self.grid_data[curr_r][curr_c] not in (START_NODE, END_NODE):
                self.grid_data[curr_r][curr_c] = EXPLORED
                self.g_cost[(curr_r,curr_c)] = g_cost[current]
                self.h_cost[(curr_r,curr_c)] = self.heuristic(current, self.end_pos)
                self.draw_full_grid()
                self.win.update()
                time.sleep(STEP_DELAY)

            for dr, dc in move_dir:
                next_r = curr_r + dr
                next_c = curr_c + dc
                if 0 <= next_r < GRID_SIZE and 0 <= next_c < GRID_SIZE:
                    if self.grid_data[next_r][next_c] == OBSTACLE:
                        continue
                    # 判断横竖/斜向步长代价
                    if dr != 0 and dc != 0:
                        step_cost = DIAG_COST
                    else:
                        step_cost = STRAIGHT_COST
                    new_g = g_cost[current] + step_cost
                    if new_g < g_cost[(next_r, next_c)]:
                        parent_record[(next_r, next_c)] = current
                        g_cost[(next_r, next_c)] = new_g
                        f_cost[(next_r, next_c)] = new_g + self.heuristic((next_r, next_c), self.end_pos)
                        heapq.heappush(open_list, (f_cost[(next_r, next_c)], (next_r, next_c)))

        if reach_target:
            back_node = self.end_pos
            while back_node in parent_record:
                br, bc = back_node
                if self.grid_data[br][bc] not in (START_NODE, END_NODE):
                    self.grid_data[br][bc] = FINAL_PATH
                back_node = parent_record[back_node]
                self.draw_full_grid()
                self.win.update()
                time.sleep(STEP_DELAY)

if __name__ == "__main__":
    main_window = tk.Tk()
    app = AStarPathGUI(main_window)
    main_window.mainloop()