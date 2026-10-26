import math
import heapq
import copy
from vision.tag_detector import TagDetector

BOARD_SIZE = 5
CENTER_CELL = (BOARD_SIZE * BOARD_SIZE) // 2

CELL_SIZE = 0.244
VISITED_CELL_COST = 2

DIRECTION_OFFSETS = [-BOARD_SIZE, 1, BOARD_SIZE, -1]


# [N, E, S, W]
walls = [[False] * 4 for _ in range(25)] # 25 Cells, each with 4 directions (N, E, S, W) one wall in each direction in order of the ids 0, 1, 2, 3, ...

walls_angles = [0, -math.pi/2, math.pi, math.pi/2]  # N, E, S, W


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def update_walls(id_tag, theta_tag, theta_robot):
    """
    Updates the walls list based on the robot's position and orientation.

    :param id_tag: The ID of the tag (cell) the robot is currently in
    :param theta_tag: The angle of the tag (cell) the robot is currently in
    :param theta_robot: The angle of the robot plus the angle of the head in yaw
    """
    theta_abs = (theta_tag + theta_robot) % (2 * math.pi) - math.pi  # [-pi, pi]

    if theta_abs == -math.pi:
        theta_abs = math.pi

    most_similar_diff = math.inf
    most_similar_id = 0

    for i, wall_theta in enumerate(walls_angles):
        diff = abs(theta_abs - wall_theta)
        
        if diff < most_similar_diff:
            most_similar_diff = diff
            most_similar_id = i

    walls[id_tag][most_similar_id] = True

def get_neighbors(cell):
    row, column = divmod(cell, BOARD_SIZE)
    neighbors = []

    for direction, offset in enumerate(DIRECTION_OFFSETS):
        if walls[cell][direction]:
            continue

        if direction == 0 and row == 0:
            continue
        if direction == 1 and column == BOARD_SIZE - 1:
            continue
        if direction == 2 and row == BOARD_SIZE - 1:
            continue
        if direction == 3 and column == 0:
            continue

        neighbor = cell + offset
        opposite_direction = (direction + 2) % 4

        if walls[neighbor][opposite_direction]:
            continue

        neighbors.append(neighbor)

    return neighbors


def dijkstra_to_center(start_cell, goal_cell=CENTER_CELL, visited_cells=None):
    distances = {cell: math.inf for cell in range(BOARD_SIZE * BOARD_SIZE)}
    previous_cell = {}
    distances[start_cell] = 0
    priority_queue = [(0, start_cell)]

    while priority_queue:
        current_distance, current_cell = heapq.heappop(priority_queue)

        if current_distance > distances[current_cell]:
            continue
        if current_cell == goal_cell:
            break

        for neighbor in get_neighbors(current_cell):
            step_cost = 1
            if visited_cells is not None and neighbor in visited_cells:
                step_cost = VISITED_CELL_COST

            new_distance = current_distance + step_cost

            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                previous_cell[neighbor] = current_cell
                heapq.heappush(priority_queue, (new_distance, neighbor))

    if distances[goal_cell] == math.inf:
        return None

    path = [goal_cell]
    while path[-1] != start_cell:
        path.append(previous_cell[path[-1]])
    path.reverse()

    return path

def convert_path_to_directions(path):
    directions = []
    last_cell = path[0]

    for cell in path:
        row, column = divmod(cell, BOARD_SIZE)
        last_row, last_column = divmod(last_cell, BOARD_SIZE)

        if row < last_row:
            directions.append(0) # North
        elif row > last_row:
            directions.append(math.pi) # South
        elif column < last_column:
            directions.append(math.pi / 2) # West
        elif column > last_column:
            directions.append(-math.pi / 2) # East

        last_cell = cell

    return directions

class Maze:
    EXPLORING = "exploring"
    RETURNING = "returning"
    FINAL_RUN = "final_run"
    FINISHED = "finished"

    def __init__(self, robot):
        self.robot = robot
        self.tag_detector = TagDetector()
        self.state = Maze.EXPLORING  # EXPLORING: going to the center, RETURNING: going back to the start discovering, FINAL_RUN: shortest path to the center
        self.started = False

        self.current_cell = 0
        self.visited_cells = {0}
        self.known_walls = copy.deepcopy(walls)

        self.path = []
        self.directions = []
        self.step_index = 0

    def get_goal_cell(self):
        if self.state == Maze.RETURNING:
            return 0
        return CENTER_CELL

    def plan(self):
        visited_cells = self.visited_cells if self.state == Maze.RETURNING else None  # Penalize visited cells on the way back to discover new ones
        self.path = dijkstra_to_center(self.current_cell, self.get_goal_cell(), visited_cells)

        if self.path:
            self.directions = convert_path_to_directions(self.path)
        else:
            self.directions = []

        self.step_index = 0
        print(f"New path: {self.path}")

    def advance_state(self):
        if self.state == Maze.EXPLORING:
            print("Reached the center. Now returning to the start.")
            self.state = Maze.RETURNING
        elif self.state == Maze.RETURNING:
            print("Returned to the start. Stopping.")
            self.robot.wait_for_start()  # Wait for the user to press the head button to restart
            self.state = Maze.FINAL_RUN
        else:
            print("Final run completed.")
            self.state = Maze.FINISHED
            return

        self.plan()

    def update_from_detection(self):
        detection, _ = self.tag_detector.get_actual_detection(show_video=True)

        if not detection:
            return

        id_tag, theta_tag = detection
        position_changed = id_tag != self.current_cell

        self.current_cell = id_tag
        self.visited_cells.add(id_tag)

        theta_robot = self.robot.get_yaw() + self.robot.get_head_yaw_angle()
        update_walls(id_tag, theta_tag, theta_robot)

        if walls != self.known_walls or position_changed:
            self.known_walls = copy.deepcopy(walls)
            self.plan()

            print(f"Detected tag {id_tag} with angle {theta_tag:.2f} radians. Updated walls.")

    def step(self):
        if self.step_index >= len(self.directions):
            return

        target_yaw = self.directions[self.step_index]
        rotation = normalize_angle(target_yaw - self.robot.get_yaw())

        self.robot.move_to(0, 0, rotation)  # Move towards the first direction in the path
        self.robot.move_to(CELL_SIZE, 0, 0)

        self.step_index += 1
        self.current_cell = self.path[self.step_index]
        self.visited_cells.add(self.current_cell)

    def run(self):
        if not self.started:
            self.robot.wait_for_start()  # Wait for the user to press the head button to start
            self.started = True
            self.plan()

        if self.state == Maze.FINISHED:
            return

        self.update_from_detection()

        if self.current_cell == self.get_goal_cell():
            self.advance_state()
            return

        self.step()