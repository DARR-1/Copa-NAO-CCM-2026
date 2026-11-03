import math
import heapq
import copy
from vision.tag_detector import TagDetector

BOARD_SIZE = 5
CENTER_CELL = (BOARD_SIZE * BOARD_SIZE) // 2

CELL_SIZE = 0.24 * 2
VISITED_CELL_COST = 2

# Bump detection while walking sideways (calibrate on the robot: watch the printed values on a normal step)
BUMP_ROLL = math.radians(8)  # torso lean from the value before the move
BUMP_SHOULDER_ERROR = 0.25  # rad between commanded and real shoulder roll
BUMP_HITS = 2  # consecutive readings needed, so a single noisy sample doesn't stop the robot

# Foot bumper hits are wall edges, not walls: back off a little and finish the side step
EDGE_BACKOFF = 0.03  # m backwards before retrying
MAX_EDGE_FIXES = 2  # after this many edge hits in one step, treat it as a wall

HEAD_PITCH = math.radians(10)  # positive = down; tags are all at about the same height, so it stays fixed

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
    # theta_tag is the tag's z axis (pointing into the wall) in the camera frame, positive = to the camera's right;
    # yaw is positive to the left, so the wall direction in the world is theta_robot - theta_tag
    theta_abs = normalize_angle(theta_robot - theta_tag)

    most_similar_diff = math.inf
    most_similar_id = 0

    for i, wall_theta in enumerate(walls_angles):
        diff = abs(normalize_angle(theta_abs - wall_theta))  # wrapped, so -179 deg is close to 180 deg (S)
        
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
        self.tag_detector = TagDetector(robot.session)
        self.state = Maze.EXPLORING  # EXPLORING: going to the center, RETURNING: going back to the start discovering, FINAL_RUN: shortest path to the center
        self.started = False

        self.current_cell = 0
        self.visited_cells = {0}
        self.known_walls = copy.deepcopy(walls)

        self.path = []
        self.directions = []
        self.step_index = 0

        self.robot.set_yaw(0)  # Ensure the robot starts facing north
        self.robot.look(HEAD_PITCH, 0)

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

    def update_from_detection(self, replan=True):
        # replan=False while a move is running: only record walls, because step() relies on the current path
        id_tag, theta_tag, _ = self.tag_detector.get_actual_detection(show_video=True)

        position_changed = False

        # Tag 0 is a valid cell, so check for None instead of falsy
        if id_tag is not None:
            theta_robot = self.robot.get_yaw() + self.robot.get_head_yaw_angle()
            update_walls(id_tag, theta_tag, theta_robot)

            if replan:
                position_changed = id_tag != self.current_cell
                self.current_cell = id_tag
                self.visited_cells.add(id_tag)

        # Also catches walls seen during the last move, even if no tag is visible now
        if replan and (walls != self.known_walls or position_changed):
            self.known_walls = copy.deepcopy(walls)
            self.plan()

            print(f"Replanned from cell {self.current_cell}. Walls updated.")

    def get_bump(self, start_roll):
        # "wall" = body/arm pushed sideways, "edge" = only the toe bumpers, None = nothing
        roll = abs(self.robot.get_roll() - start_roll)
        shoulder_error = self.robot.get_shoulder_error()
        print(f"    bump check: roll={math.degrees(roll):4.1f} deg  shoulder={shoulder_error:.2f} rad", end="\r")

        if roll > BUMP_ROLL or shoulder_error > BUMP_SHOULDER_ERROR:
            return "wall"
        if self.robot.get_foot_bumper():
            return "edge"
        return None

    def move_and_watch(self, x, y, theta, watch_bump=False):
        # Keep reading tags while walking instead of only after the move finishes.
        # With watch_bump, stops the move and returns "wall" or "edge" if the robot hits something, else None
        start_roll = self.robot.get_roll()
        hits = 0
        last_bump = None

        future = self.robot.move_to(x, y, theta, caution=True, wait=False)

        while not future.isFinished():
            self.update_from_detection(replan=False)

            if not watch_bump:
                continue

            bump = self.get_bump(start_roll)
            hits = hits + 1 if bump is not None else 0
            # A wall reading wins over an edge reading in the same streak
            last_bump = "wall" if "wall" in (bump, last_bump) else bump

            if hits >= BUMP_HITS:
                self.robot.stop_move()
                future.wait()
                return last_bump

            if bump is None:
                last_bump = None

        future.value()  # raises if the move failed
        return None

    def side_progress(self, start_position):
        # Sideways distance walked since start_position, in the robot frame at the start of the step
        x0, y0, yaw0 = start_position
        x1, y1, _ = self.robot.get_robot_position()
        return -math.sin(yaw0) * (x1 - x0) + math.cos(yaw0) * (y1 - y0)

    def handle_bump(self, target_yaw, start_position):
        print(f"Bumped into a wall going {target_yaw:.2f} rad from cell {self.current_cell}")

        # The wall is on the current cell's side we were walking toward
        wall_id = min(range(4), key=lambda i: abs(normalize_angle(target_yaw - walls_angles[i])))
        walls[self.current_cell][wall_id] = True

        # Walk back to where the step started (odometry, rotated into the robot frame)
        x0, y0, _ = start_position
        x1, y1, yaw = self.robot.get_robot_position()
        dx, dy = x0 - x1, y0 - y1
        back_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        back_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        self.robot.move_to(back_x, back_y, 0, caution=True)

        self.known_walls = copy.deepcopy(walls)
        self.plan()

    def step(self):
        if self.step_index >= len(self.directions):
            return

        target_yaw = self.directions[self.step_index]
        current_yaw = self.robot.get_yaw()

        # Rotacion necesaria para caminar de lado hacia la izquierda (+1) o la derecha (-1)
        best_side = 1
        best_rotation = None

        for side in (1, -1):
            desired_yaw = normalize_angle(target_yaw - side * math.pi / 2)
            rotation = normalize_angle(desired_yaw - current_yaw)

            if best_rotation is None or abs(rotation) < abs(best_rotation):
                best_side = side
                best_rotation = rotation

        self.move_and_watch(0, 0, best_rotation)  # Girar (maximo 90 grados)

        # Mirar hacia donde va a caminar
        self.robot.look(HEAD_PITCH, best_side * math.pi / 2)

        start_position = self.robot.get_robot_position()
        side_distance = best_side * CELL_SIZE
        remaining = side_distance
        backed_off = 0.0

        for edge_fixes in range(MAX_EDGE_FIXES + 1):
            bump = self.move_and_watch(0, remaining, 0, watch_bump=True)  # Avanzar de lado

            if bump is None:
                break

            if bump == "wall" or edge_fixes == MAX_EDGE_FIXES:
                self.handle_bump(target_yaw, start_position)  # stays in the same cell with a new path
                return

            # Toe hit an edge: step back off it and finish the rest of the side step
            print(f"\nHit an edge, backing off {EDGE_BACKOFF:.2f} m")
            self.robot.move_to(-EDGE_BACKOFF, 0, 0, caution=True)
            backed_off += EDGE_BACKOFF
            remaining = side_distance - self.side_progress(start_position)

        if backed_off > 0:
            self.robot.move_to(backed_off, 0, 0, caution=True)  # back to the middle of the row

        self.step_index += 1
        self.current_cell = self.path[self.step_index]
        self.visited_cells.add(self.current_cell)

    def run(self):
        if not self.started:
            self.robot.wait_for_start()  # Wait for the user to press the head button to start
            self.started = True
            self.plan()

        if self.state == Maze.FINISHED:
            self.robot.say("Maze completed.")
            self.robot.dance()
            return

        self.update_from_detection()

        if self.current_cell == self.get_goal_cell():
            self.advance_state()
            return

        self.step()