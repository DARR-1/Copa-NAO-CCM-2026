from collections import deque
import matplotlib.pyplot as plt
import numpy as np

# 0 = camino libre
# 1 = pared
# S = inicio
# G = meta

maze = [
    ["S", 0,   1,   0,   0],
    [1,   0,   1,   0,   1],
    [0,   0,   0,   0,   1],
    [0,   1,   1,   0,   "G"]
]

start = (0, 0)
goal = (3, 4)


def flood_fill(maze, start, goal):
    rows = len(maze)
    cols = len(maze[0])

    distances = [
        [None for _ in range(cols)]
        for _ in range(rows)
    ]

    queue = deque([goal])

    distances[goal[0]][goal[1]] = 0

    directions = [
        (-1, 0), 
        (1, 0),   
        (0, -1), 
        (0, 1)    
    ]

    while queue:
        row, col = queue.popleft()

        for dr, dc in directions:
            new_row = row + dr
            new_col = col + dc

            if (
                0 <= new_row < rows
                and 0 <= new_col < cols
                and maze[new_row][new_col] != 1
                and distances[new_row][new_col] is None
            ):
                distances[new_row][new_col] = (
                    distances[row][col] + 1
                )

                queue.append(
                    (new_row, new_col)
                )

    return distances


def get_path(distances, start, goal):
    if distances[start[0]][start[1]] is None:
        return None

    path = [start]

    current = start

    directions = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1)
    ]

    while current != goal:
        row, col = current

        best = None
        best_value = distances[row][col]

        for dr, dc in directions:
            new_row = row + dr
            new_col = col + dc

            if (
                0 <= new_row < len(distances)
                and 0 <= new_col < len(distances[0])
            ):
                value = distances[new_row][new_col]

                if (
                    value is not None
                    and value < best_value
                ):
                    best_value = value
                    best = (new_row, new_col)

        if best is None:
            return None

        current = best

        path.append(current)

    return path


def print_maze(maze):
    print("Laberinto:\n")

    for row in maze:
        for cell in row:
            if cell == 1:
                print("|", end=" ")

            elif cell == "S":
                print("S", end=" ")

            elif cell == "G":
                print("G", end=" ")

            else:
                print(".", end=" ")

        print()



def print_distances(distances, maze):
    print("\nValores de flood fill:\n")

    for row in range(len(distances)):
        for col in range(len(distances[0])):

            if maze[row][col] == 1:
                print(" | ", end=" ")

            else:
                value = distances[row][col]

                if value is None:
                    print(" X ", end=" ")

                else:
                    print(f"{value:2}", end="  ")

        print()


def print_path_maze(maze, path):
    print("\nRuta encontrada:\n")

    path_set = set(path)

    for row in range(len(maze)):
        for col in range(len(maze[0])):

            cell = (row, col)

            if maze[row][col] == 1:
                print("|", end=" ")

            elif cell == start:
                print("S", end=" ")

            elif cell == goal:
                print("G", end=" ")

            elif cell in path_set:
                print("*", end=" ")

            else:
                print(".", end=" ")

        print()


def show_maze_graphically(maze, path=None):
    rows = len(maze)
    cols = len(maze[0])

    grid = np.zeros((rows, cols))

    for r in range(rows):
        for c in range(cols):
            if maze[r][c] == 1:
                grid[r][c] = 1
            else:
                grid[r][c] = 0

    plt.figure(figsize=(6, 6))
    plt.imshow(grid)

    for r in range(rows):
        for c in range(cols):

            if maze[r][c] == "S":
                plt.text(
                    c,
                    r,
                    "S",
                    ha="center",
                    va="center",
                    fontsize=14
                )

            elif maze[r][c] == "G":
                plt.text(
                    c,
                    r,
                    "G",
                    ha="center",
                    va="center",
                    fontsize=14
                )

    if path is not None:
        x = [col for row, col in path]
        y = [row for row, col in path]

        plt.plot(
            x,
            y,
            marker="o",
            linewidth=3
        )

    plt.xticks(range(cols))
    plt.yticks(range(rows))
    plt.grid(True)

    plt.title("Resolución del laberinto con Flood Fill")

    plt.show()


print_maze(maze)

distances = flood_fill(
    maze,
    start,
    goal
)

print_distances(
    distances,
    maze
)

path = get_path(
    distances,
    start,
    goal
)

if path is None:
    print("\nNo se encontró una ruta.")

else:
    print("\nCoordenadas de la ruta:")
    print(path)

    print(
        "\nNúmero de movimientos:",
        len(path) - 1
    )

    print_path_maze(
        maze,
        path
    )

    show_maze_graphically(
        maze,
        path
    )