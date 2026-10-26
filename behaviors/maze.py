import math

# [N, E, S, W]
walls = [[False, False, False, False]]*25 # 25 Cells, each with 4 directions (N, E, S, W) one wall in each direction in order of the ids 0, 1, 2, 3, ...

walls_angles = [0, -math.pi/2, math.pi, math.pi/2]  # N, E, S, W

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
