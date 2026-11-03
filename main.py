from behaviors.maze import Maze
from robot import Robot
import time
from behaviors.goalie import Goalie
from behaviors.race import Race

def main():
    robot = Robot("192.168.0.151", 9561)
    robot.init()
    robot.prepare_for_move()
    robot.wait_for_start()

    maze = Maze(robot)
    while not robot.get_rear_head_touch():
        maze.run()

    Race(robot).run()

    robot.rest()

if __name__ == "__main__":
    main()