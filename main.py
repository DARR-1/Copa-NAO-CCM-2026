from robot import Robot
import time
from behaviors.goalie import Goalie
from behaviors.race import Race

def main():
    robot = Robot("172.27.64.1", 9559)
    robot.init()
    
    while not robot.get_rear_head_touch():
        Goalie(robot).run()

    # Race(robot).run()

if __name__ == "__main__":
    main()