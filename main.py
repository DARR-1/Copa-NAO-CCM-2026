from robot import Robot
import time
from behaviors.goalie import Goalie
from behaviors.race import Race

def main():
    robot = Robot("192.168.0.151", 9561)
    robot.init()
    
    robot.wait_for_start()
    while not robot.get_rear_head_touch():
        Goalie(robot).run()

    robot.rest()

    # Race(robot).run()

if __name__ == "__main__":
    main()