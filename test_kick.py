# Prueba de Kick en el NAO.
# Uso: python test_kick.py [IP] [PUERTO]
# ¡Ten a alguien sosteniendo/cuidando al robot! La patada es en una sola pierna.
import sys
from robot import Robot
from behaviors.kick import Kick

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.151"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 9561

COMMANDS = {
    "l": ("patada izquierda", lambda k: k.kick_left()),
    "r": ("patada derecha", lambda k: k.kick_right()),
    "a": ("acomodarse 5 cm a la izquierda", lambda k: k.adjust_left()),
    "d": ("acomodarse 5 cm a la derecha", lambda k: k.adjust_right()),
    "w": ("acomodarse 5 cm al frente", lambda k: k.adjust_front()),
}


def main():
    robot = Robot(IP, PORT)
    robot.init()
    kick = Kick(robot)

    try:
        while True:
            print()
            for key, (desc, _) in COMMANDS.items():
                print(f"  {key} = {desc}")
            print("  q = salir (el robot se sienta)")

            choice = input("> ").strip().lower()
            if choice == "q":
                break
            if choice not in COMMANDS:
                print("Opción no válida")
                continue

            desc, action = COMMANDS[choice]
            print(f"Ejecutando: {desc}...")
            action(kick)
            print("Listo")
    finally:
        robot.stop_move()
        robot.rest()


if __name__ == "__main__":
    main()
