# Prueba del striker en el NAO.
# Uso: python test_striker.py [IP] [PUERTO]
#   c = calibrar: el robot se queda quieto (cámara inferior, cabeza abajo) y
#       muestra x, y, r de la pelota. Pon la pelota donde quieres patear y anota
#       la y para STOP_Y.
#   s = correr el striker completo (buscar, acercarse, alinear y patear):
#       toca la cabeza al medio para arrancar. ¡Ten a alguien cuidando al robot!
# Toca la cabeza atrás (o Ctrl+C) para detener en cualquier momento.
import sys
from robot import Robot
from behaviors.striker import Striker, State, HEAD_PITCH

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.151"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 9561


def calibrate(robot):
    detector = robot.ball_detector
    detector.camera.set_camera(1)
    robot.look(HEAD_PITCH, 0)

    print(f"Calibrando con HeadPitch = {HEAD_PITCH}... (p en la ventana = imprimir rangos de color)")
    while not robot.get_rear_head_touch():
        detection, _ = detector.get_actual_detection(show_video=True)
        if detection is None:
            print("sin pelota")
        else:
            x, y, r = detection
            print(f"x = {x:4d}   y = {y:4d}   r = {r:3d}")


def run_striker(robot):
    striker = Striker(robot)

    print("Toca la cabeza al medio para arrancar")
    robot.wait_for_start()

    last_state = None
    last_adjusts = 0
    while not striker.is_done() and not robot.get_rear_head_touch():
        striker.run()

        if striker.state != last_state:
            print(f"--> {striker.state.name}")
            if striker.state == State.ALIGN:
                print(f"    target = {striker.target}   pie = {striker.foot}")
                last_adjusts = 0
            elif striker.state == State.SHOOT:
                print(f"    pateando con pie {striker.foot} tras {striker.adjusts} ajustes")
            last_state = striker.state

        if striker.state == State.APPROACH and striker.lost == 0:
            x, y, _ = striker.last_detection
            print(f"    x = {x:4d}   y = {y:4d}   stop_hits = {striker.stop_hits}")

        if striker.state == State.ALIGN and striker.adjusts != last_adjusts:
            print(f"    ajuste #{striker.adjusts}")
            last_adjusts = striker.adjusts


def main():
    robot = Robot(IP, PORT)
    robot.init()

    try:
        choice = input("c = calibrar, s = correr striker > ").strip().lower()
        if choice == "c":
            calibrate(robot)
        elif choice == "s":
            run_striker(robot)
        else:
            print("Opción no válida")
    finally:
        robot.stop_move()
        robot.ball_detector.close()
        robot.rest()


if __name__ == "__main__":
    main()
