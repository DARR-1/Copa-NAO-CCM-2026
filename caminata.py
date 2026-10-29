import qi
import time

NAO_IP = "192.168.0.4"
NAO_PORT = 9559

def main():
    app = qi.Application([], url=f"tcp://{NAO_IP}:{NAO_PORT}")
    app.start()
    session = app.session

    motion = session.service("ALMotion")
    posture = session.service("ALRobotPosture")
    memory = session.service("ALMemory")

    # Esperar el sensor frontal para empezar
    print("Esperando que presiones el sensor frontal de la cabeza...")
    while memory.getData("FrontTactilTouched") != 1.0:
        time.sleep(0.1)
    print("Sensor frontal presionado, comenzando movimiento.")

    try:
        motion.wakeUp()
        posture.goToPosture("StandInit", 0.5)

        # Empezar a caminar hacia adelante (no bloquea)
        motion.moveToward(0.5, 0.0, 0)   # x, y, theta
        print("Caminando. Presiona el sensor trasero para detenerlo.")

        # Seguir caminando hasta que se presione el sensor trasero
        while memory.getData("RearTactilTouched") != 1.0:
            time.sleep(0.1)

        motion.stopMove()
        print("Sensor trasero presionado, deteniendo.")

    finally:
        motion.stopMove()
        posture.goToPosture("Crouch", 0.5)
        motion.rest()

if __name__ == "__main__":
    main()