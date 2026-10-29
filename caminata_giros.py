import qi
import time
import math

NAO_IP = "192.168.0.4"
NAO_PORT = 9559

VELOCIDAD = 0.5
CORRECCION_GIRO = -0.05   # para que camine derecho
T_TRAMO = 5.0             # segundos caminando entre un giro y otro
ANGULO_GIRO = math.pi / 2 # 90 grados en radianes

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

        giro = -ANGULO_GIRO   # negativo = derecha, positivo = izquierda
        detener = False
        print("Caminando. Presiona el sensor trasero para detenerlo.")

        while not detener:
            # Caminar hacia adelante durante T_TRAMO segundos (no bloquea)
            motion.moveToward(VELOCIDAD, 0.0, CORRECCION_GIRO)
            inicio = time.time()
            while time.time() - inicio < T_TRAMO:
                if memory.getData("RearTactilTouched") == 1.0:
                    detener = True
                    break
                time.sleep(0.1)

            motion.stopMove()

            # Girar 90 grados (bloquea hasta terminar) y cambiar de lado
            if not detener:
                motion.moveTo(0.0, 0.0, giro)
                giro = -giro

        print("Sensor trasero presionado, deteniendo.")

    finally:
        motion.stopMove()
        posture.goToPosture("Crouch", 0.5)
        motion.rest()

if __name__ == "__main__":
    main()