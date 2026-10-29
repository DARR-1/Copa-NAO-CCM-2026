import qi
import time
import math

NAO_IP = "192.168.0.4"
NAO_PORT = 9559

# ============ VELOCIDAD ============
VELOCIDAD = 1.0      # fracción de MaxStepX (0.0 a 1.0). 1.0 = lo más rápido
MAX_STEP_X = 0.06    # largo máximo del paso en metros (defecto 0.04, límite 0.08)
FRECUENCIA = 1.0     # frecuencia de pasos (0.0 a 1.0). 1.0 = pasos más rápidos
ALTURA_PASO = 0.02   # qué tanto levanta el pie en metros (0.005 a 0.04)
T_ARRANQUE = 1.0     # segundos para acelerar poco a poco (evita caídas al inicio)

# ============ ÁNGULO (para caminar recto) ============
KP = 0.0                 # corrección automática: más alto = corrige más fuerte
CORRECCION_FIJA = 0.036   # giro constante extra (negativo = derecha, positivo = izquierda)
LIMITE_CORRECCION = 0.3  # máximo giro permitido para corregir (0.0 a 1.0)
# ======================================================

CONFIG = [
    ["Frequency", FRECUENCIA],
    ["MaxStepX", MAX_STEP_X],
    ["StepHeight", ALTURA_PASO],
]


def normalizar(angulo):
    """Mantiene el ángulo entre -pi y pi."""
    return math.atan2(math.sin(angulo), math.cos(angulo))


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

        # Guardar hacia dónde apunta el robot al empezar
        theta_inicial = motion.getRobotPosition(True)[2]

        print("Caminando. Presiona el sensor trasero para detenerlo.")
        inicio = time.time()

        while memory.getData("RearTactilTouched") != 1.0:
            # Aceleración gradual: de 30% a 100% de la velocidad
            t = time.time() - inicio
            x = VELOCIDAD * min(1.0, 0.3 + 0.7 * t / max(T_ARRANQUE, 0.01))

            # Corrección del ángulo
            error = normalizar(motion.getRobotPosition(True)[2] - theta_inicial)
            giro = CORRECCION_FIJA - KP * error
            giro = max(-LIMITE_CORRECCION, min(LIMITE_CORRECCION, giro))

            motion.moveToward(x, 0.0, giro, CONFIG)   # x, y, theta, configuración
            time.sleep(0.1)

        # Frenar poco a poco para no desequilibrarse
        motion.moveToward(0.3, 0.0, 0.0, CONFIG)
        time.sleep(0.4)
        motion.stopMove()
        print("Sensor trasero presionado, deteniendo.")

    finally:
        motion.stopMove()
        posture.goToPosture("Crouch", 0.5)
        motion.rest()


if __name__ == "__main__":
    main()