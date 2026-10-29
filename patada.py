import qi
import time

NAO_IP = "192.168.0.4"
NAO_PORT = 9559

# Articulaciones de la pierna derecha
PIERNA = ["RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch"]

# Los valores son CAMBIOS (en radianes) respecto a la posición de pie,
# en el orden: cadera lateral, cadera adelante/atrás, rodilla, tobillo.
# Cadera: positivo = pierna hacia atrás. Rodilla: positivo = más doblada.
# Tobillo: negativo = punta del pie hacia arriba.
BACKSWING = [0.0,  0.15, 0.90, -0.60]   # pierna hacia atrás, rodilla doblada
PATADA    = [0.0, -0.12, 0.66, -0.55]   # pierna adelante, con el pie en el aire

# Tiempos en segundos (más lento = más estable)
T_EQUILIBRIO = 3.0   # pasar el peso a la pierna izquierda
T_ASENTAR = 1.0      # pausa para que se estabilice sobre un pie
T_BACKSWING = 1.5    # preparar la pierna
T_PATADA = 1.0       # golpe (baja este valor poco a poco para más fuerza)
T_REGRESO = 1.5      # volver la pierna a su lugar


def mover_pierna(motion, reposo, cambios, tiempo):
    """Mueve la pierna derecha: posición de pie + cambios (espera a terminar)."""
    objetivo = [r + c for r, c in zip(reposo, cambios)]
    motion.angleInterpolation(
        PIERNA,
        [[a] for a in objetivo],
        [[tiempo] for _ in PIERNA],
        True,
    )


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
        # 1. Levantarse y guardar cómo está la pierna derecha de pie
        motion.wakeUp()
        posture.goToPosture("StandInit", 0.5)
        reposo = motion.getAngles(PIERNA, True)

        # 2. Pasar el peso a la pierna izquierda y dejar la derecha libre
        motion.wbEnable(True)
        motion.wbFootState("Fixed", "Legs")
        motion.wbEnableBalanceConstraint(True, "Legs")
        motion.wbGoToBalance("LLeg", T_EQUILIBRIO)
        motion.wbFootState("Fixed", "LLeg")
        motion.wbFootState("Free", "RLeg")
        motion.wbEnableBalanceConstraint(True, "LLeg")
        time.sleep(T_ASENTAR)

        # 3. Preparar y dar la patada
        mover_pierna(motion, reposo, BACKSWING, T_BACKSWING)
        mover_pierna(motion, reposo, PATADA, T_PATADA)
        time.sleep(0.3)

        # 4. Regresar la pierna y apoyar los dos pies
        mover_pierna(motion, reposo, [0.0, 0.0, 0.0, 0.0], T_REGRESO)
        motion.wbFootState("Fixed", "Legs")
        motion.wbEnableBalanceConstraint(True, "Legs")
        motion.wbGoToBalance("Legs", T_EQUILIBRIO)
        print("Listo.")

    finally:
        try:
            motion.wbEnable(False)   # siempre se debe apagar el balanceador
        except RuntimeError:
            pass
        motion.stopMove()
        posture.goToPosture("Crouch", 0.5)
        motion.rest()


if __name__ == "__main__":
    main()