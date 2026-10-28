LEG_STEP_CONFIG = [["StepHeight", 0.02]]  # pasos cortos para acomodarse frente a la pelota
SHIFT = 0.35  # inclinación de cadera/tobillo para pasar el peso a la pierna de apoyo (subir si se cae hacia la pierna que patea)
SPEED_MARGIN = 0.9  # fracción de la velocidad máxima de cada motor que se permite usar

# Pose de pie; el pie queda plano cuando HipPitch + KneePitch + AnklePitch = 0
BASE = {
    "HeadYaw": 0.0, "HeadPitch": 0.0,
    "LShoulderPitch": 1.4, "LShoulderRoll": 0.3, "LElbowYaw": 0.0, "LElbowRoll": -0.5, "LWristYaw": 0.0,
    "RShoulderPitch": 1.4, "RShoulderRoll": -0.3, "RElbowYaw": 0.0, "RElbowRoll": 0.5, "RWristYaw": 0.0,
    "LHipYawPitch": 0.0, "LHipRoll": 0.0, "LHipPitch": -0.30, "LKneePitch": 0.60, "LAnklePitch": -0.30, "LAnkleRoll": 0.0,
    "RHipRoll": 0.0, "RHipPitch": -0.30, "RKneePitch": 0.60, "RAnklePitch": -0.30, "RAnkleRoll": 0.0,
    "LHand": 0.0, "RHand": 0.0,
}

# Peso sobre la pierna izquierda y brazos abiertos para balance
LEFT_SUPPORT = {
    "LHipRoll": -SHIFT, "LAnkleRoll": SHIFT,
    "LShoulderRoll": 0.5, "RShoulderRoll": -0.5,
}

# Pierna derecha en el aire, un poco abierta y con el pie plano
RIGHT_LEG_UP = {**LEFT_SUPPORT, "RHipRoll": -0.30, "RAnkleRoll": 0.15}

# (duración en s, ángulos que cambian respecto a BASE); patada con la pierna derecha
SHIFT_RIGHT = [
    (1.5, {}),  # postura inicial
    (1.2, {**LEFT_SUPPORT, "RHipRoll": -SHIFT, "RAnkleRoll": SHIFT}),  # cargar peso
]
SWING_RIGHT = [
    (1.0, {**RIGHT_LEG_UP, "RHipPitch": -0.80, "RKneePitch": 1.80, "RAnklePitch": -1.00}),  # levantar pierna
    (0.8, {**RIGHT_LEG_UP, "RHipPitch": 0.10, "RKneePitch": 2.00, "RAnklePitch": -0.90}),  # backswing
    (0.05, {**RIGHT_LEG_UP, "RHipPitch": -0.90, "RKneePitch": 1.90, "RAnklePitch": -1.00}),  # muslo adelante, pie alto
    (0.05, {**RIGHT_LEG_UP, "RHipPitch": -1.00, "RKneePitch": 1.00, "RAnklePitch": 0.00}),  # golpe con pie plano
    (0.05, {**RIGHT_LEG_UP, "RHipPitch": -1.10, "RKneePitch": 0.50, "RAnklePitch": 0.30}),  # seguimiento
    (0.8, {**RIGHT_LEG_UP, "RHipPitch": -0.80, "RKneePitch": 1.80, "RAnklePitch": -1.00}),  # recoger pierna
    (0.8, {**LEFT_SUPPORT, "RHipRoll": -SHIFT, "RAnkleRoll": SHIFT}),  # bajar el pie al piso
]
RECOVER = [(1.5, {})]  # volver a la postura base


def mirror_joint(name, angle):
    # Cambia L/R y voltea el signo de Roll/Yaw; HipYawPitch es una sola articulación compartida
    if name == "LHipYawPitch":
        return name, angle
    if name[0] in "LR":
        name = ("R" if name[0] == "L" else "L") + name[1:]
    if "Roll" in name or "Yaw" in name:
        angle = -angle
    return name, angle


def mirror(keyframes):
    return [(duration, dict(mirror_joint(name, angle) for name, angle in changes.items()))
            for duration, changes in keyframes]


SHIFT_LEFT = mirror(SHIFT_RIGHT)
SWING_LEFT = mirror(SWING_RIGHT)


def min_duration(starts, target, max_speeds):
    # Tiempo que necesita la articulación más lenta, midiendo desde cada posición de partida
    return max(abs(t - s) / speed for start in starts for s, t, speed in zip(start, target, max_speeds))


class Kick:
    def __init__(self, robot):
        self.robot = robot
        self.motion = self.robot.motion_service
        self.names = list(BASE)
        self.max_speeds = [self.motion.getLimits(name)[0][2] * SPEED_MARGIN for name in self.names]

    def _play(self, keyframes):
        # Una interpolación por keyframe: arranca y frena en cada pose, como el smoothstep de Webots
        for duration, changes in keyframes:
            target = [{**BASE, **changes}[name] for name in self.names]
            # Se mide desde el ángulo real y el comandado porque el balancer o la protección de brazos los separan del keyframe
            starts = [self.motion.getAngles(self.names, True), self.motion.getAngles(self.names, False)]
            duration = max(duration, min_duration(starts, target, self.max_speeds))  # el NAO real rechaza movimientos más rápidos que sus motores
            self.motion.angleInterpolation(self.names, target, [duration] * len(self.names), True)

    def _kick(self, shift, swing, support_leg, kick_leg):
        self.robot.stop_move()  # la patada no puede mezclarse con la caminata

        # El whole body balancer corrige la pierna de apoyo para mantener el centro de masa sobre el pie
        self.motion.wbEnable(True)
        try:
            self.motion.wbFootState("Fixed", "Legs")
            self.motion.wbEnableBalanceConstraint(True, "Legs")
            self._play(shift)

            self.motion.wbGoToBalance(support_leg, 1.0)  # termina de pasar el centro de masa al pie de apoyo
            self.motion.wbFootState("Fixed", support_leg)
            self.motion.wbFootState("Free", kick_leg)
            self.motion.wbEnableBalanceConstraint(True, support_leg)
            self._play(swing)

            self.motion.wbFootState("Fixed", "Legs")  # ambos pies otra vez en el piso
            self.motion.wbEnableBalanceConstraint(True, "Legs")
            self._play(RECOVER)
        finally:
            self.motion.wbEnable(False)

    def kick_left(self):
        self._kick(SHIFT_LEFT, SWING_LEFT, "RLeg", "LLeg")

    def kick_right(self):
        self._kick(SHIFT_RIGHT, SWING_RIGHT, "LLeg", "RLeg")

    def adjust_left(self):
        self.motion.moveTo(0.0, 0.05, 0.0, LEG_STEP_CONFIG)

    def adjust_right(self):
        self.motion.moveTo(0.0, -0.05, 0.0, LEG_STEP_CONFIG)

    def adjust_front(self):
        self.motion.moveTo(0.05, 0.0, 0.0, LEG_STEP_CONFIG)
