# ---------------- Cámara (reemplaza a Camera del NAO) ----------------
import cv2
import numpy as np

# ---------------- Configuración ----------------
# 0 = cámara de la laptop (Windows). En WSL pon la URL del stream, ej: "http://172.25.16.1:8080/"
CAMERA_INDEX = 0

lower_color_m1 = (13, 99, 58)       # Lower bound of the color in HSV m1
upper_color_m1 = (215, 59, 31)      # Upper bound of the color in HSV m1

lower_color_m2 = (0, 124, 193)      # Lower bound of the color in HSV for m2
upper_color_m2 = (12, 255, 255)     # Upper bound of the color in HSV for m2

# Para aislar un rango mientras calibras: pon False para apagarlo
m1_activo = True
m2_activo = True

fov_rad = np.deg2rad(60)  # Field of view in radians
ball_diameter = 0.065     # Diameter of the ball in meters


# ---------------- Cámara (reemplaza a Camera del NAO) ----------------
def open_camera(index=CAMERA_INDEX):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"No se pudo abrir la cámara con índice {index}")
    return cap


def get_frame(cap):
    # OpenCV entrega los frames en BGR, igual que colorspace=13 en el NAO
    ok, frame = cap.read()
    if not ok:
        return None
    return frame


# ---------------- Calibración (barras deslizables) ----------------
def _nada(_):
    pass


def _clamp(v, maximo):
    return max(0, min(int(v), maximo))


def setup_calibration():
    """Crea una ventana de barras por cada rango (m1 y m2)."""
    for nombre, low, up in (("M1", lower_color_m1, upper_color_m1),
                            ("M2", lower_color_m2, upper_color_m2)):
        win = f"Calibrar {nombre}"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 450, 300)
        cv2.createTrackbar("Activo", win, 1, 1, _nada)
        cv2.createTrackbar("H min", win, _clamp(low[0], 179), 179, _nada)
        cv2.createTrackbar("H max", win, _clamp(up[0], 179), 179, _nada)
        cv2.createTrackbar("S min", win, _clamp(low[1], 255), 255, _nada)
        cv2.createTrackbar("S max", win, _clamp(up[1], 255), 255, _nada)
        cv2.createTrackbar("V min", win, _clamp(low[2], 255), 255, _nada)
        cv2.createTrackbar("V max", win, _clamp(up[2], 255), 255, _nada)


def update_ranges_from_trackbars():
    """Lee las barras y actualiza los rangos que usa detect_ball."""
    global lower_color_m1, upper_color_m1, lower_color_m2, upper_color_m2
    global m1_activo, m2_activo

    def leer(nombre):
        win = f"Calibrar {nombre}"
        g = lambda t: cv2.getTrackbarPos(t, win)
        return ((g("H min"), g("S min"), g("V min")),
                (g("H max"), g("S max"), g("V max")),
                bool(g("Activo")))

    lower_color_m1, upper_color_m1, m1_activo = leer("M1")
    lower_color_m2, upper_color_m2, m2_activo = leer("M2")


def print_ranges():
    print("\n# Copia esto a tu código:")
    print(f"lower_color_m1 = {lower_color_m1}")
    print(f"upper_color_m1 = {upper_color_m1}")
    print(f"lower_color_m2 = {lower_color_m2}")
    print(f"upper_color_m2 = {upper_color_m2}\n")


# ---------------- Detección ----------------
def detect_ball(frame):
    blur = cv2.GaussianBlur(frame, (9, 9), 0)
    hsv_frame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    m1 = cv2.inRange(hsv_frame, lower_color_m1, upper_color_m1)
    m2 = cv2.inRange(hsv_frame, lower_color_m2, upper_color_m2)

    if not m1_activo:
        m1[:] = 0
    if not m2_activo:
        m2[:] = 0

    mask = cv2.bitwise_or(m1, m2)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, mask

    c = max(contours, key=cv2.contourArea)
    (x, y), r = cv2.minEnclosingCircle(c)

    if r < 5:
        return None, mask

    return (int(x), int(y), int(r)), mask


def get_actual_detection(cap, show_video=False):
    frame = get_frame(cap)

    if frame is None:
        return None, None

    detection, mask = detect_ball(frame)
    if show_video:
        frame = frame.copy()

        if detection:
            x, y, r = detection
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

        cv2.imshow("Ball Detection", frame)
        cv2.imshow("Mask", mask)

    if detection is None:
        return None, mask

    # Coordenadas relativas al centro de la imagen
    x, y, r = detection
    x -= frame.shape[1] // 2
    y -= frame.shape[0] // 2

    return (x, y, r), mask


# Codigo no probado
"""
def get_ball_position(cap):
    frame = get_frame(cap)

    if frame is None:
        return None

    detection, _ = detect_ball(frame)

    if detection is None:
        return None

    x, y, r = detection

    h, w = frame.shape[:2]

    # Focal en pixeles a partir del FOV horizontal
    f = (w / 2.0) / np.tan(fov_rad / 2.0)
    cx, cy = w / 2.0, h / 2.0

    # Descartar si el circulo toca el borde (radio no confiable)
    if x - r <= 0 or y - r <= 0 or x + r >= w or y + r >= h:
        return None

    d = 2.0 * r
    Z = f * ball_diameter / d
    X = (x - cx) * Z / f
    Y = (y - cy) * Z / f

    return X, Y, Z
"""


def close(cap):
    cap.release()
    cv2.destroyAllWindows()


# ---------------- Prueba / Calibración ----------------
if __name__ == "__main__":
    cap = open_camera()
    setup_calibration()
    try:
        while True:
            update_ranges_from_trackbars()

            detection, mask = get_actual_detection(cap, show_video=True)
            if detection is not None:
                print("Detección (x, y, r) relativa al centro:", detection)

            # 'p' imprime los rangos actuales, 'q' sale
            k = cv2.waitKey(1) & 0xFF
            if k == ord("p"):
                print_ranges()
            elif k == ord("q"):
                break
    finally:
        close(cap)