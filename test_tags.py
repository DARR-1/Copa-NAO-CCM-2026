# Prueba de TagDetector en el NAO.
# Uso: python test_tags.py [IP] [PUERTO]
# Muestra TODAS las detecciones crudas (id, margin, hamming) para calibrar filtros,
# y el resultado de detect_tag() (tag más cercano). "q" para salir.
import sys
import math
import cv2
import qi
from vision.tag_detector import TagDetector

IP = sys.argv[1] if len(sys.argv) > 1 else "172.27.64.1"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 9561


def draw_detection(img, d, color):
    corners = d.corners.astype(int)
    cv2.polylines(img, [corners.reshape(-1, 1, 2)], True, color, 2)
    cx, cy = d.center.astype(int)
    cv2.circle(img, (cx, cy), 4, (0, 0, 255), -1)

    lines = [f"id={d.tag_id} m={d.decision_margin:.1f} h={d.hamming}"]
    if d.pose_t is not None:
        lines.append(f"z={d.pose_t[2][0]:.2f}m")

    for i, text in enumerate(lines):
        cv2.putText(img, text, (corners[:, 0].min(), corners[:, 1].min() - 8 - 18 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main():
    app = qi.Application(["TagTest", "--qi-url=tcp://{}:{}".format(IP, PORT)])
    app.start()

    td = TagDetector(app.session)

    # Revisa unidades: ~1.06 = radianes, ~61 = grados
    print("getHorizontalFOV(0) raw =", td.camera.video.getHorizontalFOV(0))
    print("getVerticalFOV(0) raw   =", td.camera.video.getVerticalFOV(0))
    print("camera_params (fx, fy, cx, cy) =", td.camera_params)

    try:
        while True:
            frame = td.camera.get_frame()
            if frame is None:
                continue

            # Todas las detecciones, sin filtro
            raw = td.at_detector.detect(frame,
                                        estimate_tag_pose=True,
                                        camera_params=td.camera_params,
                                        tag_size=td.tag_size)

            # Tu método
            tag_id, theta, bearing = td.detect_tag(frame)

            shown = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            for d in raw:  # type: ignore
                color = (0, 255, 0) if d.tag_id == tag_id else (0, 200, 255)
                draw_detection(shown, d, color)

            if tag_id is not None:
                summary = (f"closest id={tag_id}  theta={math.degrees(theta):.1f}deg  "
                           f"bearing={math.degrees(bearing):.1f}deg")
            else:
                summary = "no tag"
            cv2.putText(shown, summary, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            print(f"{summary}  | raw: {len(raw)}", end="\r")  # type: ignore

            cv2.imshow("Tag Test", shown)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        td.camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
