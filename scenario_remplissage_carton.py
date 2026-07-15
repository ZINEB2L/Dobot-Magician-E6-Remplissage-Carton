import cv2
import numpy as np
import math
import time
from DobotTCP import Dobot

# ============================================================
# CONFIG ROBOT
# ============================================================
ROBOT_IP    = "192.168.5.1"
ROBOT_PORT  = 29999
SPEED_FACTOR = 30  # %

# ============================================================
# CONFIG CAMÉRA
# ============================================================
CAMERA_INDEX         = 0
CAMERA_WARMUP_FRAMES = 10
CAMERA_PREVIEW       = True
CAPTURE_MODE         = "manual"   # "manual": SPACE=capture | "auto": capture auto
AUTO_CAPTURE_AFTER_S = 2.0
AUTO_START_CYCLE     = True       # True: démarrage automatique après capture + détection
DETECTION_DEBUG      = False
DETECT_CAP_COLOR     = "green"    # "green" | "blue" | "both"

# ============================================================
# ZONE ROI CAMÉRA (pixels)
# ============================================================
ROI_ENABLED = True
ROI_POLY_PX = np.array(
    [[608, 58],
     [617, 406],
     [63, 403],
     [65, 34]],
    dtype=np.int32,
)
ROI_DRAW_WITH_MOUSE = True

# ============================================================
# Z CONFIG
# ============================================================
Z_PICK            = 168     # Z de prise dans la box (plan plat)
Z_LIFT_AFTER_PICK = 75.0   # Z de remontée après aspiration
SUCTION_SETTLE_S  = 9    # Délai stabilisation pompe (s)

# ============================================================
# POSITIONS CLÉS DU ROBOT  (reprises de TIO.py)
# ============================================================
# Position caméra (= "haut de la box" dans TIO.py)
POSE_VISION = (229.56,-67.39,389.74,178.21,-8.08,-2.39)
# Orientation outil (reprise de la pose vision)
#TOOL_RX, TOOL_RY, TOOL_RZ = POSE_VISION[3], POSE_VISION[4], POSE_VISION[5]
TOOL_RX, TOOL_RY, TOOL_RZ = -180.0, 0.0, 0.0  # Orientation fixe de l'outil (face vers le bas)
# Position de sécurité intermédiaire
POSE_SECURITY = (301.79,-73.41,292.09,-180.0, 0.0, 0.0)

# Position de sécurité au-dessus du carton
#POSE_CARTON_SECURITY = (99, -291.01, 267.7, -180.0, 0.0, 0.0)
POSE_CARTON_SECURITY = (78.76,-351.04,254.5, -180.0, 0.0, 0.0)
# ============================================================
# HOMOGRAPHIE CAMÉRA → ROBOT (XY)
# 4 points de calibration : pixels caméra → coordonnées robot (mm)
# ============================================================
PTS_CAMERA = np.array(
    [[67, 123],
     [313, 126],
     [552, 128],
     [542, 359],
     [311,356],
     [77,350]],
    dtype=np.float32,
)
PTS_ROBOT = np.array(
    [[293.05,-177.31],
     [293.89,-82.83],
     [290.13,22.52],
     [385.0,23.70],
     [388.93,-78.41],
     [389.34,-174.98]],
    dtype=np.float32,
)
H, _ = cv2.findHomography(PTS_CAMERA, PTS_ROBOT)

# ============================================================
# 12 POSITIONS DANS LE CARTON  (X, Y, Z fixe=97, Rx, Ry, Rz)
# Coordonnées calibrées manuellement sur le robot (cahier 14/05/2026)
#
#  Disposition dans le carton (vue de dessus) :
#
#   [ 3][ 6][ 9][12]
#   [ 2][ 5][ 8][11]
#   [ 1][ 4][ 7][10]
#
# ============================================================

_Z  = 118        # Z fixe de dépose dans le carton (mm)
_RX = -180.0    # Orientation outil (fixe)
_RY = 0.0
_RZ = 0.0

CARTON_SLOTS = [
    ( 175.45, -308.68, _Z, _RX, _RY, _RZ),  # Slot  1
    ( 175.45, -353.53, _Z, _RX, _RY, _RZ),  # Slot  2
    ( 170.80,-403.6, _Z, _RX, _RY, _RZ),  # Slot  3
    ( 119.30, -314.55, _Z, _RX, _RY, _RZ),  # Slot  4
    ( 111.52, -361.49, _Z, _RX, _RY, _RZ),  # Slot  5
    ( 111.04, -408.94, _Z, _RX, _RY, _RZ),  # Slot  6
    (  56.06, -315.44, _Z, _RX, _RY, _RZ),  # Slot  7
    (  52.71, -363.65, _Z, _RX, _RY, _RZ),  # Slot  8
    (  45.15, -410.43, _Z, _RX, _RY, _RZ),  # Slot  9
    (  -5.75, -322.46, _Z, _RX, _RY, _RZ),  # Slot 10
    ( -10.15, -366.64, _Z, _RX, _RY, _RZ),  # Slot 11
    (  -7.06, -412.77, _Z, _RX, _RY, _RZ),  # Slot 12
]

MAX_BOUTEILLES = len(CARTON_SLOTS)   # = 12

# ============================================================
# OFFSET CAMÉRA → VENTOUSE
# ============================================================
OFFSET_X = 0
OFFSET_Y = 0


# ============================================================
# FONCTIONS UTILITAIRES
# ============================================================

def camera_to_robot_xy(u, v):
    """Convertit un pixel (u,v) caméra en coordonnées (X,Y) robot via homographie."""
    if H is None:
        raise RuntimeError("Homographie invalide. Vérifie PTS_CAMERA / PTS_ROBOT.")
    pt = np.array([[[u, v]]], dtype=np.float32)
    xy = cv2.perspectiveTransform(pt, H)[0][0]
    return float(xy[0]) + OFFSET_X, float(xy[1]) + OFFSET_Y


def is_inside_roi(u, v):
    if not ROI_ENABLED:
        return True
    return cv2.pointPolygonTest(ROI_POLY_PX, (float(u), float(v)), False) >= 0


def connect_robot():
    """Connexion et activation du robot Dobot."""
    robot = Dobot(ip=ROBOT_IP, port=ROBOT_PORT)
    robot.Connect()
    try:
        robot.EnableRobot()
    except Exception as e:
        msg = str(e)
        if "Control Mode Is Not Tcp" in msg and hasattr(robot, "RequestControl"):
            print("  [INFO] Demande RequestControl()...")
            try:
                robot.RequestControl()
                time.sleep(0.5)
            except Exception:
                pass
            robot.EnableRobot()
        else:
            raise
    if hasattr(robot, "SpeedFactor"):
        robot.SpeedFactor(SPEED_FACTOR)
    print("  -> Robot connecté ✓")
    return robot


def get_robot_xy(robot):
    """Récupère la position XY courante du robot."""
    try:
        pose = robot.GetPose()
        if isinstance(pose, (list, tuple)) and len(pose) >= 2:
            return float(pose[0]), float(pose[1])
        if isinstance(pose, dict) and "x" in pose and "y" in pose:
            return float(pose["x"]), float(pose["y"])
    except Exception:
        pass
    return float(POSE_VISION[0]), float(POSE_VISION[1])


def move_pose(robot, pose, label=""):
    """Déplace le robot vers une pose (X,Y,Z,Rx,Ry,Rz) en MovJ."""
    if label:
        print(f"  -> MovJ vers {label} : {pose[:3]}")
    robot.MovJ(f"pose={{ {pose[0]},{pose[1]},{pose[2]},{pose[3]},{pose[4]},{pose[5]} }}")


def move_security(robot, label=""):
    """Retourne à la position de sécurité (haut de la box)."""
    tag = f"[{label}] " if label else ""
    print(f"  {tag}-> Sécurité : {POSE_SECURITY[:3]}")
    move_pose(robot, POSE_SECURITY)


# ============================================================
# DÉTECTION DES BOUCHONS
# ============================================================

def detect_caps_centers(frame_bgr):
    """Détecte les centres des bouchons (vert ou bleu) dans l'image."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    mask_blue  = cv2.inRange(hsv, (85,  60,  60), (130, 255, 255))
    mask_green = cv2.inRange(hsv, (36,  25,  25), ( 86, 255, 255))

    if DETECT_CAP_COLOR == "green":
        mask = mask_green
    elif DETECT_CAP_COLOR == "blue":
        mask = mask_blue
    else:
        mask = cv2.bitwise_or(mask_blue, mask_green)

    k    = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centers = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (400 <= area <= 20000):
            continue
        per = cv2.arcLength(cnt, True)
        if per <= 1e-6:
            continue
        if (4.0 * math.pi * area / (per * per)) < 0.45:
            continue
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if radius < 10:
            continue
        centers.append((int(x), int(y)))

    # Fusion des détections proches
    merged = []
    for (u, v) in centers:
        placed = False
        for i, (mu, mv, n) in enumerate(merged):
            if (u - mu) ** 2 + (v - mv) ** 2 <= 18.0 ** 2:
                merged[i] = ((mu * n + u) / (n + 1), (mv * n + v) / (n + 1), n + 1)
                placed = True
                break
        if not placed:
            merged.append((float(u), float(v), 1))

    return [(int(mu), int(mv)) for (mu, mv, _) in merged], mask


def draw_priority_ids(frame_bgr, detections_sorted):
    """Dessine les IDs de priorité sur l'image."""
    vis = frame_bgr.copy()
    if ROI_ENABLED:
        cv2.polylines(vis, [ROI_POLY_PX], True, (0, 255, 255), 2)

    for idx, (u, v) in enumerate(detections_sorted):
        bid   = idx + 1
        color = (0, 255, 0) if bid == 1 else (0, 165, 255) if bid == 2 else (0, 0, 255)
        cv2.circle(vis, (u, v), 18, color, 2)

        label = f"ID{bid}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        tx, ty = u - tw // 2, v - 26
        cv2.rectangle(vis, (tx - 3, ty - th - 3), (tx + tw + 3, ty + 3), (0, 0, 0), -1)
        cv2.putText(vis, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        xr, yr = camera_to_robot_xy(u, v)
        cv2.putText(vis, f"({xr:.0f},{yr:.0f})", (u - 35, v + 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

    cv2.putText(vis,
        f"Bouteilles: {len(detections_sorted)} | Ordre: ID1 -> ID{len(detections_sorted)}",
        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(vis, "VERT=1er  ORANGE=2eme  ROUGE=autres",
        (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    return vis


def sort_by_robot_distance(robot, detections_uv):
    """Trie les bouteilles par distance croissante depuis la position robot actuelle."""
    xr, yr = get_robot_xy(robot)

    def dist2(uv):
        x, y = camera_to_robot_xy(uv[0], uv[1])
        return (x - xr) ** 2 + (y - yr) ** 2

    sorted_det = sorted(detections_uv, key=dist2)

    print("\n" + "=" * 58)
    print("        TABLEAU DE PRIORITÉ DES BOUTEILLES")
    print("=" * 58)
    print(f"  {'ID':>3}  {'Pixel (u,v)':^14}  {'Robot X':>8}  {'Robot Y':>8}  {'Dist':>8}")
    print("-" * 58)
    for idx, (u, v) in enumerate(sorted_det):
        x, y = camera_to_robot_xy(u, v)
        dist = math.sqrt(dist2((u, v)))
        marker = "  <-- PREMIER" if idx == 0 else ""
        print(f"  {idx+1:>3}  ({u:3d},{v:3d})         {x:>8.1f}  {y:>8.1f}  {dist:>7.1f}mm{marker}")
    print("=" * 58)
    print(f"  Robot en X={xr:.1f}, Y={yr:.1f}")
    print("=" * 58 + "\n")
    return sorted_det


def capture_and_detect_bottles():
    """Ouvre la caméra, affiche le preview et retourne les centres détectés."""
    global ROI_POLY_PX, ROI_ENABLED

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Caméra index {CAMERA_INDEX} non disponible.")

    for _ in range(CAMERA_WARMUP_FRAMES):
        cap.read()

    if CAMERA_PREVIEW:
        t0 = time.time()
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            centers, mask = detect_caps_centers(frame)
            centers = [uv for uv in centers if is_inside_roi(uv[0], uv[1])]

            if centers:
                fc_x, fc_y = frame.shape[1] // 2, frame.shape[0] // 2
                centers_preview = sorted(
                    centers,
                    key=lambda uv: (uv[0] - fc_x) ** 2 + (uv[1] - fc_y) ** 2,
                )
                dbg = draw_priority_ids(frame, centers_preview)
            else:
                dbg = frame.copy()
                if ROI_ENABLED:
                    cv2.polylines(dbg, [ROI_POLY_PX], True, (0, 255, 255), 2)
                cv2.putText(dbg, "Aucune bouteille detectee", (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(dbg,
                "[SPACE]=capturer  [ESC]=quitter  [R]=redessiner ROI  mode:" + CAPTURE_MODE,
                (10, dbg.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.imshow("camera_preview", dbg)
            cv2.imshow("mask_caps", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                cap.release()
                cv2.destroyAllWindows()
                return []
            if key in (ord("r"), ord("R")) and ROI_DRAW_WITH_MOUSE:
                poly = _draw_roi_mouse(frame, ROI_POLY_PX if ROI_ENABLED else None)
                if poly is not None:
                    ROI_POLY_PX = poly
                    ROI_ENABLED = True
                    print("Nouvelle ROI =", ROI_POLY_PX.tolist())
            if key == 32:
                cap.release()
                cv2.destroyAllWindows()
                return centers
            if CAPTURE_MODE == "auto" and (time.time() - t0) >= AUTO_CAPTURE_AFTER_S:
                cap.release()
                cv2.destroyAllWindows()
                return centers
    else:
        ok, frame = cap.read()
        cap.release()
        if not ok or frame is None:
            raise RuntimeError("Impossible de capturer une image.")
        centers, _ = detect_caps_centers(frame)
        return [uv for uv in centers if is_inside_roi(uv[0], uv[1])]


def _draw_roi_mouse(frame_bgr, initial_poly=None):
    """Permet de redessiner la ROI à la souris."""
    win = "roi_draw (LClick=ajouter  ENTER=sauver  ESC=annuler)"
    pts = []
    if initial_poly is not None and len(initial_poly) >= 3:
        pts = [(int(p[0]), int(p[1])) for p in np.array(initial_poly).reshape(-1, 2)]

    def on_mouse(event, x, y, _f, _p):
        if event == cv2.EVENT_LBUTTONDOWN:
            pts.append((int(x), int(y)))

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)
    while True:
        vis = frame_bgr.copy()
        for p in pts:
            cv2.circle(vis, p, 5, (0, 255, 255), -1)
        if len(pts) >= 2:
            cv2.polylines(vis, [np.array(pts, np.int32)], len(pts) >= 3, (0, 255, 255), 2)
        cv2.putText(vis, "LClick:add  BACK:undo  C:clear  ENTER:save  ESC:cancel",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow(win, vis)
        key = cv2.waitKey(20) & 0xFF
        if key == 27:
            cv2.destroyWindow(win)
            return None
        if key in (8, 127) and pts:
            pts.pop()
        if key in (ord("c"), ord("C")):
            pts.clear()
        if key in (13, 10) and len(pts) >= 3:
            cv2.destroyWindow(win)
            return np.array(pts, np.int32)


# ============================================================
# PICK AND PLACE — UNE BOUTEILLE
# ============================================================

def pick_and_place_one(robot, u, v, bottle_id, total):
    """
    Cycle complet pick-and-place pour UNE bouteille.
    La position dans le carton est choisie dans CARTON_SLOTS selon bottle_id.
    """
    # ── Sélection du slot carton pour cette bouteille ─────────────────────
    slot_index = bottle_id - 1
    if slot_index >= len(CARTON_SLOTS):
        print(f"  ⚠️  bottle_id={bottle_id} dépasse le nombre de slots ({len(CARTON_SLOTS)}). Arrêt.")
        return
    xp, yp, zp, rxp, ryp, rzp = CARTON_SLOTS[slot_index]

    # Conversion pixel → XY robot
    x, y   = camera_to_robot_xy(u, v)
    z_down = Z_PICK
    z_up   = Z_PICK + Z_LIFT_AFTER_PICK

    print(f"\n{'─'*60}")
    print(f"  Bouteille {bottle_id}/{total}  |  pixel=({u},{v})  |  robot=({x:.1f},{y:.1f})")
    print(f"  → Slot carton #{bottle_id}  :  ({xp:.2f}, {yp:.2f}, {zp})")
    print(f"{'─'*60}")

    # ── Étape 1 : Approche au-dessus ─────────────────────────────────────
    print(f"  Étape 1 : Approche (Z={z_up:.1f} mm)")
    robot.MovJ(f"pose={{ {x},{y},{z_up},{TOOL_RX},{TOOL_RY},{TOOL_RZ} }}")
    time.sleep(2)

    # ── Étape 2 : Descente sur la bouteille ──────────────────────────────
    print(f"  Étape 2 : Descente (Z={z_down} mm)")
    robot.MovL(f"pose={{ {x},{y},{z_down},{TOOL_RX},{TOOL_RY},{TOOL_RZ} }}")

    # ── Étape 3 : Pompe ON ────────────────────────────────────────────────
    print(f"  Étape 3 : Pompe ON")
    robot.SetSucker(1)
    time.sleep(SUCTION_SETTLE_S)
    time.sleep(5)  # Attente aspiration (identique à TIO.py)

    # ── Étape 4 : Remontée ────────────────────────────────────────────────
    print(f"  Étape 4 : Remontée (Z={z_up:.1f} mm)")
    robot.MovL(f"pose={{ {x},{y},{z_up},{TOOL_RX},{TOOL_RY},{TOOL_RZ} }}")
    time.sleep(2)

    # ── Étape 5 : Position de sécurité (haut de la box) ──────────────────
    print(f"  Étape 5 : Sécurité (haut de la box) → {POSE_SECURITY[:3]}")
    move_security(robot, f"{bottle_id}/{total}")
    time.sleep(2)

    # ── Étape 6 : Sécurité carton (haut du carton) ───────────────────────
    print(f"  Étape 6 : Sécurité carton → {POSE_CARTON_SECURITY[:3]}")
    robot.MovJ(
        f"pose={{ {POSE_CARTON_SECURITY[0]},{POSE_CARTON_SECURITY[1]},{POSE_CARTON_SECURITY[2]},"
        f"{POSE_CARTON_SECURITY[3]},{POSE_CARTON_SECURITY[4]},{POSE_CARTON_SECURITY[5]} }}"
    )
    time.sleep(2)

    # ── Étape 7 : Descente dans le slot carton ───────────────────────────
    print(f"  Étape 7 : Slot #{bottle_id} dans le carton → ({xp:.2f},{yp:.2f},{zp})")
    robot.MovJ(f"pose={{ {xp},{yp},{zp},{rxp},{ryp},{rzp} }}")

    # ── Étape 8 : Pompe OFF ───────────────────────────────────────────────
    print(f"  Étape 8 : Pompe OFF → bouteille posée ✓")
    robot.SetSucker(0)
    time.sleep(SUCTION_SETTLE_S)

    # ── Étape 9 : Retour sécurité carton ─────────────────────────────────
    print(f"  Étape 9 : Retour sécurité carton → {POSE_CARTON_SECURITY[:3]}")
    robot.MovJ(
        f"pose={{ {POSE_CARTON_SECURITY[0]},{POSE_CARTON_SECURITY[1]},{POSE_CARTON_SECURITY[2]},"
        f"{POSE_CARTON_SECURITY[3]},{POSE_CARTON_SECURITY[4]},{POSE_CARTON_SECURITY[5]} }}"
    )

    if bottle_id < total:
        print(f"  → Bouteille {bottle_id} terminée, passage à la {bottle_id + 1}\n")
    else:
        print(f"  → Dernière bouteille traitée. Cycle complet ✓\n")


# ============================================================
# SCÉNARIO PRINCIPAL
# ============================================================

def run_scenario():
    """
    Scénario TIO avec détection caméra.

    Étape 1 : HOME
    Étape 2 : MovJ → POSE_VISION (position caméra au-dessus de la box)
              ↳ Robot immobile → Caméra ouvre + capture les bouteilles
    Étape 3 : MovJ → POSE_SECURITY (position de sécurité) + confirmation
    Étapes 4→9 (répétées × nb bouteilles) : pick(caméra) → sécurité → place(fixe)
    Étape finale : Retour HOME
    """
    robot = connect_robot()

    # ── Étape 1 : HOME ───────────────────────────────────────────────────
    print("\n" + "=" * 55)
    print("  ÉTAPE 1 : Retour HOME")
    print("=" * 55)
    robot.Home()
    time.sleep(2)

    # ── Étape 2 : Position caméra + détection ────────────────────────────
    print("\n" + "=" * 55)
    print("  ÉTAPE 2 : Position CAMÉRA + Détection bouteilles")
    print(f"            {POSE_VISION[:3]}")
    print("=" * 55)
    move_pose(robot, POSE_VISION, "caméra (haut de la box)")

    # ── Attente que le robot soit BIEN arrêté avant d'ouvrir la caméra ──
    try:
        robot.Sync()           # Bloque jusqu'à fin du mouvement (si supporté)
    except Exception:
        pass
    time.sleep(2)              # Stabilisation mécanique (vibrations)
    print("  -> Position caméra atteinte ✓  (robot immobile)")

    print("  -> Ouverture caméra + capture...")
    detections = capture_and_detect_bottles()
    if not detections:
        print("  Aucune bouteille détectée (ou ESC). Arrêt.")
        robot.Home()
        return

    detections_sorted = sort_by_robot_distance(robot, detections)

    # Limite au nombre de slots disponibles dans le carton
    if len(detections_sorted) > MAX_BOUTEILLES:
        print(f"  ⚠️  {len(detections_sorted)} bouteilles détectées → limité à {MAX_BOUTEILLES} (capacité carton).")
        detections_sorted = detections_sorted[:MAX_BOUTEILLES]

    n = len(detections_sorted)

    # ── Étape 3 : Position de sécurité + confirmation ────────────────────
    print("\n" + "=" * 55)
    print("  ÉTAPE 3 : Position de SÉCURITÉ + Confirmation")
    print("=" * 55)
    move_security(robot, "ÉTAPE 3")
    time.sleep(2)

    print(f"\n  Bouteilles détectées : {n}")
    if AUTO_START_CYCLE:
        print("  Démarrage automatique du cycle après détection...")
    else:
        print("  Appuie sur ENTER pour démarrer, ESC pour annuler...\n")

    # Confirmation visuelle ou démarrage automatique
    cap_preview = cv2.VideoCapture(CAMERA_INDEX)
    if cap_preview.isOpened():
        for _ in range(5):
            cap_preview.read()
        ok, frame_final = cap_preview.read()
        cap_preview.release()
        if ok and frame_final is not None:
            vis = draw_priority_ids(frame_final, detections_sorted)
            if AUTO_START_CYCLE:
                cv2.putText(vis, "DEMARRAGE AUTOMATIQUE...",
                            (10, vis.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("Confirmation ordre pick", vis)
                cv2.waitKey(1000)
                cv2.destroyAllWindows()
            else:
                cv2.putText(vis, "ENTER=demarrer  ESC=annuler",
                            (10, vis.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("Confirmation ordre pick", vis)
                while True:
                    key = cv2.waitKey(50) & 0xFF
                    if key in (13, 10):
                        cv2.destroyAllWindows()
                        break
                    if key == 27:
                        cv2.destroyAllWindows()
                        print("  Cycle annulé par l'utilisateur.")
                        robot.Home()
                        return
    else:
        if not AUTO_START_CYCLE:
            input("  [Terminal] ENTER pour démarrer...")

    # ── Boucle pick-and-place ─────────────────────────────────────────────
    print("\n" + "=" * 55)
    print(f"  DEBUT CYCLE : {n} bouteille(s)")
    print("=" * 55)

    for i, (u, v) in enumerate(detections_sorted):
        pick_and_place_one(robot, u, v, bottle_id=i + 1, total=n)

    # ── Retour HOME ───────────────────────────────────────────────────────
    print("\n" + "=" * 55)
    print(f"  CYCLE TERMINÉ : {n} bouteille(s) placée(s) ✓")
    print("  Retour HOME...")
    print("=" * 55 + "\n")
    robot.Home()
    time.sleep(4)


# ============================================================
# POINT D'ENTRÉE
# ============================================================
if __name__ == "__main__":
    run_scenario()
