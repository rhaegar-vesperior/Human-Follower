import cv2
import mediapipe as mp
import websocket
import numpy as np
import time

# ================= CONFIG =================
ESP_IP    = "10.25.228.176"
CAM_URL   = "http://10.25.228.208:8080/video"
WS_URL    = f"ws://{ESP_IP}:81/"

SMOOTHING   = 5
TIMEOUT     = 2.0       # seconds before declaring target lost
FPS_LIMIT   = 15
Kp          = 0.004
BASE_SPEED  = 80
RECT_WIDTH  = 120       # dead-zone width in px

# ---- Re-ID config ----
SIGNATURE_MATCH_THRESH = 0.12   # max normalised distance to accept a re-match
SEARCH_SPEED           = 55     # slow rotation speed while searching
SEARCH_TIMEOUT         = 6.0    # give up searching after this long (just retarget anyone)

# ================= WS =================
def connect_ws():
    while True:
        try:
            ws = websocket.WebSocket()
            ws.connect(WS_URL)
            print("[WS] Connected")
            return ws
        except:
            print("[WS] Retrying...")
            time.sleep(1)

ws = connect_ws()

# ================= CAMERA =================
def get_cam():
    cap = cv2.VideoCapture(CAM_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap

cap = get_cam()

# ================= MEDIAPIPE =================
mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils

# Use enable_segmentation=False for speed; model_complexity=0 is fastest
pose = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=0,
    enable_segmentation=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)

# ================= RE-ID HELPERS =================

def compute_signature(lm, w, h):
    """
    Build a lightweight appearance signature from pose landmarks.
    Returns a dict with normalised geometric features.
    All values are normalised to [0,1] so frame-size doesn't matter.
    """
    L_SHOULDER = mp_pose.PoseLandmark.LEFT_SHOULDER
    R_SHOULDER = mp_pose.PoseLandmark.RIGHT_SHOULDER
    L_HIP      = mp_pose.PoseLandmark.LEFT_HIP
    R_HIP      = mp_pose.PoseLandmark.RIGHT_HIP
    NOSE       = mp_pose.PoseLandmark.NOSE

    ls = lm[L_SHOULDER]
    rs = lm[R_SHOULDER]
    lh = lm[L_HIP]
    rh = lm[R_HIP]
    nose = lm[NOSE]

    # shoulder width as fraction of frame width
    shoulder_width = abs(ls.x - rs.x)

    # torso height as fraction of frame height
    torso_height = abs(((ls.y + rs.y) / 2) - ((lh.y + rh.y) / 2))

    # torso aspect ratio (width / height)
    torso_ratio = (shoulder_width / torso_height) if torso_height > 1e-4 else 0

    # vertical position of shoulder midpoint in frame
    shoulder_y = (ls.y + rs.y) / 2

    # head-to-shoulder distance (proxy for how upright / close the person is)
    head_dist = abs(nose.y - (ls.y + rs.y) / 2)

    return {
        "shoulder_width": shoulder_width,
        "torso_height":   torso_height,
        "torso_ratio":    torso_ratio,
        "shoulder_y":     shoulder_y,
        "head_dist":      head_dist,
    }


def signature_distance(sig_a, sig_b):
    """
    Euclidean distance between two signatures in feature space.
    Lower = more similar.
    """
    keys = ["shoulder_width", "torso_height", "torso_ratio", "head_dist"]
    diffs = [(sig_a[k] - sig_b[k]) ** 2 for k in keys]
    return np.sqrt(np.mean(diffs))


# ================= STATE =================
class TrackerState:
    IDLE    = "IDLE"       # no target ever acquired
    LOCKED  = "LOCKED"     # actively tracking a person
    SEARCH  = "SEARCH"     # target lost, rotating to find them

state = TrackerState.IDLE

target_signature  = None   # stored signature of locked person
center_buffer     = []     # smoothing buffer for cx
last_seen         = time.time()
search_start_time = None
prev_time         = 0

# ================= LOOP =================
print("[INFO] Starting tracker. Press ESC to quit.")

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("[CAM] Lost… reconnecting")
        cap.release()
        time.sleep(1)
        cap = get_cam()
        continue

    # FPS limit
    now = time.time()
    if now - prev_time < 1.0 / FPS_LIMIT:
        continue
    prev_time = now

    frame = cv2.resize(frame, (320, 240))
    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape
    cx_frame_center = w // 2

    left_bound  = cx_frame_center - RECT_WIDTH // 2
    right_bound = cx_frame_center + RECT_WIDTH // 2

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    try:
        results = pose.process(rgb)
    except Exception as e:
        print(f"[POSE] Error: {e}")
        continue

    left_speed  = 0
    right_speed = 0
    status_text = state  # for HUD
    detected_cx  = None
    detected_sig = None

    if results.pose_landmarks:
        lm = results.pose_landmarks.landmark

        # --- landmarks for shoulder centre ---
        l = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
        r = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]

        # only use if both shoulders visible and confident
        if l.visibility > 0.4 and r.visibility > 0.4:
            detected_cx  = int((l.x + r.x) / 2 * w)
            detected_sig = compute_signature(lm, w, h)

    # ================================================================== #
    #  STATE MACHINE                                                       #
    # ================================================================== #

    if state == TrackerState.IDLE:
        if detected_cx is not None:
            # Lock onto whoever we first see
            target_signature = detected_sig
            state     = TrackerState.LOCKED
            last_seen = time.time()
            center_buffer = [detected_cx]
            print("[TRACKER] Target acquired → LOCKED")

    elif state == TrackerState.LOCKED:
        if detected_cx is not None:
            # Check if this is our target or an intruder
            dist = signature_distance(detected_sig, target_signature)

            if dist <= SIGNATURE_MATCH_THRESH:
                # Same person — track them and REFRESH signature slowly
                # (exponential moving average so signature adapts to distance changes)
                alpha = 0.15
                for k in target_signature:
                    target_signature[k] = (1 - alpha) * target_signature[k] + alpha * detected_sig[k]

                center_buffer.append(detected_cx)
                if len(center_buffer) > SMOOTHING:
                    center_buffer.pop(0)
                cx_smooth = int(np.mean(center_buffer))

                # Motor control
                if left_bound <= cx_smooth <= right_bound:
                    left_speed  = BASE_SPEED
                    right_speed = BASE_SPEED
                else:
                    error = cx_smooth - cx_frame_center
                    turn  = int(Kp * error * 200)
                    left_speed  = max(0, min(255, BASE_SPEED - turn))
                    right_speed = max(0, min(255, BASE_SPEED + turn))

                last_seen   = time.time()
                status_text = f"LOCKED  match={dist:.3f}"

                # Draw
                cv2.rectangle(frame, (left_bound, 0), (right_bound, h), (0, 255, 0), 2)
                cv2.circle(frame, (cx_smooth, h // 2), 8, (0, 200, 255), -1)
                mp_draw.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            else:
                # Different person in frame — IGNORE, keep last command
                status_text = f"INTRUDER dist={dist:.3f} – ignoring"
                # Don't update last_seen → will time out if real target gone
                # Keep previous speeds (stop is safer though)
                left_speed  = 0
                right_speed = 0

        else:
            # Nobody detected this frame
            left_speed  = 0
            right_speed = 0

        # Check timeout → switch to SEARCH
        if time.time() - last_seen > TIMEOUT:
            state             = TrackerState.SEARCH
            search_start_time = time.time()
            center_buffer     = []
            print("[TRACKER] Target lost → SEARCH")

    elif state == TrackerState.SEARCH:
        search_elapsed = time.time() - search_start_time

        if detected_cx is not None:
            dist = signature_distance(detected_sig, target_signature)

            if dist <= SIGNATURE_MATCH_THRESH or search_elapsed > SEARCH_TIMEOUT:
                # Re-acquired original target  OR  gave up and retargeting anyone
                if search_elapsed > SEARCH_TIMEOUT:
                    print("[TRACKER] Search timeout — retargeting nearest person")
                    target_signature = detected_sig   # adopt new target
                else:
                    print(f"[TRACKER] Target re-acquired (dist={dist:.3f}) → LOCKED")

                state         = TrackerState.LOCKED
                last_seen     = time.time()
                center_buffer = [detected_cx]
                left_speed    = 0
                right_speed   = 0
            else:
                # Someone else in frame — keep searching (slow rotate)
                status_text = f"SEARCH – intruder dist={dist:.3f}"
                left_speed  = SEARCH_SPEED
                right_speed = 0   # rotate right to scan
        else:
            # Nothing detected — slow rotate to scan
            status_text = "SEARCH – scanning…"
            left_speed  = SEARCH_SPEED
            right_speed = 0

    
    #  HUD OVERLAY
    state_colors = {
        TrackerState.IDLE:   (180, 180, 180),
        TrackerState.LOCKED: (0, 255, 100),
        TrackerState.SEARCH: (0, 165, 255),
    }
    color = state_colors.get(state, (255, 255, 255))

    cv2.rectangle(frame, (0, 0), (w, 22), (0, 0, 0), -1)
    cv2.putText(frame, f"[{state}] {status_text if isinstance(status_text,str) else ''}",
                (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
    cv2.putText(frame, f"L:{left_speed}  R:{right_speed}",
                (4, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

    
    #  SEND TO ESP                                                         
    
    msg = f"{left_speed},{right_speed}"
    try:
        ws.send(msg)
    except Exception:
        ws = connect_ws()

    cv2.imshow("Human Tracker", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()