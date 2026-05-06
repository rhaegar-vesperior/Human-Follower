import cv2
import mediapipe as mp
import websocket
import numpy as np
import time

# ================= CONFIG =================
ESP_IP = "10.25.228.176" # add you esp ID here
CAM_URL = "http://10.25.228.208:8080/video" # add your camera URL here (from IP Webcam app)

WS_URL = f"ws://{ESP_IP}:81/"

SMOOTHING = 5
TIMEOUT = 1.0
FPS_LIMIT = 15

Kp = 0.004
BASE_SPEED = 80
RECT_WIDTH = 120   # 🔴 dead-zone width (tune this)

# ================= WS =================
def connect_ws():
    while True:
        try:
            ws = websocket.WebSocket()
            ws.connect(WS_URL)
            print("WS Connected")
            return ws
        except:
            print("Retrying WS...")
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
pose = mp_pose.Pose()

center_buffer = []
last_seen = time.time()
prev_time = 0

# ================= LOOP =================
while True:
    ret, frame = cap.read()

    if not ret or frame is None:
        print("Camera lost... reconnecting")
        cap.release()
        time.sleep(1)
        cap = get_cam()
        continue

    # FPS limit
    now = time.time()
    if now - prev_time < 1/FPS_LIMIT:
        continue
    prev_time = now

    frame = cv2.resize(frame, (320, 240))
    frame = cv2.flip(frame, 1)

    h, w, _ = frame.shape
    center = w // 2

    # 🔴 rectangle bounds
    left_bound  = center - RECT_WIDTH//2
    right_bound = center + RECT_WIDTH//2

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    try:
        results = pose.process(rgb)
    except:
        continue

    left_speed = 0
    right_speed = 0

    if results.pose_landmarks:
        lm = results.pose_landmarks.landmark

        l = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
        r = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]

        cx = int((l.x + r.x)/2 * w)

        # smoothing
        center_buffer.append(cx)
        if len(center_buffer) > SMOOTHING:
            center_buffer.pop(0)

        cx = int(np.mean(center_buffer))

        # 🔴 NEW CONTROL LOGIC
        if left_bound <= cx <= right_bound:
            # inside rectangle → straight
            left_speed = BASE_SPEED
            right_speed = BASE_SPEED
        else:
            error = cx - center
            turn = int(Kp * error * 200)

            left_speed = BASE_SPEED - turn
            right_speed = BASE_SPEED + turn

        # clamp
        left_speed = max(0, min(255, left_speed))
        right_speed = max(0, min(255, right_speed))

        last_seen = time.time()

        # 🔴 DRAW RECTANGLE + TARGET
        cv2.rectangle(frame,
                      (left_bound, 0),
                      (right_bound, h),
                      (0,255,0), 2)

        cv2.circle(frame, (cx, h//2), 8, (255,0,0), -1)

    # no human
    if time.time() - last_seen > TIMEOUT:
        left_speed = 0
        right_speed = 0

    msg = f"{left_speed},{right_speed}"

    try:
        ws.send(msg)
    except:
        ws = connect_ws()

    cv2.imshow("Tracking", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
