import cv2
import socket
import time
import mediapipe as mp

# ================= CONFIG =================
IP_WEBCAM_URL = "http://192.168.1.101:8080/video"
ESP_IP = "192.168.1.100"
ESP_PORT = 1234

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SEND_INTERVAL = 0.05

# ================= INIT =================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5)

cap = cv2.VideoCapture(IP_WEBCAM_URL)

smooth_centerX = FRAME_WIDTH // 2
smooth_area = 0

ALPHA = 0.7
last_send = 0

# ================= MAIN LOOP =================
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Camera not available")
        continue

    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    results = pose.process(rgb)

    centerX, area = 0, 0

    if results.pose_landmarks:
        lm = results.pose_landmarks.landmark

        # Torso points (stable)
        idx = [11, 12, 23, 24]

        x_coords = [lm[i].x for i in idx]
        y_coords = [lm[i].y for i in idx]

        xmin, xmax = min(x_coords), max(x_coords)
        ymin, ymax = min(y_coords), max(y_coords)

        centerX = int(((xmin + xmax) / 2) * FRAME_WIDTH)

        w = (xmax - xmin) * FRAME_WIDTH
        h = (ymax - ymin) * FRAME_HEIGHT
        area = int(w * h)

        # Smooth (critical)
        smooth_centerX = int(ALPHA * smooth_centerX + (1 - ALPHA) * centerX)
        smooth_area = int(ALPHA * smooth_area + (1 - ALPHA) * area)

        # Draw
        cv2.rectangle(frame,
                      (int(xmin*FRAME_WIDTH), int(ymin*FRAME_HEIGHT)),
                      (int(xmax*FRAME_WIDTH), int(ymax*FRAME_HEIGHT)),
                      (0,255,0), 2)

    else:
        # decay instead of sudden zero
        smooth_area = int(ALPHA * smooth_area)

    # ================= UDP =================
    now = time.time()
    if now - last_send > SEND_INTERVAL:
        msg = f"{smooth_centerX},{smooth_area}"
        sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))
        last_send = now

    # ================= DISPLAY =================
    cv2.putText(frame, f"X:{smooth_centerX} A:{smooth_area}",
                (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

    cv2.imshow("Follower", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
sock.close()