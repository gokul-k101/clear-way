import cv2
import numpy as np
import time

# =====================================
# CONFIGURATION
# =====================================
SIGNAL_DELAY = 5  # seconds per traffic
EMERGENCY_THRESHOLD = 10
EMERGENCY_LOCK_FRAMES = 60
MIN_CONTOUR_AREA = 4000

# =====================================
# TRAFFIC SYSTEM (3-WAY)
# =====================================
roads = ["TRAFFIC 1", "TRAFFIC 2", "TRAFFIC 3"]
normal_index = 0

STATE_NORMAL = 0
STATE_EMERGENCY = 1

state = STATE_NORMAL
prev_state = STATE_NORMAL

last_switch_time = time.time()

# =====================================
# EMERGENCY VARIABLES
# =====================================
emergency_frames = 0
emergency_lock = 0
emergency_road = 0  # fixed road for demo

# =====================================
# SIGNAL FUNCTIONS
# =====================================
def normal_cycle(idx):
    signals = {r: "RED" for r in roads}
    signals[roads[idx % 3]] = "GREEN"
    return signals

def emergency_cycle(idx):
    signals = {r: "RED" for r in roads}
    signals[roads[idx]] = "GREEN"
    return signals

# =====================================
# START WEBCAM
# =====================================
video = cv2.VideoCapture(0)
if not video.isOpened():
    raise SystemExit("❌ Webcam not accessible")

print("✅ Webcam started")

# =====================================
# MAIN LOOP
# =====================================
while True:
    ret, frame = video.read()
    if not ret:
        break

    frame = cv2.resize(frame, (900, 500))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # =====================================
    # RED + BLUE EMERGENCY DETECTION
    # =====================================
    red1 = cv2.inRange(hsv, (0,120,70), (10,255,255))
    red2 = cv2.inRange(hsv, (170,120,70), (180,255,255))
    blue = cv2.inRange(hsv, (100,150,70), (140,255,255))

    mask = red1 + red2 + blue
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected = False
    for cnt in contours:
        if cv2.contourArea(cnt) > MIN_CONTOUR_AREA:
            detected = True
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,255), 2)

    # =====================================
    # EMERGENCY STABILITY LOGIC
    # =====================================
    if detected:
        emergency_frames += 1
    else:
        emergency_frames = 0

    if emergency_frames > EMERGENCY_THRESHOLD:
        state = STATE_EMERGENCY
        emergency_lock = EMERGENCY_LOCK_FRAMES

    if emergency_lock > 0:
        state = STATE_EMERGENCY
        emergency_lock -= 1
    else:
        state = STATE_NORMAL

    # =====================================
    # FIXED TIMING LOGIC (KEY PART)
    # =====================================
    current_time = time.time()

    # 🔑 Reset timer when emergency ends
    if prev_state == STATE_EMERGENCY and state == STATE_NORMAL:
        last_switch_time = current_time

    if state == STATE_EMERGENCY:
        signals = emergency_cycle(emergency_road)
        status_text = "🚨 EMERGENCY VEHICLE PRIORITY 🚨"
        status_color = (0,255,0)
    else:
        signals = normal_cycle(normal_index)
        status_text = "NORMAL TRAFFIC OPERATION"
        status_color = (0,0,255)

        if current_time - last_switch_time >= SIGNAL_DELAY:
            normal_index += 1
            last_switch_time = current_time

    prev_state = state

    # =====================================
    # UI DISPLAY
    # =====================================
    cv2.rectangle(frame, (0,0), (900,70), (35,35,35), -1)
    cv2.putText(frame, status_text, (20,45),
                cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 3)

    y = 120
    for road, sig in signals.items():
        color = (0,255,0) if sig == "GREEN" else (0,0,255)
        cv2.circle(frame, (60, y-10), 12, color, -1)
        cv2.putText(frame, f"{road} : {sig}", (90, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        y += 60

    cv2.imshow("AI Emergency Traffic Control System", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
        break

# =====================================
# CLEANUP
# =====================================
video.release()
cv2.destroyAllWindows()
print("✅ Program ended cleanly")