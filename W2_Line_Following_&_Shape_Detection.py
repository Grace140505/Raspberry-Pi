# =========================================================
# Combined Line Follow (Lower ROI) + Symbol/Shape Detect (Upper ROI)
# =========================================================

from picamera2 import Picamera2
import cv2 as cv
import numpy as np
import time
import RPi.GPIO as GPIO
from collections import deque, Counter
from pathlib import Path

# =========================================================
# CAMERA / ROI SETTINGS
# =========================================================
FRAME_W = 640
FRAME_H = 480

UPPER_Y1 = 0
UPPER_Y2 = 280   

LOWER_Y1 = 240
LOWER_Y2 = 480
LOWER_X1 = 100
LOWER_X2 = 540

# =========================================================
# DETECTION / BEHAVIOUR TUNING
# =========================================================
SHAPE_MIN_AREA = 800
SLOW_RATIO = 0.01    
STOP_RATIO = 0.02    
ORB_EVERY_N_FRAMES = 1

LABEL_HISTORY_LEN = 3
MIN_CONFIRM_COUNT = 1        
MIN_STOP_CONFIRM_COUNT = 1   
HOLD_TIME_SEC = 1.5          
COOLDOWN_AFTER_HOLD = 1.0    

# =========================================================
# ORB REFERENCE IMAGES
# =========================================================
SAMPLE_DICT = {
    0: ("hazardSymbol.jpg", 45),
    1: ("fingerPrint.jpg", 45),
    2: ("recycleSymbol.jpg", 35),
    3: ("qrCode.jpg", 40),
    4: ("pushButton.jpg", 60)
}

# =========================================================
# COLOR RANGES
# =========================================================
COLOR_RANGES = {
    "Green":  {"space": "HSV", "lower": np.array([45, 85, 50]),   "upper": np.array([80, 255, 255])},
    "Yellow": {"space": "HSV", "lower": np.array([25, 150, 50]),  "upper": np.array([35, 255, 255])},
    "Blue":   {"space": "LAB", "lower": np.array([0, 130, 0]),    "upper": np.array([120, 185, 120])},
    "Teal":   {"space": "LAB", "lower": np.array([5, 110, 65]),   "upper": np.array([90, 145, 120])},
    "Purple": {"space": "LAB", "lower": np.array([15, 135, 60]),  "upper": np.array([255, 175, 140])},
    "Red":    {"space": "LAB", "lower": np.array([20, 160, 130]), "upper": np.array([150, 255, 180])},
    "Orange": {"space": "LAB", "lower": np.array([20, 120, 160]), "upper": np.array([255, 165, 200])}
}

# =========================================================
# GPIO PINS
# =========================================================
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 18
ENB = 19

# =========================================================
# GLOBAL VARIABLES & PID
# =========================================================
pwm_left = None
pwm_right = None

NORMAL_LEFT_BASE_SPEED = 35
NORMAL_RIGHT_BASE_SPEED = 35
SLOW_LEFT_BASE_SPEED = 20
SLOW_RIGHT_BASE_SPEED = 20

prev_time = time.monotonic()

PID_state = {
    'last_error': 0,
    'integral': 0,
    'last_time': time.monotonic()
}

KP = 0.42
KI = 0.01
KD = 0.05

X_CENTRE_REFERENCE = (LOWER_X2 - LOWER_X1) // 2
Y_CENTRE_REFERENCE = (LOWER_Y2 - LOWER_Y1) // 2

current_x = X_CENTRE_REFERENCE
current_y = Y_CENTRE_REFERENCE
output_x = 0
contour_area = 0
frame_counter = 0

robot_state = "FOLLOW"   
hold_until = 0.0
cooldown_until = 0.0
locked_label = "None"

label_history = deque(maxlen=LABEL_HISTORY_LEN)
ratio_history = deque(maxlen=LABEL_HISTORY_LEN)

# =========================================================
# SHAPE FUNCTIONS
# =========================================================
def is_diamond_vs_trapezium(approx):
    pts = [tuple(pt[0]) for pt in approx]
    def dist(p1, p2):
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
    sides = [dist(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    max_side, min_side = max(sides), min(sides)
    if min_side == 0: return False
    diag1, diag2 = dist(pts[0], pts[2]), dist(pts[1], pts[3])
    if min(diag1, diag2) == 0: return False
    return (max_side / min_side) < 1.2 and (max(diag1, diag2) / min(diag1, diag2)) < 1.3

def detect_shape(contour):
    area = cv.contourArea(contour)
    perimeter = cv.arcLength(contour, True)
    if perimeter == 0: return "Unknown", None

    approx = cv.approxPolyDP(contour, 0.03 * perimeter, True)
    vertices = len(approx)

    hull = cv.convexHull(contour)
    hull_area = cv.contourArea(hull)
    if hull_area == 0: return "Unknown", None

    solidity = float(area) / hull_area
    is_convex = cv.isContourConvex(approx)
    circularity = 4 * np.pi * area / (perimeter * perimeter)
    x, y, w, h = cv.boundingRect(contour)
    aspect_ratio = w / float(h) if h != 0 else 0

    if vertices == 4 and is_convex and 0.9 <= aspect_ratio <= 1.35:
        return ("Diamond", None) if is_diamond_vs_trapezium(approx) else ("Trapezium", None)
    elif (8 <= vertices <= 9) and is_convex and (0.93 <= solidity <= 1): return "Octagon", None
    elif 10 <= vertices <= 14 and not is_convex and 0.55 < solidity < 0.88: return "Plus", None
    elif (8 <= vertices <= 10) and (0.53 <= circularity <= 0.84) and (0.85 <= aspect_ratio <= 1.4) and (0.7 < solidity < 0.85): return "3/4 Circle", None
    elif (6 <= vertices <= 7) and is_convex and 0.70 <= circularity <= 0.82: return "Semicircle", None
    elif vertices >= 10 and not is_convex and solidity < 0.60: return "Star", None
    elif 7 <= vertices <= 10 and not is_convex and (0.45 <= solidity <= 0.75):
        M = cv.moments(contour)
        if M["m00"] == 0: return "Arrow", "Unknown"
        cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        farthest = max(contour, key=lambda p: (p[0][0] - cx)**2 + (p[0][1] - cy)**2)
        dx, dy = farthest[0][0] - cx, farthest[0][1] - cy
        if abs(dx) > abs(dy): direction = "Left" if dx > 0 else "Right"
        else: direction = "Up" if dy > 0 else "Down"
        return "Arrow", direction

    return "Unknown", None

# =========================================================
# MOTOR / GPIO
# =========================================================
def func_init():
    global pwm_left, pwm_right
    GPIO.setmode(GPIO.BCM)
    for pin in [IN1, IN2, IN3, IN4, ENA, ENB]:
        GPIO.setup(pin, GPIO.OUT)

    pwm_left = GPIO.PWM(ENA, 500)
    pwm_right = GPIO.PWM(ENB, 500)
    pwm_left.start(0)
    pwm_right.start(0)

def moveForward(left_speed, right_speed):
    if left_speed < 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    left_speed = max(0, min(100, abs(left_speed)))

    if right_speed < 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    right_speed = max(0, min(100, abs(right_speed)))

    pwm_left.ChangeDutyCycle(left_speed)
    pwm_right.ChangeDutyCycle(right_speed)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

def brake_stop():
    stop()

# =========================================================
# PID / LINE FOLLOW
# =========================================================
def PID_control():
    global PID_state, output_x, current_x

    error = X_CENTRE_REFERENCE - current_x
    current_time = time.monotonic()
    dt = current_time - PID_state['last_time']
    PID_state['last_time'] = current_time

    P_x = KP * error
    PID_state['integral'] += error * dt
    I_x = KI * PID_state['integral']
    D_x = KD * (error - PID_state['last_error']) / dt if dt > 0 else 0

    PID_state['last_error'] = error
    output_x = P_x + I_x + D_x

def line_follow_on_lower_roi(full_bgr, full_gray, base_left, base_right):
    global current_x, current_y, contour_area

    roi_bgr = full_bgr[LOWER_Y1:LOWER_Y2, LOWER_X1:LOWER_X2].copy()
    roi_gray = full_gray[LOWER_Y1:LOWER_Y2, LOWER_X1:LOWER_X2].copy()

    blur_frame = cv.GaussianBlur(roi_gray, (3, 3), 0)
    # TWEAKED: Strict threshold of 65 to ignore shapes on the ground
    _, threshold_image = cv.threshold(blur_frame, 75, 255, cv.THRESH_BINARY_INV)

    contours, _ = cv.findContours(threshold_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv.contourArea)
        contour_area = cv.contourArea(largest_contour)

        if contour_area > 5500:
            M = cv.moments(largest_contour)
            if M['m00'] != 0:
                current_x = int(M['m10'] / M['m00'])
                current_y = int(M['m01'] / M['m00'])

            cv.drawContours(roi_bgr, [largest_contour], -1, (0, 255, 0), 2)
            PID_control()
            moveForward(base_left - output_x, base_right + output_x)
        else:
            moveForward(base_left - output_x, base_right + output_x)
    else:
        if output_x > 0:
            moveForward(-60, 70)
        else:
            moveForward(70, -60)

    cv.circle(roi_bgr, (X_CENTRE_REFERENCE, Y_CENTRE_REFERENCE), 5, (0, 0, 255), -1)
    cv.circle(roi_bgr, (current_x, current_y), 5, (0, 255, 255), -1)
    cv.putText(roi_bgr, f"Area: {contour_area:.0f}", (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
    cv.putText(roi_bgr, f"Output: {output_x:.0f}", (10, 42), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)

    return roi_bgr, threshold_image

# =========================================================
# ORB SETUP + MATCH
# =========================================================
def prepare_reference_data():
    script_dir = Path(__file__).resolve().parent
    refs_dir = script_dir / "orb_refs"
    orb = cv.ORB_create(nfeatures=1000)
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=False)

    reference_data = []
    print("\n[ORB] Loading reference images from:", refs_dir)

    for key, (img_file, threshold) in SAMPLE_DICT.items():
        img_path = refs_dir / img_file
        img = cv.imread(str(img_path), cv.IMREAD_GRAYSCALE)
        if img is None: continue

        kp, des = orb.detectAndCompute(img, None)
        if des is None or len(des) == 0: continue

        reference_data.append({
            "id": key, "name": img_path.stem, "des": des, "threshold": threshold
        })
        print(f"[ORB] OK -> {img_path.name} | features={len(des)} | threshold={threshold}")

    return orb, bf, reference_data

def orb_symbol_match(gray_crop, orb, bf, reference_data):
    if gray_crop is None or gray_crop.size == 0: return None
    kp_scene, des_scene = orb.detectAndCompute(gray_crop, None)
    if des_scene is None: return None

    best_name, best_score = None, -1
    for ref in reference_data:
        if ref["des"] is None: continue
        matches = bf.knnMatch(ref["des"], des_scene, k=2)
        good = sum(1 for m, n in matches if len(matches) == 2 and m.distance < 0.75 * n.distance)
        if good > best_score and good >= ref["threshold"]:
            best_score = good
            best_name = ref["name"]

    return best_name

# =========================================================
# UPPER ROI DETECTION
# =========================================================
def detect_upper_object_and_label(upper_rgb, upper_bgr, upper_gray, orb, bf, reference_data, frame_counter):
    blurred = cv.GaussianBlur(upper_rgb, (5, 5), 0)
    hsv = cv.cvtColor(blurred, cv.COLOR_RGB2HSV)
    lab = cv.cvtColor(blurred, cv.COLOR_RGB2LAB)

    combined_mask = np.zeros((upper_rgb.shape[0], upper_rgb.shape[1]), dtype=np.uint8)
    for _, params in COLOR_RANGES.items():
        m = cv.inRange(hsv, params["lower"], params["upper"]) if params["space"] == "HSV" else cv.inRange(lab, params["lower"], params["upper"])
        combined_mask = cv.bitwise_or(combined_mask, m)

    kernel = np.ones((5, 5), np.uint8)
    combined_mask = cv.morphologyEx(cv.morphologyEx(combined_mask, cv.MORPH_OPEN, kernel), cv.MORPH_CLOSE, kernel)

    contours, _ = cv.findContours(combined_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours: return False, "None", 0.0

    c = max(contours, key=cv.contourArea)
    area = cv.contourArea(c)
    if area < SHAPE_MIN_AREA: return False, "None", 0.0

    upper_area = upper_rgb.shape[0] * upper_rgb.shape[1]
    ratio = area / float(upper_area)

    x, y, w, h = cv.boundingRect(c)
    cv.drawContours(upper_bgr, [c], -1, (0, 255, 0), 2)
    cv.rectangle(upper_bgr, (x, y), (x + w, y + h), (255, 0, 0), 2)

    shape, direction = detect_shape(c)
    label = shape + (f" ({direction})" if direction else "")

    if len(reference_data) > 0 and (frame_counter % ORB_EVERY_N_FRAMES == 0):
        pad = 5
        x1, y1 = max(0, x - pad), max(0, y - pad)
        x2, y2 = min(upper_gray.shape[1], x + w + pad), min(upper_gray.shape[0], y + h + pad)
        crop_gray = upper_gray[y1:y2, x1:x2]
        symbol = orb_symbol_match(crop_gray, orb, bf, reference_data)
        if symbol is not None: label = symbol

    return True, label, ratio

def get_stable_label_and_ratio():
    valid_labels = [x for x in label_history if x != "None"]
    if not valid_labels: return "None", 0, 0.0
    stable_label, count = Counter(valid_labels).most_common(1)[0]
    ratios = [ratio_history[i] for i in range(len(label_history)) if label_history[i] == stable_label]
    return stable_label, count, sum(ratios) / len(ratios) if ratios else 0.0

def cal_FPS(frame):
    global prev_time
    now = time.monotonic()
    fps = 1 / (now - prev_time) if now != prev_time else 0
    prev_time = now
    cv.putText(frame, f"FPS: {int(fps)}", (500, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

# =========================================================
# MAIN
# =========================================================
def main():
    global frame_counter, robot_state, hold_until, cooldown_until, locked_label, output_x

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (FRAME_W, FRAME_H)}))
    picam2.start()

    func_init()
    orb, bf, reference_data = prepare_reference_data()

    print("Running Complete Code... (ESC to quit)")

    try:
        while True:
            frame_counter += 1
            now = time.monotonic()

            rgb = picam2.capture_array()
            full_bgr = cv.cvtColor(rgb, cv.COLOR_RGB2BGR)
            full_gray = cv.cvtColor(rgb, cv.COLOR_RGB2GRAY)

            # ---- Upper ROI ----
            upper_rgb = rgb[UPPER_Y1:UPPER_Y2, :]
            upper_bgr = full_bgr[UPPER_Y1:UPPER_Y2, :].copy()
            upper_gray = full_gray[UPPER_Y1:UPPER_Y2, :]

            # ---- State Machine ----
            if robot_state == "HOLD":
                brake_stop()
                
                # FULL FRAME DETECTION: Use the entire 640x480 frame while stopped!
                detected_full, label_full, ratio_full = detect_upper_object_and_label(
                    rgb, full_bgr, full_gray, orb, bf, reference_data, frame_counter
                )
                
                # If it sees a shape in the full frame, override the locked label
                if detected_full and label_full not in ["Unknown", "None"]:
                    locked_label = label_full

                mode_text = f"HOLD ({locked_label})"

                if now >= hold_until:
                    moveForward(50,50)
                    robot_state = "FOLLOW"
                    cooldown_until = now + COOLDOWN_AFTER_HOLD
                    label_history.clear()
                    ratio_history.clear()
                    
                    PID_state['last_time'] = time.monotonic()
                    PID_state['integral'] = 0
                    PID_state['last_error'] = 0
                    output_x = 0

                lower_bgr = full_bgr[LOWER_Y1:LOWER_Y2, LOWER_X1:LOWER_X2].copy()
                # TWEAKED: Strict threshold of 65 in HOLD state
                _, lower_thresh = cv.threshold(cv.GaussianBlur(full_gray[LOWER_Y1:LOWER_Y2, LOWER_X1:LOWER_X2], (3, 3), 0), 65, 255, cv.THRESH_BINARY_INV)

            else:
                # Normal Cruising: Only scan the Upper ROI
                detected, label_now, ratio_now = detect_upper_object_and_label(
                    upper_rgb, upper_bgr, upper_gray, orb, bf, reference_data, frame_counter
                )

                if detected:
                    label_history.append(label_now)
                    ratio_history.append(ratio_now)
                else:
                    label_history.append("None")
                    ratio_history.append(0.0)

                stable_label, stable_count, stable_ratio = get_stable_label_and_ratio()

                cv.putText(upper_bgr, f"Stable: {stable_label} ({stable_count})", (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                cv.putText(upper_bgr, f"Ratio: {stable_ratio*100:.1f}%", (10, 52), cv.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

                in_cooldown = now < cooldown_until
                should_slow = (stable_label != "None" and stable_count >= MIN_CONFIRM_COUNT and stable_ratio >= SLOW_RATIO)
                should_stop = (not in_cooldown and stable_label != "None" and stable_count >= MIN_STOP_CONFIRM_COUNT and stable_ratio >= STOP_RATIO)

                if should_stop:
                    robot_state = "HOLD"
                    locked_label = stable_label
                    hold_until = now + HOLD_TIME_SEC
                    brake_stop()
                    print(f"[STOP] {locked_label} | count={stable_count} | ratio={stable_ratio*100:.1f}%")
                    mode_text = f"HOLD ({locked_label})"
                    lower_bgr = full_bgr[LOWER_Y1:LOWER_Y2, LOWER_X1:LOWER_X2].copy()
                    # TWEAKED: Strict threshold of 65 when transitioning to HOLD
                    _, lower_thresh = cv.threshold(cv.GaussianBlur(full_gray[LOWER_Y1:LOWER_Y2, LOWER_X1:LOWER_X2], (3, 3), 0), 65, 255, cv.THRESH_BINARY_INV)

                elif should_slow:
                    robot_state = "SLOW"
                    mode_text = "SLOW"
                    lower_bgr, lower_thresh = line_follow_on_lower_roi(full_bgr, full_gray, SLOW_LEFT_BASE_SPEED, SLOW_RIGHT_BASE_SPEED)

                else:
                    robot_state = "FOLLOW"
                    mode_text = "FOLLOW"
                    lower_bgr, lower_thresh = line_follow_on_lower_roi(full_bgr, full_gray, NORMAL_LEFT_BASE_SPEED, NORMAL_RIGHT_BASE_SPEED)

            # ---- Display ----
            # If we are NOT holding, paste the upper ROI back. If holding, full_bgr already has the full frame scan lines!
            if robot_state != "HOLD":
                full_bgr[UPPER_Y1:UPPER_Y2, :] = upper_bgr
                
            cv.line(full_bgr, (0, UPPER_Y2), (FRAME_W, UPPER_Y2), (255, 255, 0), 2)
            cv.line(full_bgr, (0, LOWER_Y1), (FRAME_W, LOWER_Y1), (255, 255, 0), 2)
            cv.rectangle(full_bgr, (LOWER_X1, LOWER_Y1), (LOWER_X2, LOWER_Y2), (255, 0, 255), 2)
            cv.putText(full_bgr, f"Mode: {mode_text}", (10, FRAME_H - 15), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cal_FPS(full_bgr)

            cv.imshow("Combined Detection + Line Follow", full_bgr)

            if cv.waitKey(1) & 0xFF == 27:
                break

    finally:
        stop()
        if pwm_left is not None: pwm_left.stop()
        if pwm_right is not None: pwm_right.stop()
        GPIO.cleanup()
        cv.destroyAllWindows()
        picam2.stop()
        print("Closed cleanly.")

if __name__ == "__main__":
    main()