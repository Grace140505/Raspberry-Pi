# NOTE: OpenCV capture image as RGB and expect BGR image for display
"""
Multiprocessing line-following + image-recognition robot.

Process layout
──────────────
  main        – camera capture  ▸  motor control  ▸  display
  line_worker   – line detection  ▸  PID calculation
  img_worker    – ORB symbol / colour-shape recognition

Frame sharing uses multiprocessing.shared_memory (zero-copy),
so no pickling overhead between processes.
"""

from picamera2 import Picamera2
import cv2 as cv
import numpy as np
import time
import RPi.GPIO as GPIO
from collections import deque
import multiprocessing as mp
from multiprocessing import shared_memory
import requests

# ══════════════════════════════════════════════════════════════
# PIN DECLARATIONS  (BCM)
# ══════════════════════════════════════════════════════════════
IN1, IN2 = 17, 27   # motor_a (Left) direction
IN3, IN4 = 22, 23   # motor_b (Right) direction
ENA, ENB = 18, 19   # PWM enable

# ══════════════════════════════════════════════════════════════
# FLASK CONFIG
# ══════════════════════════════════════════════════════════════
PC_IP_ADDRESS   = "172.20.10.5"
PC_URL          = f"http://{PC_IP_ADDRESS}:5000/analyze"
REQUEST_TIMEOUT = 5.0
#REQUEST_TIMEOUT is set to 5 seconds to balance between allowing enough time for the Flask server to process the image and return a response, while also ensuring that the Raspberry Pi does not wait indefinitely if there are issues with the network connection or if the Flask server is unresponsive. This timeout helps maintain the responsiveness of the robot's main loop by preventing it from getting stuck waiting for a response from the Flask server.
# ══════════════════════════════════════════════════════════════
# FRAME CONFIG
# ══════════════════════════════════════════════════════════════
FRAME_W      = 480
FRAME_H      = 360
FRAME_SHAPE  = (FRAME_H, FRAME_W, 3)
FRAME_NBYTES = FRAME_H * FRAME_W * 3

LINE_DISP_SHAPE  = (180, FRAME_W, 3)
LINE_DISP_NBYTES = 180 * FRAME_W * 3
IMG_DISP_SHAPE   = (FRAME_H, FRAME_W, 3)
IMG_DISP_NBYTES  = FRAME_H * FRAME_W * 3

# ══════════════════════════════════════════════════════════════
# SPEED / PID CONSTANTS
# ══════════════════════════════════════════════════════════════
MOTOR_A_SPEED_NORMAL,  MOTOR_B_SPEED_NORMAL  = 23, 23
MOTOR_A_SPEED_PUSH,    MOTOR_B_SPEED_PUSH    = 25, 25

KP, KI, KD     = 0.42, 0.010, 0.0315
X_CENTRE_REF   = 240
Y_CENTRE_REF   = 100

# ══════════════════════════════════════════════════════════════
# IMAGE-RECOGNITION CONFIG
# ══════════════════════════════════════════════════════════════
SAMPLE_DICT = {
    0: (["pushButton-1.jpg",    "pushButton-2.jpg",    "pushButton-3.jpg"],    25),
    1: (["fingerPrint-1.jpg",   "fingerPrint-2.jpg",   "fingerPrint-3.jpg"],   25),
    2: (["qrCode-1.jpg",        "qrCode-2.jpg",        "qrCode-3.jpg"],        15),
    3: (["recycleSymbol.jpg"],  20),
    4: (["hazardSymbol-1.jpg",  "hazardSymbol-2.jpg",  "hazardSymbol-3.jpg"],  25)
}

SYMBOL_NAMES = {
    0: "pushButton",
    1: "fingerPrint",
    2: "qrCode",
    3: "recycleSymbol",
    4: "hazardSymbol",
}

#hsv = hue, saturation, value
IMAGE_COLOUR_RANGES = {
    "Green":    {"space": "HSV", "lower": np.array([40,  60,  50]), "upper": np.array([85,  255, 255])},
    "Yellow":   {"space": "HSV", "lower": np.array([25, 150,  50]), "upper": np.array([35,  255, 255])},
    "Purple":   {"space": "LAB", "lower": np.array([0, 135,  60 ]), "upper": np.array([255, 195, 135])},
    "Blue/Teal":{"space": "LAB", "lower": np.array([0 , 100,  60]), "upper": np.array([230, 165, 120])},
    "Red":      {"space": "LAB", "lower": np.array([0 , 160, 130]), "upper": np.array([255, 255, 180])},
    "Orange":   {"space": "LAB", "lower": np.array([0, 130, 165 ]), "upper": np.array([255, 180, 200])},
}

LINE_COLOUR_RANGES = {
    "Red":    {"lower_1": np.array([0, 100, 100]), "lower_2": np.array([160, 100, 100]), "upper_1": np.array([10, 255, 255]), "upper_2": np.array([180, 255, 255])},
    "Yellow": {"lower": np.array([20, 80, 80]),    "upper": np.array([40, 255, 255])},
    "Black":  {"lower": np.array([0, 0, 0]),       "upper": np.array([180, 255, 70])}
}

LABEL_TO_INSTRUCTION = {
    "Arrow (Left)":  "TURN_LEFT",
    "Arrow (Right)": "TURN_RIGHT",
    "Arrow (Up)":    "MOVE_FORWARD",
    "pushButton":    "STOP",
    "hazardSymbol":  "STOP",
    "recycleSymbol": "360-TURN",
    "fingerPrint":   "SCAN_STOP",
    "qrCode":        "SCAN_STOP",
}

# ══════════════════════════════════════════════════════════════
# COLOUR → SYMBOL / SHAPE RULES
# ══════════════════════════════════════════════════════════════
# Which ORB symbol IDs to attempt per colour
COLOUR_TO_ORB_IDS = {
    "Yellow":   [4],
    "Blue/Teal":[1, 2],
    "Purple":   [1, 2],
    "Green":    [0, 3],
}

# Which shapes are permitted per colour (None = all shapes allowed)
COLOUR_TO_SHAPES = {
    "Orange": {"Plus"},           # Orange → Plus only
    "Purple": {"Diamond", "Trapezium"},  # Purple → Diamond / Trapezium only
    # All other colours: any shape (no restriction)
}

# ══════════════════════════════════════════════════════════════
# FLASK HELPER
# ══════════════════════════════════════════════════════════════

def send_frame_to_flask(frame_rgb: np.ndarray):
#frame_bgr is the expected format for OpenCV display, but Flask server expects RGB bytes, so convert back before encoding
    frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
#successfully encode the frame as JPEG with reasonable quality for transmission
    success, encoded = cv.imencode('.jpg', frame_bgr, [cv.IMWRITE_JPEG_QUALITY, 85])
    if not success:
        print("[main] Failed to encode frame for Flask.")
        return None
#respond to Flask with raw JPEG bytes, setting appropriate content type and timeout
    try:
        res = requests.post(
            PC_URL,
            data=encoded.tobytes(),
            headers={'Content-Type': 'application/octet-stream'},
            timeout=REQUEST_TIMEOUT
        )
        if res.status_code == 200:
            data = res.json()
            print(f"[main] Flask response: {data}")
            return data
        else:
            print(f"[main] Flask returned status {res.status_code}")
    except requests.exceptions.Timeout:
        print("[main] Flask request timed out.")
    except requests.exceptions.ConnectionError:
        print(f"[main] Cannot reach Flask at {PC_URL}.")
    except Exception as e:
        print(f"[main] Flask error: {e}")
    return None

# ══════════════════════════════════════════════════════════════
# MOTOR HELPERS  (main process only)
# ══════════════════════════════════════════════════════════════
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    for pin in (IN1, IN2, IN3, IN4, ENA, ENB):
        GPIO.setup(pin, GPIO.OUT)
    pwm_motor_a = GPIO.PWM(ENA, 50); pwm_motor_a.start(0)
    pwm_motor_b = GPIO.PWM(ENB, 50); pwm_motor_b.start(0)
    return pwm_motor_a, pwm_motor_b

def _drive_side(in_a, in_b, pwm, speed):
    if speed < 0:
        GPIO.output(in_a, GPIO.HIGH); GPIO.output(in_b, GPIO.LOW)
        speed = max(3, min(100, abs(speed)))
    else:
        GPIO.output(in_a, GPIO.LOW); GPIO.output(in_b, GPIO.HIGH)
        speed = max(3, min(100, speed))
    pwm.ChangeDutyCycle(speed)

def move_forward(pwm_motor_a, pwm_motor_b, speed_motor_a, speed_motor_b):
    _drive_side(IN1, IN2, pwm_motor_a, speed_motor_a)
    _drive_side(IN3, IN4, pwm_motor_b, speed_motor_b)

def stop_motors(pwm_motor_a, pwm_motor_b):
    for pin in (IN1, IN2, IN3, IN4): GPIO.output(pin, GPIO.LOW)
    pwm_motor_a.ChangeDutyCycle(0)
    pwm_motor_b.ChangeDutyCycle(0)

# ══════════════════════════════════════════════════════════════
# SHARED HELPERS
# ══════════════════════════════════════════════════════════════
# Given a binary mask, find the largest contour and return it along with its area.
def best_contour(mask):
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0
    c = max(contours, key=cv.contourArea)
    return c, cv.contourArea(c)
# Safely write a string into a fixed-size shared array, ensuring null-termination and truncation if necessary.
def _write_str(arr: mp.Array, text: str, max_bytes: int):
    enc = text.encode()[:max_bytes - 1]
    arr.raw = enc + b'\x00' * (max_bytes - len(enc))
# Read a null-terminated string from a shared array, stripping padding and decoding it.
def _read_str(arr: mp.Array) -> str:
    return arr.raw.rstrip(b'\x00').decode(errors='replace')

# ══════════════════════════════════════════════════════════════
# LINE-FOLLOWING WORKER PROCESS
# ══════════════════════════════════════════════════════════════
def line_worker(
# shared memory and synchronization primitives for frame access
    shm_name, frame_lock, shared_fid, my_fid,
    out_pid, out_cx, out_cy, out_has_line, out_lineArea,
    out_is_priority, out_turn_cmd,
    disp_shm_name, disp_lock,
):
#Attach to shared memory for frame access
    shm  = shared_memory.SharedMemory(name=shm_name)
#Create a NumPy array view for the frame buffer
    fbuf = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)
#Attach to shared memory for line display
    disp_shm = shared_memory.SharedMemory(name=disp_shm_name)
#Create a NumPy array view for the line display buffer
    disp_buf = np.ndarray(LINE_DISP_SHAPE, dtype=np.uint8, buffer=disp_shm.buf)
#Initialize PID state and other variables
    pid_state = {'last_error': 0.0, 'integral': 0.0, 'last_time': time.monotonic()}
    pid_out = 0.0
    cx, cy  = X_CENTRE_REF, 150

    prev_fps_time = time.monotonic()
    smoothed_fps  = 0.0

    lane_memory    = None
    red_left_votes = 0; red_right_votes = 0
    was_on_red     = False; was_on_yellow = False

    while True:
# Wait for a new frame by monitoring the shared frame ID
        fid = shared_fid.value
        if fid == my_fid.value:
            time.sleep(0.002); continue
        my_fid.value = fid
#Calculate smoothed FPS based on time elapsed since last frame
        now = time.monotonic()
        dt_fps = now - prev_fps_time; prev_fps_time = now
        if dt_fps > 0:
            smoothed_fps = 0.9 * smoothed_fps + 0.1 / dt_fps

        with frame_lock: frame = fbuf.copy()
#crop to bottom half of the frame for line detection, convert to BGR for display, and prepare HSV for color masking
        crop_rgb = frame[180:360, :]
        crop_bgr = cv.cvtColor(crop_rgb, cv.COLOR_RGB2BGR)
        blur     = cv.GaussianBlur(crop_rgb, (3, 3), 0)
        hsv      = cv.cvtColor(blur, cv.COLOR_RGB2HSV)
#Create binary masks for each line color based on predefined HSV ranges
        mask_black  = cv.inRange(hsv, LINE_COLOUR_RANGES["Black"]["lower"],  LINE_COLOUR_RANGES["Black"]["upper"])
        mask_yellow = cv.inRange(hsv, LINE_COLOUR_RANGES["Yellow"]["lower"], LINE_COLOUR_RANGES["Yellow"]["upper"])
        mask_red    = cv.bitwise_or(
            cv.inRange(hsv, LINE_COLOUR_RANGES["Red"]["lower_1"], LINE_COLOUR_RANGES["Red"]["upper_1"]),
            cv.inRange(hsv, LINE_COLOUR_RANGES["Red"]["lower_2"], LINE_COLOUR_RANGES["Red"]["upper_2"])
        )
#Find the largest contour and area for each color mask
        cnt_black,  area_black  = best_contour(mask_black)
        cnt_yellow, area_yellow = best_contour(mask_yellow)
        cnt_red,    area_red    = best_contour(mask_red)
#Helper function to safely get bounding box from a contour, returning (0,0,0,0) if contour is None
        def get_bbox(cnt): return cv.boundingRect(cnt) if cnt is not None else (0, 0, 0, 0)
        xr, yr, wr, hr = get_bbox(cnt_red)
        xy, yy, wy, hy = get_bbox(cnt_yellow)
        xb, yb, wb, hb = get_bbox(cnt_black)
# Decision logic to determine which line to follow based on area and size thresholds, with a priority for red, then yellow, then black. 
# Also includes voting mechanism for red line direction if lane memory is not set.
        if area_red > 4500 and hr > 50 and wr > 30: #hr > 100
            cnts = [cnt_red] if cnt_red is not None else []
            follow_colour = "Red"; draw_colour = (0, 0, 255)
        elif area_yellow > 4500 and hy > 135 and wy > 20:
            cnts = [cnt_yellow] if cnt_yellow is not None else []
            follow_colour = "Yellow"; draw_colour = (0, 255, 255)
            if not was_on_yellow:
                print(f"\n[line_worker] ---> DETECTING YELLOW LINE (Box: {wy}x{hy}) <---\n")
                was_on_yellow = True
        elif area_black > 4500:
            cnts = [cnt_black] if cnt_black is not None else []
            follow_colour = "Black"; draw_colour = (0, 255, 0)
        else:
            cnts = []; follow_colour = "None"; draw_colour = (0, 255, 0)

        if follow_colour != "Yellow": was_on_yellow = False

        has_line = False; current_area = 0.0
# If a valid contour exists for the chosen line color, calculate its properties, update PID state, and draw it on the display buffer.
        if cnts:
            has_line = True
            largest_contour = max(cnts, key=cv.contourArea)
            current_area    = cv.contourArea(largest_contour)
# Calculate bounding box and center of the largest contour, draw it on the display buffer, and compute PID output based on horizontal error from center reference.
            x, y, w, h = cv.boundingRect(largest_contour)
            cv.rectangle(crop_bgr, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv.putText(crop_bgr, f"w:{w} h:{h}", (x, max(10, y - 10)), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
# Calculate moments to find the centroid of the contour, with a fallback to bounding box center if moments are degenerate
            M = cv.moments(largest_contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00']); cy = int(M['m01'] / M['m00'])
            else:
                cx, cy = x + w // 2, y + h // 2

            cv.drawContours(crop_bgr, [largest_contour], -1, draw_colour, 2)
# PID calculation: compute error as horizontal distance from center reference, update integral and derivative terms, and compute final PID output.
            error = X_CENTRE_REF - cx
            now   = time.monotonic(); dt = now - pid_state['last_time']; pid_state['last_time'] = now
            P = KP * error
            pid_state['integral'] += error * dt; I = KI * pid_state['integral']
            D = (KD * (error - pid_state['last_error']) / dt) if dt > 0.01 else 0.0
            pid_state['last_error'] = error; pid_out = P + I + D
        else:
            pid_state['last_time'] = time.monotonic()
# Voting mechanism for red line direction if lane memory is not set.
        if follow_colour == "Red":
            if lane_memory is None:
# If we have no lane memory, vote based on whether the red line's centroid is left or right of the center reference, using the black line as a secondary reference if available.
                cx_blk = X_CENTRE_REF
                if cnt_black is not None and area_black > 1000:
                    M_blk = cv.moments(cnt_black)
                    if M_blk['m00'] != 0: cx_blk = int(M_blk['m10'] / M_blk['m00'])
                if cx < cx_blk: red_left_votes += 1
                else:           red_right_votes += 1
                if red_left_votes > 5:
                    lane_memory = "Left"
                    print(f"\n[line_worker] VOTING COMPLETE: Locked LEFT ({red_left_votes} votes)\n")
                elif red_right_votes > 7:
                    lane_memory = "Right"
                    print(f"\n[line_worker] VOTING COMPLETE: Locked RIGHT ({red_right_votes} votes)\n")
            was_on_red = True
        elif follow_colour == "Black":
            if was_on_red:
                if lane_memory == "Left":  out_turn_cmd.value = 1
                elif lane_memory == "Right": out_turn_cmd.value = 2
                was_on_red = False; lane_memory = None; red_left_votes = 0; red_right_votes = 0
        else:
            was_on_red = False
# Draw center reference and contour centroid on the display buffer, and put FPS text. Then copy the display buffer to shared memory for the main process to show.   
        cv.circle(crop_bgr, (X_CENTRE_REF, Y_CENTRE_REF), 5, (0, 255, 255), -1)
        cv.circle(crop_bgr, (cx, cy), 5, (0, 0, 255), -1)
        cv.putText(crop_bgr, f"FPS: {int(smoothed_fps)}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

        with disp_lock: np.copyto(disp_buf, crop_bgr)

        out_pid.value      = pid_out
        out_cx.value       = cx
        out_cy.value       = cy
        out_is_priority.value = (follow_colour in ["Red", "Yellow"])
        out_lineArea.value = current_area
        out_has_line.value = has_line


# ══════════════════════════════════════════════════════════════
# IMAGE-RECOGNITION HELPERS
# ══════════════════════════════════════════════════════════════
# Given a BFMatcher, reference entries with descriptors, scene descriptors, and a match threshold, determine if any reference matches the scene sufficiently.
def orb_match_symbol(bf, ref_entries, des_scene, threshold):
    for ref in ref_entries:
        if ref["des"] is None: continue
        matches = bf.knnMatch(ref["des"], des_scene, k=2)
        good = sum(1 for m in matches if len(m) == 2 and m[0].distance < 0.72 * m[1].distance)
        if good >= threshold: return True, good
    return False, 0
# Determine if a 4-vertex contour approximates a diamond shape by checking the ratios of its sides and diagonals.
def is_diamond(approx):
    pts  = [tuple(p[0]) for p in approx]
    dist = lambda a, b: ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5
    sides = [dist(pts[i], pts[(i+1) % 4]) for i in range(4)]
    d1 = dist(pts[0], pts[2]); d2 = dist(pts[1], pts[3])
    return (max(sides)/min(sides) < 1.25) and (max(d1,d2)/min(d1,d2) < 1.3)
# Given a contour, analyze its geometric properties (area, perimeter, solidity, circularity, aspect ratio) and return a shape label based on predefined criteria. 
# Also determine arrow direction if applicable.
def detect_shape(contour):
    area      = cv.contourArea(contour)
    peri      = cv.arcLength(contour, True)
    approx    = cv.approxPolyDP(contour, 0.02 * peri, True)
    v         = len(approx)
    hull_area = cv.contourArea(cv.convexHull(contour))
    if hull_area == 0: return "Unknown", None

    solidity     = area / hull_area
    is_convex    = cv.isContourConvex(approx)
    circ         = 4 * np.pi * area / (peri * peri)
    x, y, w, h  = cv.boundingRect(contour)
    aspect_ratio = w / float(h)

    if v == 4 and is_convex and 0.7 <= aspect_ratio <= 1.35:
        return ("Diamond" if is_diamond(approx) else "Trapezium"), None
    if 8 <= v <= 9  and is_convex and 0.93 <= solidity <= 1:
        return "Octagon", None
    if 10 <= v <= 14 and not is_convex and 0.55 < solidity < 0.88:
        return "Plus", None
    if 8 <= v <= 10 and 0.53 <= circ <= 0.84 and 0.85 <= aspect_ratio <= 1.4 and 0.7 <= solidity <= 0.85:
        return "Pacman", None
    if 6 <= v <= 7  and is_convex and 0.70 <= circ <= 0.82:
        return "Segment", None
    if v >= 10 and not is_convex and 0.49 <= solidity <= 0.55 and 0.25 <= circ <= 0.35:
        return "Star", None
    if 7 <= v <= 10 and not is_convex and 0.52 <= solidity <= 0.68 and circ >= 0.15:
        M = cv.moments(contour)
        if M["m00"] == 0: return "Arrow", "Unknown"
        cx_ = int(M["m10"] / M["m00"]); cy_ = int(M["m01"] / M["m00"])
        far = max(contour, key=lambda p: (p[0][0]-cx_)**2 + (p[0][1]-cy_)**2)
        dx = far[0][0] - cx_; dy = far[0][1] - cy_
        if abs(dx) > abs(dy): direction = "Left" if dx > 0 else "Right"
        else:                 direction = "Up"   if dy > 0 else "Down"
        return "Arrow", direction
    return "Unknown", None


# ══════════════════════════════════════════════════════════════
# IMAGE-RECOGNITION WORKER PROCESS
# ══════════════════════════════════════════════════════════════
def image_worker(
    shm_name, frame_lock, shared_fid, my_fid,
    out_found, out_label, out_instruction, out_instruction_ready,
    disp_shm_name, disp_lock, out_is_priority,
):
    shm      = shared_memory.SharedMemory(name=shm_name)
    fbuf     = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)
    disp_shm = shared_memory.SharedMemory(name=disp_shm_name)
    disp_buf = np.ndarray(IMG_DISP_SHAPE, dtype=np.uint8, buffer=disp_shm.buf)

# Initialize ORB detector and BFMatcher for feature matching
    orb = cv.ORB_create(nfeatures=500, nlevels=8, fastThreshold=17)
# bf is set to crossCheck=False to allow for knnMatch and ratio test, which is more robust for symbol recognition than strict one-to-one matching. 
# The distance norm is set to NORM_HAMMING, which is appropriate for ORB descriptors.
    bf  = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=False)
# Precompute ORB keypoints and descriptors for all reference images, storing them in a structured format for efficient access during matching.
    reference_data = []
    for symbol_id, (img_files, threshold) in SAMPLE_DICT.items():
        refs = []
        for img_file in img_files:
            img = cv.imread(img_file, cv.IMREAD_GRAYSCALE)
            if img is None:
                print(f"[img_worker] WARNING: '{img_file}' not found – skipping.")
                refs.append({"filename": img_file, "kp": None, "des": None}); continue
# kp (keypoints) and des (descriptors) are computed for each reference image using ORB. 
# Keypoints represent distinctive locations in the image
# While descriptors are binary vectors that describe the local appearance around each keypoint.
            kp, des = orb.detectAndCompute(img, None)
            refs.append({"filename": img_file, "kp": kp, "des": des})
            print(f"[img_worker] Loaded '{img_file}' — {len(kp)} keypoints.")
        reference_data.append({
            "id": symbol_id, "name": SYMBOL_NAMES[symbol_id],
            "threshold": threshold, "refs": refs
        })
# Create a dictionary to quickly access reference entries by their symbol ID during matching.
    ref_by_id     = {entry["id"]: entry for entry in reference_data}
    label_history = deque(maxlen=3)
    cooldown_counter = 0
    missed_frames    = 0

    while True:
        fid = shared_fid.value
        if fid == my_fid.value:
            time.sleep(0.002); continue
        my_fid.value = fid

        with frame_lock: frame = fbuf.copy()
        display_bgr = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

        found = False; label = ""; instruction = ""
        best_contour_for_display = None
        current_is_priority = bool(out_is_priority.value)

        blurred = cv.GaussianBlur(frame, (3, 3), 0)
        HSV = cv.cvtColor(blurred, cv.COLOR_RGB2HSV)
        LAB = cv.cvtColor(blurred, cv.COLOR_RGB2LAB)

        # ══════════════════════════════════════════════════════
        # CANDIDATE COLLECTION
        #
        # Green  → keep ALL contours >= 1200 px²
        # Others → keep only the single largest contour >= 1200 px²
        #
        # Evaluation order:
        #   1. Non-green candidates (top 3 by area, same as original)
        #   2. Green candidates (all, sorted largest-first)
        #
        # BUT: if no non-green candidates exist at all, skip straight
        #      to evaluating all green candidates.
        # ══════════════════════════════════════════════════════
        non_green_candidates = []   # list of (area, cnt, colour_name)
        green_candidates     = []   # list of (area, cnt, "Green")
# Iterate through each defined colour range, create a mask, find contours, and collect candidates based on the specified rules for green vs non-green colours.
        for colour_name, params in IMAGE_COLOUR_RANGES.items():
            src  = HSV if params["space"] == "HSV" else LAB
            mask = cv.inRange(src, params["lower"], params["upper"])
            raw_contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
# For green contours, keep all that exceed the area threshold. 
# For non-green contours, keep only the single largest one that exceeds the threshold. 
# Store candidates in separate lists for later evaluation.
            if colour_name == "Green":
                # ALL contours above threshold
                for cnt in raw_contours:
                    a = cv.contourArea(cnt)
                    if a >= 1200:
                        green_candidates.append((a, cnt, "Green"))
            else:
                # Only the single largest contour per non-green colour
                valid = [(cv.contourArea(c), c) for c in raw_contours if cv.contourArea(c) >= 1200]
                if valid:
                    best_area, best_cnt = max(valid, key=lambda x: x[0])
                    non_green_candidates.append((best_area, best_cnt, colour_name))

        # Sort both pools largest-first
        non_green_candidates.sort(key=lambda x: x[0], reverse=True)
        green_candidates.sort(key=lambda x: x[0], reverse=True)

        # If non-green candidates exist → non-green (top 3) first, then green
        # If NO non-green candidates exist → all green candidates only
        if non_green_candidates:
            all_candidates = non_green_candidates[:3] + green_candidates
        else:
            all_candidates = green_candidates   # full list, no cap

        # ══════════════════════════════════════════════════════
        # ORB SCENE DESCRIPTORS  (compute once if needed)
        # ══════════════════════════════════════════════════════
# Check if any candidate is eligible for ORB matching based on its colour. If so, compute ORB descriptors for the entire scene once to use for all symbol matches, avoiding redundant computations.
        orb_eligible = any(colour in COLOUR_TO_ORB_IDS for _, _, colour in all_candidates)
        des_s = None
        if orb_eligible:
            gray_scene = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
            _, des_s   = orb.detectAndCompute(gray_scene, None)

        # ══════════════════════════════════════════════════════
        # EVALUATE EACH CANDIDATE
        # ══════════════════════════════════════════════════════
# For each candidate contour, first attempt ORB symbol matching if its colour is eligible. 
# If that fails and the candidate is not from a priority colour, attempt shape detection with colour-based restrictions. 
# Stop at the first successful match.
        for area, cnt, detected_colour in all_candidates:

            # ── 1. ORB symbol matching (colour-gated) ──────────
# Only attempt ORB matching if the candidate's colour is in the COLOUR_TO_ORB_IDS mapping and if ORB descriptors for the scene were successfully computed. 
# This ensures we only spend time on ORB matching when it's likely to yield results based on our predefined rules.
# des_s will be None if ORB descriptors couldn't be computed for the scene (e.g., due to an error), in which case we skip ORB matching entirely to avoid unnecessary processing and potential errors.
            if detected_colour in COLOUR_TO_ORB_IDS and des_s is not None:
                for sym_id in COLOUR_TO_ORB_IDS[detected_colour]:
                    entry = ref_by_id[sym_id]
# matched will be True if any of the reference images for this symbol ID have enough good matches with the scene descriptors based on the defined threshold.
                    matched, _ = orb_match_symbol(bf, entry["refs"], des_s, entry["threshold"])
                    if matched:
                        label = entry["name"]
                        found = True
                        best_contour_for_display = cnt
                        break

            # ── 2. Shape detection (colour-restricted, skip if priority) ──
# If ORB matching did not find a symbol and the candidate's colour is not currently priority (i.e., red or yellow line), attempt shape detection.
# The detected shape must also satisfy any colour-to-shape restrictions defined in COLOUR_TO_SHAPES to be accepted.
            if not found and not current_is_priority:
                shape, direction = detect_shape(cnt)
# Only consider the shape valid if it's not "Unknown". 
# If the colour has specific allowed shapes defined, the detected shape must be in that set to be accepted.
                if shape != "Unknown":
                    # Check colour-to-shape restriction
                    allowed_shapes = COLOUR_TO_SHAPES.get(detected_colour, None)
# If allowed_shapes is None, there are no restrictions for this colour and any detected shape is accepted.
                    if allowed_shapes is None:
                        # No restriction for this colour — accept any shape
                        label = shape + (f" ({direction})" if direction else "")
                        found = True
                        best_contour_for_display = cnt
# If allowed_shapes is a set, only accept the shape if it's in the allowed set for this colour. 
# Otherwise, skip silently without marking as found.
                    elif shape in allowed_shapes:
                        # Shape is in the allowed set for this colour
                        label = shape + (f" ({direction})" if direction else "")
                        found = True
                        best_contour_for_display = cnt
                    # else: shape not allowed for this colour — skip silently

            if found:
                break

        # ══════════════════════════════════════════════════════
        # CONFIRMATION & INSTRUCTION DISPATCH
        # ══════════════════════════════════════════════════════
# If a symbol/shape was found and we're not in cooldown, add the label to the history.
# If we get 3 consistent detections in a row, confirm the label and set the corresponding instruction.
        if found and cooldown_counter == 0:
            label_history.append(label)
            missed_frames = 0
            if len(label_history) == label_history.maxlen and len(set(label_history)) == 1:
                confirmed_label = label_history[0]
                instruction     = LABEL_TO_INSTRUCTION.get(confirmed_label, "")
                print(f"----------\n[img_worker] Detected : {confirmed_label}")
                if instruction: print(f"[img_worker] Instruction: {instruction}")
                print(f"----------")
                label_history.clear(); cooldown_counter = 10
# If no symbol/shape was found, or if we're in cooldown, increment the missed frames counter. 
# If we miss too many frames in a row, clear the label history and reset the instruction to avoid stale detections.
        else:
            if cooldown_counter > 0: cooldown_counter -= 1
            missed_frames += 1
            if missed_frames > 4:
                label_history.clear(); instruction = ""

        # ══════════════════════════════════════════════════════
        # DISPLAY
        # ══════════════════════════════════════════════════════
# If the current line-following state is priority (red/yellow line), overlay a "Line Following Only" message on the display to indicate that symbol recognition is currently deprioritized.
        if current_is_priority:
            cv.putText(display_bgr, "Line Following Only", (10, 40),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
# If a contour was selected for display (either from ORB symbol match or shape detection), draw its bounding box and label on the display.
        if best_contour_for_display is not None:
            x, y, w, h = cv.boundingRect(best_contour_for_display)
            cv.rectangle(display_bgr, (x, y), (x + w, y + h), (255, 0, 0), 3)
            if label:
                cv.putText(display_bgr, label, (x, max(10, y - 10)),
                           cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        elif label:
            cv.putText(display_bgr, label, (10, 80),
                       cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
# Finally, copy the annotated display buffer to shared memory for the main process to show, and update the shared output values for found status, label, and instruction.
        with disp_lock: np.copyto(disp_buf, display_bgr)

        out_found.value = found
        _write_str(out_label, label, 64)
        if instruction:
            _write_str(out_instruction, instruction, 32)
            out_instruction_ready.value = True

# ══════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════
def main():
# Create shared memory for the camera frame and display buffers, along with synchronization primitives and shared variables for inter-process communication.
    shm  = shared_memory.SharedMemory(create=True, size=FRAME_NBYTES)
    fbuf = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)
# Shared memory for line-following display and image-recognition display, along with locks to synchronize access to these buffers between the main process and the worker processes.
    line_disp_shm = shared_memory.SharedMemory(create=True, size=LINE_DISP_NBYTES)
    img_disp_shm  = shared_memory.SharedMemory(create=True, size=IMG_DISP_NBYTES)
    line_disp_buf = np.ndarray(LINE_DISP_SHAPE, dtype=np.uint8, buffer=line_disp_shm.buf)
    img_disp_buf  = np.ndarray(IMG_DISP_SHAPE,  dtype=np.uint8, buffer=img_disp_shm.buf)
    line_disp_lock, img_disp_lock = mp.Lock(), mp.Lock()
# Lock for synchronizing access to the camera frame buffer between the main process and the worker processes.
    frame_lock = mp.Lock()
    shared_fid = mp.Value('i',  0)
    line_fid   = mp.Value('i', -1)
    img_fid    = mp.Value('i', -1)
# Shared variables for line-following outputs (PID value, contour center, line area, has_line flag) and image-recognition outputs (found flag, label, instruction, priority flag, turn command).
    out_pid      = mp.Value('d', 0.0)
    out_cx       = mp.Value('i', X_CENTRE_REF)
    out_cy       = mp.Value('i', Y_CENTRE_REF)
    out_lineArea = mp.Value('d', 0.0)
    out_has_line = mp.Value('b', False)
# Shared variables for image-recognition outputs
    out_found             = mp.Value('b', False)
    out_label             = mp.Array('c', 64)
    out_instruction       = mp.Array('c', 32)
    out_instruction_ready = mp.Value('b', False)
    out_is_priority       = mp.Value('b', False)
    out_turn_cmd          = mp.Value('i', 0)
# Create and start the line-following and image-recognition worker processes, passing them the necessary shared memory names, locks, and shared variables for communication.
    p_line = mp.Process(
        target=line_worker,
        args=(shm.name, frame_lock, shared_fid, line_fid,
              out_pid, out_cx, out_cy, out_has_line, out_lineArea,
              out_is_priority, out_turn_cmd,
              line_disp_shm.name, line_disp_lock),
        daemon=True, name="LineWorker"
    )
# The image worker is given access to the same shared frame buffer and synchronization primitives, but also has its own shared variables for communicating symbol detection results and instructions back to the main process.
    p_img = mp.Process(
        target=image_worker,
        args=(shm.name, frame_lock, shared_fid, img_fid,
              out_found, out_label, out_instruction, out_instruction_ready,
              img_disp_shm.name, img_disp_lock, out_is_priority),
        daemon=True, name="ImgWorker"
    )
# Start both worker processes. 
# They will run in the background, continuously processing frames and updating shared variables for the main process to read and act upon.
    p_line.start(); p_img.start()
    print("[main] Workers started.")

    pwm_motor_a, pwm_motor_b = setup_gpio()
# Initialize the camera, configure it for preview mode with the specified frame size, and start it. 
# Wait a couple of seconds to ensure the camera is ready before entering the main loop.
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (FRAME_W, FRAME_H)}))
    picam2.start()
    time.sleep(2.0)
    print("[main] Camera ready. Press ESC to quit.")

    line_loss_counter  = 0
    active_instruction = ""
    display_frame_skip = 2
    loop_count         = 0
# Main loop: continuously capture frames from the camera, update shared memory and variables for the workers, 
# read outputs from the workers, and dispatch motor commands based on the current instruction and line-following state.
    try:
        while True:
            RGB = picam2.capture_array()
            if RGB.ndim == 3 and RGB.shape[2] == 4: RGB = RGB[:, :, :3]

            with frame_lock: np.copyto(fbuf, RGB)
            shared_fid.value += 1
            loop_count += 1

            pid      = out_pid.value
            cx       = out_cx.value
            cy       = out_cy.value
            has_line = bool(out_has_line.value)
            found    = bool(out_found.value)
            label    = _read_str(out_label)
            current_is_priority = bool(out_is_priority.value)

            new_instr = ""
            if out_instruction_ready.value:
                new_instr = _read_str(out_instruction)
                out_instruction_ready.value = False

            turn_cmd = out_turn_cmd.value
            if turn_cmd != 0:
                if turn_cmd == 1:
                    print("[main] Exiting priority line — turning LEFT")
                    move_forward(pwm_motor_a, pwm_motor_b, MOTOR_A_SPEED_PUSH, MOTOR_B_SPEED_PUSH); time.sleep(0.25)
                    move_forward(pwm_motor_a, pwm_motor_b, -40, 55); time.sleep(0.8)
                elif turn_cmd == 2:
                    print("[main] Exiting priority line — turning RIGHT")
                    move_forward(pwm_motor_a, pwm_motor_b, MOTOR_A_SPEED_PUSH, MOTOR_B_SPEED_PUSH); time.sleep(0.25)
                    move_forward(pwm_motor_a, pwm_motor_b, 55, -40); time.sleep(0.8)
                out_turn_cmd.value = 0
# new_instr will be non-empty if the image worker has confirmed a new instruction based on consistent detections.
            if new_instr:
                active_instruction = new_instr
                print(f"[main] Instruction '{active_instruction}' confirmed.")
                _write_str(out_instruction, "", 32)

            # ── INSTRUCTION DISPATCH ────────────────────────────
            if active_instruction:
                if active_instruction == "TURN_LEFT":
                    move_forward(pwm_motor_a, pwm_motor_b, MOTOR_A_SPEED_NORMAL - pid, MOTOR_B_SPEED_NORMAL + pid); time.sleep(0.3)
                    move_forward(pwm_motor_a, pwm_motor_b, -45, 55); time.sleep(0.2)
                    move_forward(pwm_motor_a, pwm_motor_b, MOTOR_A_SPEED_NORMAL - pid, MOTOR_B_SPEED_NORMAL + pid); time.sleep(0.2)
                    move_forward(pwm_motor_a, pwm_motor_b, -45, 55); time.sleep(0.5)
                    stop_motors(pwm_motor_a, pwm_motor_b); active_instruction = ""

                elif active_instruction == "TURN_RIGHT":
                    move_forward(pwm_motor_a, pwm_motor_b, MOTOR_A_SPEED_NORMAL - pid, MOTOR_B_SPEED_NORMAL + pid); time.sleep(0.5)
                    move_forward(pwm_motor_a, pwm_motor_b, 55, -35); time.sleep(0.5)
                    stop_motors(pwm_motor_a, pwm_motor_b); active_instruction = ""

                elif active_instruction == "MOVE_FORWARD":
                    move_forward(pwm_motor_a, pwm_motor_b, MOTOR_A_SPEED_NORMAL, MOTOR_B_SPEED_NORMAL); time.sleep(1)
                    active_instruction = ""

                elif active_instruction == "STOP":
                    stop_motors(pwm_motor_a, pwm_motor_b); time.sleep(2)
                    move_forward(pwm_motor_a, pwm_motor_b, 50, 50); time.sleep(0.2)
                    active_instruction = ""

                elif active_instruction == "360-TURN":
                    move_forward(pwm_motor_a, pwm_motor_b, MOTOR_A_SPEED_NORMAL, MOTOR_B_SPEED_NORMAL); time.sleep(0.5)
                    move_forward(pwm_motor_a, pwm_motor_b, -70, 70); time.sleep(2.1)
                    stop_motors(pwm_motor_a, pwm_motor_b); active_instruction = ""

                elif active_instruction == "SCAN_STOP":
                    print("[main] SCAN_STOP — halting for 10 s, continuously sending frames to Flask…")
                    stop_motors(pwm_motor_a, pwm_motor_b)
                    
                    # Set a 10-second timer
                    end_time = time.monotonic() + 20.0
                    
                    while time.monotonic() < end_time:
                        remaining = max(0, int(end_time - time.monotonic()))
                        # Use carriage return to keep the countdown on the same line
                        print(f"\r[main] Scanning pause … {remaining}s remaining   ", end="", flush=True)
                        
                        scan_frame = picam2.capture_array()
                        if scan_frame.ndim == 3 and scan_frame.shape[2] == 4:
                            scan_frame = scan_frame[:, :, :3]
                            
                        flask_result = send_frame_to_flask(scan_frame)
                        
                        if flask_result:
                            faces = flask_result.get("faces", [])
                            if faces:
                                names = [f["name"] for f in faces]
                                # Print on a new line so it doesn't overwrite the countdown
                                print(f"\n[main] Flask identified: {', '.join(names)}")
                            # Note: "No faces" print is omitted here to prevent console spam, 
                            # leaving only the countdown heartbeat or positive detections.
                        else:
                            # Print on a new line if connection fails
                            print(f"\n[main] No response from Flask.")
                            
                    print("\n[main] Scan complete — continuing in 5 seconds.")
                    time.sleep(4.5)
                    active_instruction = ""

            else:
                # Normal line-following drive
# If the current line-following state is priority (red/yellow line), use fixed motor speeds to ensure stable tracking, ignoring PID adjustments.
# If not priority, adjust motor speeds based on the PID output to steer towards the line, with different base speeds if a symbol/shape was found to provide a more assertive response.
# If the line is lost, continue with normal speeds for a short grace period before initiating a turn to try to reacquire the line, with the turn direction based on the last known PID error sign. 
                if current_is_priority:
                    L_base, R_base = 24, 23
                else:
                    L_base = MOTOR_A_SPEED_PUSH if found else MOTOR_A_SPEED_NORMAL
                    R_base = MOTOR_B_SPEED_PUSH if found else MOTOR_B_SPEED_NORMAL

                if has_line:
                    move_forward(pwm_motor_a, pwm_motor_b, L_base - pid, R_base + pid)
                    line_loss_counter = 0
                else:
                    if line_loss_counter <= 8:
                        move_forward(pwm_motor_a, pwm_motor_b, MOTOR_A_SPEED_NORMAL, MOTOR_B_SPEED_NORMAL)
                        line_loss_counter += 1
                    else:
                        if pid > 0: move_forward(pwm_motor_a, pwm_motor_b, -45, 55)
                        else:       move_forward(pwm_motor_a, pwm_motor_b,  55, -45)

            if loop_count % display_frame_skip == 0:
                with line_disp_lock: line_frame = line_disp_buf.copy()
                with img_disp_lock:  img_frame  = img_disp_buf.copy()

                if active_instruction:
                    cv.putText(line_frame, f"CMD: {active_instruction}", (10, 220), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv.putText(img_frame,  f"CMD: {active_instruction}", (10, 120), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                cv.imshow("Line Following",  line_frame)
                # cv.imshow("Image Detection", img_frame)

                if loop_count > 100: loop_count = 0

            if cv.waitKey(1) & 0xFF == 27: break

    except KeyboardInterrupt:
        print("\n[main] Ctrl+C received — stopping.")
    except Exception as e:
        import traceback; print(f"\n[main] ERROR: {e}"); traceback.print_exc()
    finally:
        print("[main] Shutting down…")
        try: stop_motors(pwm_motor_a, pwm_motor_b)
        except: pass
        try: pwm_motor_a.stop(); pwm_motor_b.stop()
        except: pass
        p_line.terminate(); p_line.join()
        p_img.terminate();  p_img.join()
        try:
            shm.close(); shm.unlink()
            line_disp_shm.close(); line_disp_shm.unlink()
            img_disp_shm.close();  img_disp_shm.unlink()
        except: pass
        try: GPIO.cleanup()
        except: pass
        try: picam2.stop()
        except: pass
        cv.destroyAllWindows()
        print("[main] Done.")


if __name__ == "__main__":
    mp.set_start_method("forkserver")
    main()
