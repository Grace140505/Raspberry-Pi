from picamera2 import Picamera2
import cv2 as cv
import numpy as np

# Configuration
SAMPLE_DICT = {
    0: ("hazardSymbol.jpg", 45),
    1: ("fingerPrint.jpg", 45),
    2: ("recycleSymbol.jpg", 35),
    3: ("qrCode.jpg", 40),
    4: ("pushButton.jpg", 60)
}

COLOR_RANGES = {
    "Green":  {"space": "HSV", "lower": np.array([45, 85, 50]),    "upper": np.array([80, 255, 255])},
    "Yellow": {"space": "HSV", "lower": np.array([25, 150, 50]),   "upper": np.array([35, 255, 255])},
    "Blue":   {"space": "LAB", "lower": np.array([0, 130, 0]),     "upper": np.array([120, 185, 120])},
    "Teal":   {"space": "LAB", "lower": np.array([5, 110, 65]),   "upper": np.array([90, 145, 120])},
    "Purple": {"space": "LAB", "lower": np.array([15, 135, 60]),   "upper": np.array([255, 175, 140])},
    "Red":    {"space": "LAB", "lower": np.array([20, 160, 130]),  "upper": np.array([150, 255, 180])},
    "Orange": {"space": "LAB", "lower": np.array([20, 120, 160]),  "upper": np.array([255, 165, 200])}
}

def is_diamond_vs_trapezium(approx):
    """
    Determine if a 4-vertex convex shape is a Diamond or Trapezium.
    Returns:
        True -> Diamond
        False -> Trapezium (or not diamond)
    """

    # Convert points
    pts = [tuple(pt[0]) for pt in approx]

    # Compute side lengths
    def dist(p1, p2):
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

    sides = [dist(pts[i], pts[(i+1)%4]) for i in range(4)]
    max_side = max(sides)
    min_side = min(sides)

    # Compute diagonal lengths (optional, adds robustness)
    diag1 = dist(pts[0], pts[2])
    diag2 = dist(pts[1], pts[3])

    # Diamond rules:
    # - All sides roughly equal (within 15%)
    # - Diagonals roughly equal (within 20%)
    side_ratio = max_side / min_side
    diag_ratio = max(diag1, diag2) / min(diag1, diag2)


    if side_ratio < 1.2 and diag_ratio < 1.3:
        return True  # Diamond

    # Otherwise, consider it trapezium
    return False

def detect_shape(contour):
    area = cv.contourArea(contour)

    perimeter = cv.arcLength(contour, True)
    approx = cv.approxPolyDP(contour, 0.02 * perimeter, True)
    vertices = len(approx)

    hull = cv.convexHull(contour)
    hull_area = cv.contourArea(hull)
    if hull_area == 0:
        return "Error!!!" , None
    
    solidity = float(area) / hull_area
    is_convex = cv.isContourConvex(approx)
    circularity = 4 * np.pi * area / (perimeter * perimeter)
    x, y, w, h = cv.boundingRect(contour)
    aspect_ratio = w / float(h)
    

    # ---- Shape Rules ----
    if vertices == 4 and is_convex and 0.9 <= aspect_ratio <= 1.35:
        if(is_diamond_vs_trapezium(approx)):
            return "Diamond", None
        else:
            return "Trapezium", None
        
    elif (8 <= vertices <= 9) and is_convex and (0.93 <= solidity <= 1):
        return "Octagon" , None

    elif 10 <= vertices <= 14 and not is_convex and 0.55 < solidity < 0.88:
        return "Plus", None
    
    elif (8 <= vertices <= 10) and (0.53 <= circularity <= 0.84) and (0.85 <= aspect_ratio <= 1.4) and (0.7 < solidity < 0.85):
        return "3/4 Circle", None

    elif( 6 <= vertices <= 7) and is_convex and 0.70 <= circularity <= 0.82: #and (aspect_ratio > 1.5 or aspect_ratio < 0.7):
        return "Semicircle", None

    elif vertices >= 10 and not is_convex and solidity < 0.53:
        return "Star" , None

    elif 7 <= vertices <= 10 and not is_convex and (0.50 <= solidity <= 0.7) :
            # Direction detection
            M = cv.moments(contour)
            if M["m00"] == 0:
                return "Arrow", "Unknown"

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            farthest = max(contour,key=lambda p: (p[0][0]-cx)**2 + (p[0][1]-cy)**2)

            dx = farthest[0][0] - cx
            dy = farthest[0][1] - cy

            if abs(dx) > abs(dy):
                direction = "Left" if dx > 0 else "Right"
            else:
                direction = "Up" if dy > 0 else "Down"
            return "Arrow", direction

    else:
        return "Unknown" , None

def find_simple_shapes(frame, display_frame):

    while True:
        # --- 2. OPTIMIZED BLURRING ---
        # Blur the main RGB frame ONCE, rather than blurring HSV and LAB separately
        blurred_frame = cv.GaussianBlur(frame, (3, 3), 0)
            
        BGR_frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR) # Keep sharp frame for drawing
        HSV_frame = cv.cvtColor(blurred_frame, cv.COLOR_RGB2HSV)
        LAB_frame = cv.cvtColor(blurred_frame, cv.COLOR_RGB2LAB)

        # --- 3. OPTIMIZED CONTOUR TRACKING ---
        # Instead of appending all contours to a list, just track the largest one dynamically
        global_largest_area = 0
        global_largest_contour = None
        global_largest_color = ""

        for color_name, params in COLOR_RANGES.items():
            if params["space"] == "HSV":
                mask = cv.inRange(HSV_frame, params["lower"], params["upper"])
            else:
                mask = cv.inRange(LAB_frame, params["lower"], params["upper"])
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                
            # Check contours immediately without building large intermediate lists
            for contour in contours:
                area = cv.contourArea(contour)
                if area > global_largest_area:
                    global_largest_area = area
                    global_largest_contour = contour
                    global_largest_color = color_name

        # --- 4. DRAW AND DETECT ON THE SINGLE LARGEST CONTOUR ---
        if global_largest_area > 3000 and global_largest_contour is not None:
            shape, direction = detect_shape(global_largest_contour)

            # Draw it
            cv.drawContours(display_frame, [global_largest_contour], -1, (0, 255, 0), 2)
                
            # Build label
            label = f"{global_largest_color} {shape}"
            if direction:
                label += f" ({direction})"
            print(label)

            cv.imshow("Coloured Contours", BGR_frame)
            cv.waitKey(1)
            
            break
        else:
            print("No shape detected")
            break
    
            

def main():
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.start()

    # --- 1. PRE-PROCESSING (Do this once) ---
    orb = cv.ORB_create(nfeatures=1000) # Slightly reduced features for speed
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=False)
    
    reference_data = []
    for key, (img_file, threshold) in SAMPLE_DICT.items():
        img = cv.imread(img_file, cv.IMREAD_GRAYSCALE)
        if img is None:
            print(f"Warning: {img_file} not found.")
            continue
        kp, des = orb.detectAndCompute(img, None)
        reference_data.append({
            "id": key,
            "name": img_file.split('.')[0],
            "kp": kp,
            "des": des,
            "threshold": threshold,
            "img": img
        })
    print("Setup complete. Starting detection...")
    
    try:
        while True:
            frame = picam2.capture_array()
            gray_scene = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
            display_frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

            # Dictionary to accumulate matches for each reference symbol
            match_counts = {ref['id']: 0 for ref in reference_data}
            NUM_SAMPLES = 5

            # --- 1. COLLECT SAMPLES ---
            for _ in range(NUM_SAMPLES):
                # We reuse the same frame/gray_scene for speed, 
                # but you could capture new frames here if you want temporal variety.
                frame = picam2.capture_array()
                gray_scene = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
                kp_scene, des_scene = orb.detectAndCompute(gray_scene, None)
                
                if des_scene is not None:
                    for ref in reference_data:
                        matches = bf.knnMatch(ref["des"], des_scene, k=2)
                        
                        good_matches_count = 0
                        for pair in matches:
                            if len(pair) == 2:
                                m, dist_n = pair # Renamed 'n' to 'dist_n' to avoid conflict
                                if m.distance < 0.75 * dist_n.distance:
                                    good_matches_count += 1
                        
                        match_counts[ref['id']] += good_matches_count

            # --- 2. CHECK AVERAGES ---
            found_any = False
            for ref in reference_data:
                average_matches = match_counts[ref['id']] / float(NUM_SAMPLES)
                
                if average_matches >= ref["threshold"]:
                    print(f"Detected: {ref['name']} (Avg: {average_matches:.1f})")
                    found_any = True
                    break # Found the best match, stop checking others
            
            if not found_any:
                find_simple_shapes(frame, display_frame)  # Call shape detection if no symbol detected

            cv.imshow("Detection", display_frame)
            if cv.waitKey(1) & 0xFF == 27:
                break
    finally:
        print("Closing resources...")
        cv.destroyAllWindows()
        picam2.stop()

if __name__ == "__main__":
    main()