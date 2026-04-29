from flask import Flask, request, jsonify, Response
import cv2
import numpy as np
import time
import pickle
import face_recognition

app = Flask(__name__)

# Load your trained model
print("[INFO] Loading face encodings...")
try:
    with open("encodings.pickle", "rb") as f:
        data = pickle.loads(f.read())
    known_face_encodings = data["encodings"]
    known_face_names = data["names"]
    print(f"[INFO] Loaded model with names: {set(known_face_names)}")
except Exception as e:
    print(f"[ERROR] Could not load encodings.pickle: {e}")
    known_face_encodings, known_face_names = [], []

latest_frame = None
latest_time = ""

@app.route('/')
def index():
    return '''
    <html>
        <body style="background: #111; color: white; text-align: center; font-family: Arial;">
            <h1>Face Recognition Live Stream</h1>
            <p>Status: <span id="time">---</span></p>
            <img src="/stream" style="border: 4px solid #0f0; max-width: 90%;">
            <script>
                setInterval(() => {
                    fetch('/time').then(r => r.json()).then(d => {
                        document.getElementById('time').textContent = d.time + " | " + (d.name || "No Face");
                    });
                }, 500);
            </script>
        </body>
    </html>
    '''

@app.route('/time')
def get_time():
    return jsonify({"time": latest_time})

@app.route('/stream')
def stream():
    def generate():
        global latest_frame
        while True:
            if latest_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
            time.sleep(0.05) 
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/analyze', methods=['POST'])
def analyze_face():
    nparr = np.frombuffer(request.data, np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    if frame is None: 
        return jsonify({"faces": []})

    # Shrink to speed up
    small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    faces_data = []
    for face_encoding, (top, right, bottom, left) in zip(face_encodings, face_locations):
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding, tolerance=0.5)
        name = "Unknown"
        
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        if len(face_distances) > 0:
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_face_names[best_match_index]
        
        # Scale bounding box back up to normal size
        faces_data.append({
            "name": name,
            "location": [top * 2, right * 2, bottom * 2, left * 2]
        })

    # Return coordinates to the Pi so the Pi can draw it on the stream!
    return jsonify({"faces": faces_data})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)