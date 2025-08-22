# Face Recognition with RTSP Streaming and MQTT Integration

This project provides a **real-time face recognition system** using **TP-Link Tapo IP cameras (RTSP streaming)**, **InsightFace** for face embedding extraction, and **MQTT** for door access control.  
It is designed for environments where face authentication is required, such as office entry systems, labs, or IoT-integrated access points.

---

## 🚀 Features
- Real-time face recognition via **RTSP camera stream**
- Automatic **face embedding database creation**
- **Frontal face filtering** to improve recognition accuracy
- **MQTT integration** for access control (door open/close)
- Auto-restart of RTSP stream and model to ensure stability
- Configurable **threshold** for cosine similarity-based recognition
- Optional **CUDA GPU acceleration** for faster processing

---

## 📦 Requirements

Core dependencies (see `requirements.txt` for exact versions):
- `opencv-python` – image capture and display
- `numpy` – mathematical operations
- `insightface` – face detection & embeddings (`buffalo_l` model)
- `paho-mqtt` – MQTT communication (for door access control)

### CUDA Acceleration
- By default, the system runs on CPU (`ctx_id=-1`).  
- If CUDA is available, you can enable GPU acceleration by setting:
  ```python
  app.prepare(ctx_id=0)
  ```

## ⚙️ Parameters Explanation

## 1. `Step1.py` – Building Face Dataset
This script captures face images from the RTSP stream and stores them in a dataset folder for later use.

- **`person_name = "x"`**  
  Identifier for a specific person. Can be replaced with a **name** (e.g., `"Alice"`) or a **numeric ID** (e.g., `"001"`).  
  Each person will have a dedicated subfolder under `face_dataset/`.

- **`target_count = 20`**  
  Number of valid face images to capture for building the dataset.  
  Based on testing, **10–20 frontal images per person** provide good recognition accuracy.

- **`save_dir = os.path.join("face_dataset", person_name)`**  
  Storage location for the captured face dataset.  
  Each new person will have a unique folder.

- **`rtsp_url = "rtsp://username:password@192.168.0.100:554/stream1"`**  
  The RTSP stream link of your **TP-Link Tapo camera** (or other IP cameras).  
  The RTSP format is usually found in the device’s manual or app.

- **`min_interval = 0.5`**  
  Minimum interval (in seconds) between saving images.  
  Example: `0.5` means one image every half second.  
  Increase this value to **reduce dataset redundancy**, or decrease for **faster collection**.

---

## 2. `Step2.py` – Building Face Embedding Database
This script processes the collected dataset and generates an **embedding database** (`face_db.pkl`).

- It scans all images stored under `face_dataset/` folders.  
- Extracts embeddings using **InsightFace**.  
- Saves the embeddings into `face_db.pkl`, which will be used for recognition.  

⚠️ No major parameters to adjust here — it simply processes all existing images.

---

## 3. `Main.py` – Real-Time Recognition & Access Control
The main program for recognition and access control.

### 🔑 MQTT Settings

Used to communicate with an Arduino-based access control system:
  ```python
  MQTT_BROKER = "192.168.0.200" 
  MQTT_PORT = 1883
  MQTT_USER = "user" 
  MQTT_PASS = "pass" 
  MQTT_TOPIC_CARD = "new_record"
  MQTT_TOPIC_STATUS = "face_det" 
  ROOM_NAME = "LAB1"
  ```
- MQTT_BROKER → IP address of your MQTT broker
- MQTT_PORT → Default is 1883
- MQTT_USER, MQTT_PASS → Authentication credentials
- MQTT_TOPIC_CARD → Topic to publish recognized face info
- MQTT_TOPIC_STATUS → Topic to publish door open/close events
- ROOM_NAME → Custom room/device identifier

### 📷 RTSP Settings
  ```python
  RTSP_URL = "rtsp://username:password@192.168.0.100:554/stream1"
  os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|buffer_size;4096"
  ```
#### tcp vs udp →
tcp ensures reliability (less frame loss, more stable, slightly higher latency).
udp allows lower latency but may drop frames if the network is unstable.
#### buffer_size → Adjusts video buffering size.
Larger values = more stable but higher delay.
Smaller values = lower delay but risk of dropped frames.

### ⚙️ State Variables
  ```python
  THRESHOLD = 0.5
  last_found = None
  last_valid_time = 0
  last_reset_time = datetime.now()
  frame_count = 0
  door_open = False
  consecutive_none_count = 0
  RECONNECT_THRESHOLD = 50
  ```
#### THRESHOLD = 0.5 → Cosine similarity threshold for recognition.
Higher = stricter matching (fewer false positives, more false negatives).
Lower = looser matching (more sensitive, but risk of false positives).
#### door_open → Tracks door status (True = currently open).
#### RECONNECT_THRESHOLD → Number of failed frames before RTSP reconnect.

👁️ Frontal Face Filter
  ```python
  def is_frontal_face(kps, max_eye_diff_ratio=0.2, max_nose_offset_ratio=0.2):
  ```
#### max_eye_diff_ratio
Controls how much vertical misalignment between eyes is acceptable.
Larger value = allows tilted heads.
Smaller value = stricter (only near-perfect frontal faces).
#### max_nose_offset_ratio
Controls horizontal offset of the nose relative to the eyes.
Larger value = allows more side-facing angles.
Smaller value = stricter (ensures straight frontal view).
