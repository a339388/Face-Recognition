import cv2
import time
import pickle
import numpy as np
import paho.mqtt.client as mqtt
import insightface
import json
import os
import subprocess
from datetime import datetime, timedelta
from threading import Thread
cv2.startWindowThread()
# ========== RTSP 多執行緒讀取 ==========
class FFmpegRTSPStream:
    def __init__(self, rtsp_url, width=1280, height=720):
        print("1-1")
        self.rtsp_url = rtsp_url
        self.width = width
        self.height = height
        self.frame_size = width * height * 3  # bgr24
        self.process = None
        self.frame = None
        self.stopped = False
        self.thread = Thread(target=self.update, daemon=True)
        self.restart()  # 初次啟動
        self.thread.start()
        print("1-2")

    def restart(self):
        print("1-3")
        if self.process:
            self.process.kill()
            self.process.stdout.close()
        ffmpeg_cmd = [
            "ffmpeg",
            "-rtsp_transport", "udp",
            "-i", self.rtsp_url,
            "-vf", "fps=15,scale=1280:720",   # ← 加入限制幀率與縮放解析度
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-an", "-sn",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-probesize", "32",
            "-analyzeduration", "0",
            "-loglevel", "quiet",
            "-"
        ]
        print("1-4")
        self.process = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, bufsize=10**8)
        print("🔁 FFmpeg 已啟動")

    def update(self):
        missed_count = 0
        while not self.stopped:
            try:
                raw_frame = self.process.stdout.read(self.frame_size)
                if len(raw_frame) != self.frame_size:
                    missed_count += 1
                    if missed_count > 10:
                        print("⚠️ FFmpeg frame 不足，自動重啟")
                        self.restart()
                        missed_count = 0
                    time.sleep(0.2)
                    continue
                frame = np.frombuffer(raw_frame, np.uint8).reshape((self.height, self.width, 3))
                self.frame = frame
                missed_count = 0
            except Exception as e:
                print(f"❌ FFmpeg frame 讀取失敗: {e}")
                self.restart()
                time.sleep(1)

    def read(self):
        return self.frame

    def stop(self):
        print("4")
        self.stopped = True
        self.thread.join()
        print("5")
        if self.process:
            self.process.kill()
            self.process.stdout.close()
        print("6")

# ========== MQTT 設定 ==========
MQTT_BROKER = "120.118.142.140"
MQTT_PORT = 1883
MQTT_USER = "ip"
MQTT_PASS = "etc7337383"
MQTT_TOPIC_CARD = "new_record"
MQTT_TOPIC_STATUS = "face_IP"
ROOM_NAME = "IP"

# ========== 判斷是否為正面臉 ==========
def is_frontal_face(kps, max_eye_diff_ratio=0.2, max_nose_offset_ratio=0.2):
    left_eye, right_eye, nose, left_mouth, right_mouth = kps
    eye_y_diff = abs(left_eye[1] - right_eye[1])
    eye_dist = np.linalg.norm(left_eye - right_eye)
    eye_y_ratio = eye_y_diff / eye_dist
    eye_mid_x = (left_eye[0] + right_eye[0]) / 2
    nose_offset = abs(nose[0] - eye_mid_x)
    nose_offset_ratio = nose_offset / eye_dist
    return (eye_y_ratio < max_eye_diff_ratio) and (nose_offset_ratio < max_nose_offset_ratio)

# ========== 初始化模型與串流 ==========
def init_face_analysis():
    app = insightface.app.FaceAnalysis(name="buffalo_l")
    app.prepare(ctx_id=0)
    return app

# ========== 載入 face_db ==========
with open("face_db.pkl", "rb") as f:
    face_db = pickle.load(f)

# ========== MQTT ==========
def safe_publish(topic, payload, retry=3):
    for _ in range(retry):
        result = mqtt_client.publish(topic, payload)
        if result.rc == 0:
            return True
        time.sleep(0.2)
    print(f"❌ MQTT 發送失敗，topic = {topic}")
    return False

# ========== 初始化 MQTT ==========
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

# ========== RTSP 設定 ==========
RTSP_URL = "rtsp://etc000:Mor35473!@192.168.0.220:554/stream1"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|buffer_size;4096"

cap = FFmpegRTSPStream(RTSP_URL)
app = init_face_analysis()

# ========== 狀態變數 ==========
THRESHOLD = 0.5
last_found = None
last_valid_time = 0
last_reset_time = datetime.now()
frame_count = 0
door_open = False  # 🚪 控制門開啟狀態
consecutive_none_count = 0  # ⏱️ RTSP 逾時計數器
RECONNECT_THRESHOLD = 50    # ⏱️ 連續失敗 50 次（約 3 秒）就重連

print("🚀 開始辨識中... 按 Ctrl+C 結束")

try:
    while True:
        now = datetime.now()
        mqtt_client.loop(timeout=0.1)
        # ========== 超時自動關門 ==========
        if door_open and (time.time() - last_valid_time > 5):
            mqtt_client.publish(MQTT_TOPIC_STATUS, 'off')
            print(f"📤 發送 MQTT: topic = {MQTT_TOPIC_STATUS} → off")
            last_found = None
            door_open = False

        # ========== 每小時重啟模型與串流 ==========
        if now - last_reset_time > timedelta(hours=1):
            print("🔄 一小時已到，自動釋放並重啟 RTSP 與模型")
            try:
                cap.stop()
            except Exception as e:
                print(f"⚠️ cap.stop() 失敗: {e}")
            time.sleep(5)
            print("1")
            cap = FFmpegRTSPStream(RTSP_URL)  # ← 重新建立新物件
            print("2")
            app = init_face_analysis()
            print("3")
            last_reset_time = now
            print("✅ 重啟完成")


        # ========== 讀取影像 ==========
        frame = cap.read()
        if frame is None:
            consecutive_none_count += 1
            if consecutive_none_count >= RECONNECT_THRESHOLD:
                print("⚠️ RTSP 連續讀取失敗，自動重啟串流...")
                cap.stop()
                time.sleep(1)
                cap = FFmpegRTSPStream(RTSP_URL)
                consecutive_none_count = 0
                print("✅ 串流重啟完成")
            continue
        else:
            consecutive_none_count = 0  # 成功讀取 → 重置計數器

        frame_count += 1

        # 每3幀只顯示，不做辨識（降負載）
        if frame_count % 5 != 0:
            if frame is not None and frame.shape == (720, 1280, 3):
                cv2.imshow("Face Recognition", frame)
            else:
                print("⚠️ 無效的 frame，跳過顯示")
            key = cv2.waitKey(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # ========== 門開啟中，跳過辨識 ==========
        if door_open:
            frame = frame.copy()
            cv2.putText(frame, "門已開啟中...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            if frame is not None and frame.shape == (720, 1280, 3):
                cv2.imshow("Face Recognition", frame)
            else:
                print("⚠️ 無效的 frame，跳過顯示")
            key = cv2.waitKey(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.2)
            continue

        # ========== 臉部辨識 ==========
        found = None
        faces = app.get(frame)

        if faces:
            faces = sorted(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]), reverse=True)
            for face in faces:
                x1, y1, x2, y2 = map(int, face.bbox)
                width = x2 - x1
                height = y2 - y1
                if width < 60 or height < 80:
                    continue
                if not is_frontal_face(face.kps):
                    continue

                frame_ori = frame.copy()
                emb = face.embedding / np.linalg.norm(face.embedding)
                best_match = None
                best_score = 0
                
                for name, ref_emb_list in face_db.items():
                    for ref_emb in ref_emb_list:
                        sim = np.dot(emb, ref_emb / np.linalg.norm(ref_emb))  # 確保已正規化
                        if sim > best_score:
                            best_score = sim
                            best_match = name
                
                if best_score > THRESHOLD:
                    found = best_match
                else:
                    found = None

        # ========== 狀態改變才觸發動作 ==========
        if found and found != last_found:
            frame = frame.copy()
            # 標示 + 儲存影像
            x1, y1, x2, y2 = map(int, face.bbox)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, found, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            timestamp = now.strftime("%Y%m%d%H%M%S")
            save_dir = "captures"
            os.makedirs(save_dir, exist_ok=True)
            cv2.imwrite(os.path.join(save_dir, f"{found}_{timestamp}_ori.jpg"), frame_ori)
            cv2.imwrite(os.path.join(save_dir, f"{found}_{timestamp}_mem.jpg"), frame)

            # 發送 MQTT
            payload_card = {
                "room": ROOM_NAME,
                "card": found,
                "auth": True
            }
            safe_publish(MQTT_TOPIC_STATUS, 'on')
            safe_publish(MQTT_TOPIC_CARD, json.dumps(payload_card, separators=(',', ':')))

            print(f"📤 發送 MQTT: topic = {MQTT_TOPIC_CARD}")
            print(f"📦 payload = {json.dumps(payload_card, ensure_ascii=False)}")
            print(f"📤 發送 MQTT: topic = {MQTT_TOPIC_STATUS} → on")

            last_found = found
            last_valid_time = time.time()
            door_open = True

        # ========== 顯示畫面 ==========
        if frame is not None and frame.shape == (720, 1280, 3):
            cv2.imshow("Face Recognition", frame)
        else:
            print("⚠️ 無效的 frame，跳過顯示")
        key = cv2.waitKey(1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.2)

except KeyboardInterrupt:
    print("🛑 結束辨識")

finally:
    cap.stop()
    cv2.destroyAllWindows()
