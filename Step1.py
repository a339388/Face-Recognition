import cv2
import os
import time
import insightface
import numpy as np

# 初始化人臉分析器
app = insightface.app.FaceAnalysis(name='buffalo_l')
app.prepare(ctx_id=0)  # 沒有 GPU 改成 ctx_id=-1

# 設定參數
person_name = "x"
target_count = 20
save_dir = os.path.join("face_dataset", person_name)
rtsp_url = "rtsp://etc000:Mor35473%21@192.168.0.220:554/stream1"
min_interval = 0.5  # 每幾秒儲存一次

os.makedirs(save_dir, exist_ok=True)

# 檢查是否為正面人臉
def is_frontal_face(kps, max_eye_diff_ratio=0.1, max_nose_offset_ratio=0.1):
    left_eye, right_eye, nose, left_mouth, right_mouth = kps

    eye_y_diff = abs(left_eye[1] - right_eye[1])
    eye_dist = np.linalg.norm(left_eye - right_eye)
    eye_y_ratio = eye_y_diff / eye_dist

    eye_mid_x = (left_eye[0] + right_eye[0]) / 2
    nose_offset = abs(nose[0] - eye_mid_x)
    nose_offset_ratio = nose_offset / eye_dist

    return (eye_y_ratio < max_eye_diff_ratio) and (nose_offset_ratio < max_nose_offset_ratio)

# 開啟 RTSP
cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
if not cap.isOpened():
    print("❌ 無法開啟 RTSP 串流")
    exit()

saved_count = 0
last_saved_time = 0

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("⚠️ 串流讀取失敗")
        continue

    # 判斷是否出現正面人臉
    faces = app.get(frame)
    has_frontal = False
    for face in faces:
        if is_frontal_face(face.kps):
            has_frontal = True
            break

    # 儲存整張畫面
    current_time = time.time()
    if has_frontal and current_time - last_saved_time > min_interval:
        filename = f"{saved_count:03}.jpg"
        cv2.imwrite(os.path.join(save_dir, filename), frame)
        print(f"✅ 儲存整張圖：{filename}")
        saved_count += 1
        last_saved_time = current_time

    # 顯示即時畫面（但不標註）
    cv2.imshow("RTSP Preview", frame)
    if cv2.waitKey(1) & 0xFF == ord('q') or saved_count >= target_count:
        break

cap.release()
cv2.destroyAllWindows()
