import os
import cv2
import pickle
import numpy as np
import insightface

# 載入模型
app = insightface.app.FaceAnalysis(name='buffalo_l')
app.prepare(ctx_id=0)  # 無 GPU 改成 -1

# 設定資料夾
dataset_root = "face_dataset"
face_db = {}

for person in os.listdir(dataset_root):
    person_dir = os.path.join(dataset_root, person)
    if not os.path.isdir(person_dir):
        continue

    embeddings = []
    for fname in os.listdir(person_dir):
        img_path = os.path.join(person_dir, fname)
        img = cv2.imread(img_path)
        if img is None:
            continue

        faces = app.get(img)
        if len(faces) == 0:
            print(f"⚠️ 無法偵測臉: {img_path}")
            continue

        face = faces[0]
        embeddings.append(face.embedding)

    if embeddings:
        face_db[person] = embeddings
        print(f"✅ {person} 已儲存 {len(embeddings)} 筆 embedding")
    else:
        print(f"⛔ {person} 無有效臉部資料")

# 儲存資料庫
with open("face_db.pkl", "wb") as f:
    pickle.dump(face_db, f)
    print("📦 face_db.pkl 儲存完成！")
