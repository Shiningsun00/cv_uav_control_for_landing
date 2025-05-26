from ultralytics import YOLO

# 사전학습된 모델 불러오기 (예: yolov8n: 가장 가벼운 모델)
model = YOLO("yolov8s.pt")

# 학습 시작
model.train(
    data="/home/haechan/Repo_2025/machine_AI/vertiport_detect/training_yolo/real_dataset/data.yaml",
    epochs=1000 ,
    device='cuda',
    batch=16       # 기본값은 모델/메모리에 따라 자동 설정됨     # 입력 이미지 사이즈 (기본: 640)
)
