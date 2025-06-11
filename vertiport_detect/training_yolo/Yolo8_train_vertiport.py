from ultralytics import YOLO

# 사전학습된 모델 불러오기 (예: yolov8n: 가장 가벼운 모델)
model = YOLO("yolov8s.pt")
model

# 학습 시작
model.train(
    data="/home/haechan/Repo_2025/machine_AI/cv_uav_control_for_landing/vertiport_detect/training_yolo/real_dataset/data.yaml",
    epochs=200,    # 학습 에폭 수
    imgsz=640,      # 입력 이미지 사이즈 (기본: 640)
    patience=50,   # Early Stopping patience
    save=True,     # 학습 후 모델 저장 여부
    name="train_vertiport",  # 결과 폴더 이름
    workers=8,     # 데이터 로딩을 위한 워커 수
    optimizer='SGD',  # 옵티마이저 (기본값: 'SGD')
    lr0=0.01,      # 초기 학습률
    lrf=0.1,       # 최종 학습률 = lr0 * lrf
    momentum=0.937,  # 모멘텀 값
    weight_decay=5e-4,  # 가중치 감쇠
    warmup_epochs=3.0,   # 워밍업 에폭 수
    warmup_momentum=0.8, # 워밍업 모멘텀
    warmup_bias_lr=0.1,  # 워밍업 바이어스 학습률
    box=7.5,       # 박스 손실 가중치
    cls=0.5,       # 클래스 손실 가중치
    dfl=1.5,       # DFL 손실 가중치
    label_smoothing=0.1,# 라벨 스무딩 비율 (기본: 0.0)
    mixup=0.0,     # MixUp 비율 (기본: 0.0)
    val=True,      # 검증 데이터셋 사용 여부
    device='cuda',
    batch=8       # 기본값은 모델/메모리에 따라 자동 설정됨     # 입력 이미지 사이즈 (기본: 640)
)
