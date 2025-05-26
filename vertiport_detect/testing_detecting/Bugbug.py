import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

# YOLO 모델 로드 (학습된 best.pt 경로 수정)
model = YOLO("/home/haechan/Repo_2025/machine_AI/vertiport_detect/training_yolo/runs/detect/train6/weights/best.pt")

# RealSense 카메라 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

try:
    print("🚀 RealSense 카메라에서 프레임을 받아 저장 중...")
    
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow("RealSense Live Feed", color_image)

        # 저장 키: 's' 키 입력 시 프레임 저장 및 추론 실행
        key = cv2.waitKey(1)
        if key == ord('s'):
            save_path = "check_frame.jpg"
            cv2.imwrite(save_path, color_image)
            print(f"🖼 프레임 저장됨: {save_path}")

            # YOLO 모델로 수동 추론
            print("🔍 저장된 프레임으로 YOLO 추론 중...")
            results = model.predict(source=save_path, show=True, conf=0.5)

            # 탐지 결과 출력
            if results[0].boxes:
                print("✅ 객체 감지됨!")
                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    print(f" → 클래스 ID: {cls_id}, 정확도: {conf:.2f}")
            else:
                print("❌ 탐지된 객체가 없습니다.")

        # 종료 키: ESC
        elif key == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
