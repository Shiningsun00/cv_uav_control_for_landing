import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO


class VertiportDetect:
    def __init__(self, model_path):
        # YOLO 모델 로드 (학습된 best.pt 경로 수정)
        self.model = YOLO(model_path)

    # Real Sence 카메라 설정
    def setting_camera(self, camera_width, camera_height, fps):    
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, camera_width, camera_height, rs.format.bgr8, fps)
        self.pipeline.start(self.config)
        
    def detecting_vertiport(self):
        try:
            print("🚀 실시간 YOLO 탐지 시작... (ESC 누르면 종료)")

            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # 프레임 → 이미지
                color_image = np.asanyarray(color_frame.get_data())

                results = self.model.predict(
                    source=color_image,      # 실시간 프레임 (numpy array)
                    conf=0.5,                # confidence threshold
                    augment=True,            # ✅ TTA 활성화
                    stream=False,
                    verbose=False
                )
                # 탐지 결과 시각화
                annotated_frame = results[0].plot()  # 바운딩 박스 및 라벨을 포함한 이미지 반환

                # 화면에 출력
                cv2.imshow("YOLO Real-Time Detection", annotated_frame)

                # 종료 조건: ESC
                key = cv2.waitKey(1)
                if key == 27:
                    break

        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

def main():
    # YOLO 모델 경로
    model_path = "/home/haechan/Repo_2025/machine_AI/vertiport_detect/training_yolo/runs/detect/train9/weights/best.pt"
    
    # VertiportDetect 클래스 인스턴스 생성
    vertiport_detector = VertiportDetect(model_path)
    # 카메라 설정
    vertiport_detector.setting_camera(camera_width=1920, camera_height=1080, fps=30)
    # 탐지 시작
    vertiport_detector.detecting_vertiport()
    
    
if __name__ == "__main__":
    main()
