# ✅ 전체 설치 목록 요약

|목적|설치할 것|명령어|
|---|---|---|
|ROS2 핵심|ROS2 Foxy, Humble, Iron 등|ROS 공식 설치 가이드|
|Python 메시지 변환|`cv_bridge`|`sudo apt install ros-${ROS_DISTRO}-cv-bridge`|
|이미지/센서 메시지|`sensor_msgs`, `geometry_msgs`|ROS2 기본 내장됨|
|OpenCV|이미지 표시, 전처리|`pip install opencv-python`|
|NumPy|수학, 배열 처리|`pip install numpy`|
|YOLOv8 (Ultralytics)|객체 인식|`pip install ultralytics`|
|PyTorch|YOLO 동작 필수|`pip install torch torchvision torchaudio`|
|카메라 드라이버 (RealSense)|`realsense2_camera`|`sudo apt install ros-${ROS_DISTRO}-realsense2-camera`|

> ⚠️ `${ROS_DISTRO}`는 본인이 사용하는 ROS2 배포판으로 바꾸세요 (예: `humble`, `foxy`, `iron`)

---

## ✅ 한 줄 요약: Python 패키지

bash

복사편집

`pip install opencv-python numpy ultralytics torch torchvision torchaudio`

---

## ✅ ROS2 관련 패키지

bash

복사편집

`sudo apt update sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-realsense2-camera`

> ROS2의 메시지(`sensor_msgs`, `geometry_msgs`)는 ROS2 설치 시 자동으로 포함돼 있어요.

---

## ✅ Realsense가 없다면

만약 `realsense2_camera` 패키지가 ROS2 저장소에 없을 경우, 다음처럼 직접 빌드도 가능해요:

bash

복사편집

`cd ~/ros2_ws/src git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development cd .. rosdep install --from-paths src --ignore-src -r -y colcon build source install/setup.bash`

---

## 🧸 환경 체크팁

bash

복사편집

`python3 -c "import torch; print(torch.cuda.is_available())"  # GPU 사용 가능 여부`

bash

복사편집

`ros2 topic list  # 카메라 노드가 퍼블리시하는 토픽 확인`

---

## 🎁 추가 팁

|작업|패키지/도구|
|---|---|
|이미지 확인 GUI|`rqt_image_view` (`sudo apt install ros-${ROS_DISTRO}-rqt-image-view`)|
|라벨 필터링|`box.cls` / `model.names[...]`|
|다중 객체 관리|`for box in results.boxes:` 반복|
