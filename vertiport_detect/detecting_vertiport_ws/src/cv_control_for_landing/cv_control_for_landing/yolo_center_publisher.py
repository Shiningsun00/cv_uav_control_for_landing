import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class YOLOCenterPublisher(Node):
    def __init__(self):
        super().__init__('yolo_center_publisher')

        # YOLO 모델 로드
        model_path = '/home/haechan/Repo_2025/machine_AI/vertiport_detect/training_yolo/runs/detect/train9/weights/best.pt'
        self.model = YOLO(model_path)

        # 카메라 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/realsense_front/color/image_raw',
            self.image_callback,
            10
        )

        # 깊이 이미지 구독 (depth aligned to color)
        self.depth_sub = self.create_subscription(
            Image,
           '/camera/realsense_front/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # 깊이 프레임 저장용
        self.latest_depth_image = None

        # 중심 좌표 퍼블리셔
        self.publisher = self.create_publisher(Point, '/yolo_center', 10)

        self.bridge = CvBridge()
        self.get_logger().info("🚀 YOLO 라벨 + 깊이 좌표 퍼블리셔 노드 시작!")

    def depth_callback(self, msg):
        # 최신 depth 이미지 저장 (단위: mm → m 변환 필요 시 여기서 처리)
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if self.latest_depth_image is None:
            self.get_logger().warn("아직 depth 프레임 없음. 대기 중...")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls_id = int(box.cls[0].item())
            cls_name = self.model.names[cls_id]

            # 중심 좌표 계산
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # 중심 좌표에 해당하는 깊이 값 가져오기
            if 0 <= cy < self.latest_depth_image.shape[0] and 0 <= cx < self.latest_depth_image.shape[1]:
                depth_value = self.latest_depth_image[cy, cx]  # mm 단위
                depth_in_m = float(depth_value) / 1000.0  # m 단위로 변환
            else:
                depth_in_m = 0.0

            # ROS 메시지로 퍼블리시
            point = Point()
            point.x = float(cx)
            point.y = float(cy)
            point.z = depth_in_m
            self.publisher.publish(point)

            # 출력
            self.get_logger().info(f"🎯 객체: {cls_name} | 중심: ({cx}, {cy}) | 깊이: {depth_in_m:.2f} m")

        # 시각화
        annotated = results.plot()
        cv2.imshow("YOLOv8 Detection with Depth", annotated)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOCenterPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 종료됨.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
