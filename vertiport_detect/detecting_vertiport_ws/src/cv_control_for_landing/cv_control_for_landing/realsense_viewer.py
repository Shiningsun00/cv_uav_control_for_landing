from ultralytics import YOLO  # ✅ YOLOv8은 이렇게 import해야 함
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
class RealSenseYOLOViewer(Node):
    def __init__(self):
        super().__init__('realsense_yolo_viewer')

        self.subscription = self.create_subscription(
            Image,
            '/camera/realsense_front/color/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

        # ✅ YOLOv8 모델 로드 (본인 경로로 수정)
        model_path = '/home/haechan/Repo_2025/machine_AI/vertiport_detect/training_yolo/runs/detect/train9/weights/best.pt'
        self.model = YOLO(model_path)

        self.get_logger().info("📷 YOLOv8 + RealSense 뷰어 시작됨!")

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ✅ YOLOv8 추론 (OpenCV 이미지 그대로 넣기)
        results = self.model(cv_image)[0]

        # ✅ 결과 이미지 만들기
        annotated_frame = results.plot()

        cv2.imshow("YOLOv8 Detection", annotated_frame)
        cv2.waitKey(1)
def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYOLOViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 종료됨.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
