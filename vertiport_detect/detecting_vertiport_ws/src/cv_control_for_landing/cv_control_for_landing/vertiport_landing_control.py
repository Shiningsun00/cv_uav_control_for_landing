import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import numpy as np
from sensor_msgs.msg import CameraInfo

class LandingControl(Node):
    def __init__(self):
        super().__init__("landing_control")
        self.get_logger().info("✅ Landing Control Node Initialized")

        # YOLO 바운딩 박스 중심 기준 픽셀 오차
        self.dx_pixel = None
        self.dy_pixel = None
        self.depth_z = None  # YOLO에서 전달된 실제 깊이 정보 (meter)
        self.camera_info_received = False

        self.fx = None  # 카메라 내부 파라미터
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/realsense_front/color/camera_info",  # 또는 /camera/depth/camera_info
            self.camera_info_callback,
            10
        )

        # 속도 명령 초기화
        self.max_speed = 0.5

        # 착륙 조건 설정
        self.xy_tolerance = 20.0           # 픽셀 기준 오차 허용 범위
        self.landing_depth_limit = 0.3     # 착륙 완료로 간주할 깊이 (m)

        # 구독자: YOLO 중심 및 깊이
        self.point_sub = self.create_subscription(Point, "/yolo_center", self.yolo_center_callback, 10)

        # 퍼블리셔: 속도 명령
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # 타이머: 주기적 제어
        self.timer = self.create_timer(0.1, self.control_landing)  # 10Hz
    def camera_info_callback(self, msg):
        if self.camera_info_received:
            return  # 이미 받았으면 무시

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.get_logger().info(
            f"📷 Camera Info 수신됨 → fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}"
        )

        self.camera_info_received = True
        
    def yolo_center_callback(self, msg):
        self.dx_pixel = msg.x - self.cx
        self.dy_pixel = msg.y - self.cy

        # 깊이값 유효성 검사
        if msg.z > 0.1:  # 10cm 이하의 값은 신뢰하지 않음
            self.depth_z = msg.z
        else:
            self.get_logger().warn("⚠️ 유효하지 않은 깊이값 수신됨 (z=0 또는 너무 작음) → 이전 값 유지")
            # self.depth_z = None  # ← 완전히 제거할 경우, 다음 step에서 control_landing이 종료됨


    def control_landing(self):
        # 기본 조건 확인
        if self.dx_pixel is None or self.dy_pixel is None or self.depth_z is None:
            return

        # 깊이값 유효성 검사 (깊이값이 10cm 이하이면 무효로 간주하고 착륙 중단)
        if self.depth_z < 0.1:
            self.get_logger().warn("🛑 깊이 정보 무효(z < 0.1m) → 착륙 제어 중단")
            return

        # 픽셀 deadzone 적용
        deadzone_threshold = 2.0  # 픽셀 단위
        if abs(self.dx_pixel) < deadzone_threshold:
            self.dx_pixel = 0.0
        if abs(self.dy_pixel) < deadzone_threshold:
            self.dy_pixel = 0.0

        # 기본 속도 계산 (z 포함된 방향으로 계산되지만 이후 필요 시 제거됨)
        velocity_frd = self.pixel_error_to_velocity_frd(
            self.dx_pixel, self.dy_pixel, self.fx, self.fy, self.max_speed
        )

        # 착륙 조건: 오차가 작고, 아직 높으면 하강 명령
        if abs(self.dx_pixel) < self.xy_tolerance and abs(self.dy_pixel) < self.xy_tolerance:
            if self.depth_z > self.landing_depth_limit:
                # 착륙 유도 시작
                velocity_frd[0] = 0.3   # Down (FRD 기준 z)
                velocity_frd[1] = 0.0   # Forward 정지
                velocity_frd[2] = 0.0   # Right 정지
                self.get_logger().info("🛬 착륙 유도 중...")
            else:
                # 착륙 완료
                velocity_frd = np.array([0.0, 0.0, 0.0])
                self.get_logger().info("✅ 착륙 완료 (지면 근접)")
        else:
            # ❗ 정렬 중에는 z축 하강을 차단해야 함
            velocity_frd[0] = 0.0  # FRD z = Down 방향 속도 차단

        # Twist 메시지 변환 (FRD → ROS Twist)
        vel_msg = Twist()
        vel_msg.linear.x = float(velocity_frd[1])   # Forward
        vel_msg.linear.y = float(-velocity_frd[2])  # Right (좌우 반전 보정)
        vel_msg.linear.z = float(velocity_frd[0])   # Down

        self.vel_pub.publish(vel_msg)

        self.get_logger().info(
            f"\n🚀 속도 명령 (FRD 기준):"
            f"\n   x (forward) = {velocity_frd[1]:.3f}"
            f"\n   y (right)   = {-velocity_frd[2]:.3f}"
            f"\n   z (down)    = {velocity_frd[0]:.3f}"
            f"\n   depth       = {self.depth_z:.2f} m"
        )




    def pixel_error_to_velocity_frd(self, dx_pixel, dy_pixel, fx, fy, max_speed):
        x_norm = dx_pixel / fx   # 오른쪽 +
        y_norm = dy_pixel / fy   # 아래 +
        z_norm = 1.0             # 정면

        direction_frd = np.array([z_norm, x_norm, y_norm])
        direction_unit = direction_frd / np.linalg.norm(direction_frd)
        velocity = direction_unit * max_speed
        return velocity

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LandingControl())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
