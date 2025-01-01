import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
import numpy as np
import cv2

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')

        # QoS 설정
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # 맵 구독
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',  # 맵 토픽 이름
            self.map_callback,
            10
        )

        # 로봇 위치 구독
        self.subscription_pose = self.create_subscription(
            Odometry,  # 로봇 위치 데이터
            '/odom',   # Odometry 토픽 이름
            self.pose_callback,
            qos  # QoS 프로파일
        )

        # 이미지 포즈 구독
        self.subscription_image_pose = self.create_subscription(
            String,  # 포즈 데이터를 String 타입으로 구독
            '/imagepose',  # 토픽 이름
            self.image_callback,
            qos  # QoS 프로파일
        )

        self.map_info = None
        self.robot_position = None
        self.colored_data = None
        self.image_positions = []  # 여러 개의 이미지 위치 저장

    def map_callback(self, msg):
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        # 점유 상태를 0~255로 변환 (-1: 알 수 없음 -> 127)
        data = np.where(data == -1, 127, data)  # -1은 회색으로 처리
        data = (255 - (data * 255 // 100)).astype(np.uint8)  # 0: 흰색, 100: 검정색

        # y축 반전 (OpenCV와 RViz 시각화를 일치시키기 위해)
        data = np.flipud(data)  # y축 반전


        # 그레이스케일 맵을 컬러맵으로 변환
        self.colored_data = cv2.cvtColor(data, cv2.COLOR_GRAY2BGR)

        # 맵 확대 배율
        zoom_factor = 1.5
        resized_map = cv2.resize(self.colored_data, None, fx=zoom_factor, fy=zoom_factor, interpolation=cv2.INTER_NEAREST)

        # 로봇 위치 표시
        if self.robot_position and self.map_info:
            robot_pixel = self.world_to_pixel(self.robot_position, self.map_info)
            zoomed_robot_pixel = (int(robot_pixel[0] * zoom_factor), int(robot_pixel[1] * zoom_factor))

            if 0 <= zoomed_robot_pixel[1] < resized_map.shape[0] and 0 <= zoomed_robot_pixel[0] < resized_map.shape[1]:
                cv2.circle(resized_map, zoomed_robot_pixel, 7, (255, 0, 0), -1)  # 파란색 점

        # 이미지 포즈 표시
        for label, position in self.image_positions:
            if self.map_info:
                image_pixel = self.world_to_pixel(position, self.map_info)
                zoomed_image_pixel = (int(image_pixel[0] * zoom_factor), int(image_pixel[1] * zoom_factor))

                if 0 <= zoomed_image_pixel[1] < resized_map.shape[0] and 0 <= zoomed_image_pixel[0] < resized_map.shape[1]:
                    color = (0, 255, 0) if label == 'human' else (0, 0, 255)  # 초록색 또는 빨간색 점
                    cv2.circle(resized_map, zoomed_image_pixel, 7, color, -1)

        # OpenCV로 맵을 시각화
        cv2.imshow('TurtleBot4 Map', resized_map)
        cv2.waitKey(10)

    def world_to_pixel(self, position, map_info):
        resolution = map_info.resolution
        origin = map_info.origin.position

        # 좌표 변환
        x_pixel = int((position[0] - origin.x) / resolution)
        y_pixel = int((position[1] - origin.y) / resolution)

        # y축 반전 처리 (OpenCV와 RViz 좌표계 일치)
        y_pixel = map_info.height - y_pixel

        return (x_pixel, y_pixel)

    def pose_callback(self, msg):
        # 로봇의 위치를 저장 (월드 좌표)
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # self.get_logger().info(f"Received image position: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}")

    def image_callback(self, msg):
        try:
            # label과 x, y 값을 파싱하여 이미지 좌표로 저장
            parts = msg.data.split(',')
            label = parts[0]
            x = float(parts[1].split(':')[1])
            y = float(parts[2].split(':')[1])
            self.image_positions.append((label, (x, y)))
            self.get_logger().info(f"Received image position: label={label}, x={x}, y={y}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse image position: {e}")


def main(args=None):
    rclpy.init(args=args)
    visualizer = MapVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
