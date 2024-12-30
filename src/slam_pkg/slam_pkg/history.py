import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from collections import deque


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')

        # ReentrantCallbackGroup을 사용하여 여러 콜백을 동시에 처리하도록 설정
        self.callback_group = ReentrantCallbackGroup()

        # Odometry 데이터를 구독하기 위한 QoS 설정
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT로 설정
            durability=DurabilityPolicy.VOLATILE  # DURABILITY 설정
        )

        # Odometry 구독 설정
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile, callback_group=self.callback_group)

        # 맵 데이터를 구독
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10, callback_group=self.callback_group)

        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.current_position = (0.0, 0.0)

    def map_callback(self, msg):
        # 맵 데이터 받기
        self.map_data = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.get_logger().info('Received new map')
        self.get_logger().info(f"map width: {self.map_width}, map height: {self.map_height}")

    def odom_callback(self, msg):
        # Odometry 메시지에서 로봇의 위치 추출
        position = msg.pose.pose.position
        self.current_position = (position.x, position.y)

        # 로봇 위치 출력
        self.get_logger().info(f"Current position: x={position.x}, y={position.y}")

        # 맵에서의 로봇 위치 계산
        col, row = self.get_map_coordinates(position.x, position.y)

        # 맵 좌표 출력
        self.get_logger().info(f"Robot is at map coordinates: row={row}, col={col}")

    def get_map_coordinates(self, robot_x, robot_y):
        """
        로봇의 (x, y) 좌표를 맵의 그리드 좌표 (row, col)로 변환
        """
        if self.map_resolution == 0:
            self.get_logger().error("Map resolution is zero! Cannot convert coordinates.")
            return None, None
            
        # 로봇 위치를 맵의 그리드 좌표로 변환
        col = int((robot_x - self.map_origin_x) / self.map_resolution)
        row = int((robot_y - self.map_origin_y) / self.map_resolution)

        # 맵의 크기를 초과하지 않도록 제한
        col = max(0, min(self.map_width - 1, col))
        row = max(0, min(self.map_height - 1, row))

        return col, row

    def shutdown(self):
        self.get_logger().info("Shutting down exploration.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()

    # MultiThreadedExecutor를 사용하여 여러 콜백을 동시에 실행
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin the node to execute callbacks in separate threads
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
