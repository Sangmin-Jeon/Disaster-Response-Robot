import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from collections import deque
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.callback_group = ReentrantCallbackGroup()

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  
            durability=DurabilityPolicy.VOLATILE  
        )

        map_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  
            history=HistoryPolicy.KEEP_LAST,  
            depth=1  
        )

        # Odometry 구독 설정
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile, callback_group=self.callback_group)

        # 맵 데이터를 구독
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos_profile, callback_group=self.callback_group)

        self.marker_pub = self.create_publisher(Marker, 'visited_zones', 10, callback_group=self.callback_group)


        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.current_position = (0.0, 0.0)
        self.is_cal_coodrinate = False
        self.goal_in_progress = False  # 목표가 진행 중인지 여부를 추적

        # 방문한 구역을 저장할 집합 (구역 단위)
        self.visited_zones = set()

        # 구역 크기 설정 (예: 10x10 격자)
        self.zone_size = 10
        # 방문 범위를 설정 (예: 3x3)
        self.nearby_range = 3

    def map_callback(self, msg):
        # 맵 데이터 받기
        self.map_data = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        # self.get_logger().info('Received new map')
        # self.get_logger().info(f"map width: {self.map_width}, map height: {self.map_height}")

    def odom_callback(self, msg):
        # Odometry 메시지에서 로봇의 위치 추출
        position = msg.pose.pose.position
        self.current_position = (position.x, position.y)

        if not self.is_cal_coodrinate:
            self.is_cal_coodrinate = True
            # 로봇 위치 출력
            # self.get_logger().info(f"좌표 매핑 이전 robot position: x={position.x}, y={position.y}")

            # 맵에서의 로봇 위치 계산
            col, row = self.get_map_coordinates(position.x, position.y)


            if col == None and row == None:
                return

            # 방문한 구역 기록
            # self.record_visited_zone(col, row)

            farthest_zone = self.get_next_zone(col, row)

            self.send_goal(farthest_zone)

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

    def record_visited_zone(self, col, row):
        """방문한 구역을 기록하는 함수"""
        # map_resolution을 고려하여 구역의 크기를 실제 크기로 변환
        real_zone_size = self.zone_size * self.map_resolution

        # 구역의 좌표를 계산 (zone_size에 따라 나누기)
        zone_col = col // self.zone_size
        zone_row = row // self.zone_size
        visited_zone = (zone_row, zone_col)

        # 방문한 구역 기록
        # self.visited_zones.add(visited_zone)
        # self.get_logger().info(f"Visited zone: {visited_zone}")

        # 주변 n x n 범위 구역 방문 처리
        for d_row in range(-self.nearby_range, self.nearby_range + 1):
            for d_col in range(-self.nearby_range, self.nearby_range + 1):
                nearby_zone_row = zone_row + d_row
                nearby_zone_col = zone_col + d_col

                # 범위 내의 구역만 처리
                if 0 <= nearby_zone_row < self.map_height // self.zone_size and 0 <= nearby_zone_col < self.map_width // self.zone_size:
                    nearby_zone = (nearby_zone_row, nearby_zone_col)
                    self.visited_zones.add(nearby_zone)
                    # self.get_logger().info(f"Visited nearby zone: {nearby_zone}")
        
        # 방문한 구역을 Marker로 시각화
        #self.publish_visited_zone_marker(zone_row, zone_col)

        # 실제 물리적 구역 크기 출력
        # self.get_logger().info(f"Real zone size: {real_zone_size} meters")

    
    def publish_visited_zone_marker(self, zone_row, zone_col):
        """ 방문한 구역을 표시하는 마커를 퍼블리시하는 함수 """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.map_origin_x + zone_col * self.zone_size * self.map_resolution
        marker.pose.position.y = self.map_origin_y + zone_row * self.zone_size * self.map_resolution
        marker.pose.position.z = 0.1  # 위치를 살짝 올려서 표시
        marker.scale.x = self.zone_size * self.map_resolution
        marker.scale.y = self.zone_size * self.map_resolution
        marker.scale.z = 0.1  # 얇은 상자 형태로 표시

        # 수정된 부분: 색상 설정
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)  # 빨간색 반투명

        self.marker_pub.publish(marker)


    def get_next_zone(self, init_x, init_y):
        def cal_negone_grid(graph, zone):
            count_zero = 0
            in_negative_one = False
            n, m = 5, 5  # n과 m의 크기
            center_x, center_y = zone[0], zone[1]  # 중심 좌표
            
            # 중심을 기준으로 n * m 범위만큼 검사
            for i in range(center_x - n // 2, center_x + n // 2 + 1):
                for j in range(center_y - m // 2, center_y + m // 2 + 1):
                    # 그래프의 범위 내에 존재하는지 확인
                    if 0 <= i < len(graph) and 0 <= j < len(graph[0]):
                        if graph[i][j] == 100:
                            return False
                        if graph[i][j] == 0:
                            count_zero += 1
                        if graph[i][j] == -1:
                            in_negative_one = True

            if count_zero >= 7 and in_negative_one:
                return True
            return False
                

        """로봇 위치(init_x, init_y)에서 가장 멀리 떨어진 방문하지 않은 구역을 찾는 함수"""
        graph = [[self.map_data[row * self.map_width + col] for col in range(self.map_width)] for row in range(self.map_height)]

        # 탐색을 위한 큐 초기화
        queue = deque([(init_y, init_x, 0)])  # (row, col, distance)
        visited = [[False for _ in range(self.map_width)] for _ in range(self.map_height)]
        visited[init_y][init_x] = True

        # 가장 먼 구역을 찾기 위한 변수
        max_distance = -1
        farthest_zone = None

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while queue:
            current_row, current_col, distance = queue.popleft()

            # 현재 위치가 맵의 경계를 벗어나지 않는지 확인
            if not (0 <= current_row < self.map_height and 0 <= current_col < self.map_width):
                continue

            # 구역을 방문했으면
            if graph[current_row][current_col] == 0:
                zone_col = current_col // self.zone_size
                zone_row = current_row // self.zone_size
                zone = (zone_row, zone_col)

                # 해당 구역을 아직 방문하지 않았다면
                # if zone not in self.visited_zones:
                # 가장 멀리 있고 주변에 -1이 있는 경계 구역을 찾은 경우 업데이트
                is_boundary = False
                for dr, dc in directions:
                    new_row, new_col = zone_row + dr, zone_col + dc
                    # print(f'{len(graph[0])}, {new_row} : {len(graph)}, {new_col}')
                    if 0 <= new_row < len(graph[0]) and 0 <= new_col < len(graph) and graph[new_row][new_col] == -1:
                        is_boundary = True

                if is_boundary:
                    if cal_negone_grid(graph, zone): 
                        if distance > max_distance:
                            max_distance = distance
                            farthest_zone = zone  

            for dr, dc in directions:
                new_row, new_col = current_row + dr, current_col + dc

                # 새로운 위치가 visited 배열의 범위를 벗어나지 않도록
                if 0 <= new_row < len(visited) and 0 <= new_col < len(visited[0]) and not visited[new_row][new_col]:
                    visited[new_row][new_col] = True
                    queue.append((new_row, new_col, distance + 1))

        if farthest_zone:
            self.get_logger().info(f"가장 멀리 있는 구역: {farthest_zone}")
        else:
            self.get_logger().info("가장 멀리 있는 구역을 찾을 수 없습니다.")

        return farthest_zone



    def send_goal(self, farthest_zone):
        """가장 멀리 있는 구역으로 goal을 전송하는 함수"""
        if farthest_zone:
            zone_row, zone_col = farthest_zone
            # 구역의 (row, col)을 실제 좌표로 변환
            goal_x = self.map_origin_x + zone_col * self.zone_size * self.map_resolution
            goal_y = self.map_origin_y + zone_row * self.zone_size * self.map_resolution

            # 목표 위치 출력
            self.get_logger().info(f"목표 위치로 {goal_x}, {goal_y}를 설정했습니다.")
            
            # navigate_to_pose 액션 보내기
            self.send_navigation_goal(goal_x, goal_y)
        else:
            self.get_logger().info("가장 멀리 있는 구역을 찾을 수 없어서 목표를 설정할 수 없습니다.")

    def send_navigation_goal(self, x, y):
        """NavigateToPose 액션을 통해 목표를 로봇에 전달"""
        goal_msg = NavigateToPose.Goal()
        
        goal_msg.pose.header.frame_id = "map"  # 반드시 설정
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 추가
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # 회전 설정, 직진 이동만 할 경우

        self.goal_in_progress = True

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("액션 서버가 준비되지 않았습니다!")
            return

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.goal_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """액션 서버로부터 목표 수신 확인"""
        result = future.result()
        if result.accepted:
            self.get_logger().info("목표가 수락되었습니다.")
        else:
            self.get_logger().info("목표 수락이 거부되었습니다.")

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('목표 도달 완료!')
            self.is_cal_coodrinate = False

        if result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('목표 도달 실패, 목표 재설정 요청 합니다.')
            self.is_cal_coodrinate = False

    def goal_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if not self.is_cal_coodrinate:
            return

        self.get_logger().info(f'목표 도달 거리: {feedback.distance_remaining}')
        if feedback.distance_remaining <= 1.5:
            self.get_logger().info('목표 도달 완료!')
            self.is_cal_coodrinate = False

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
