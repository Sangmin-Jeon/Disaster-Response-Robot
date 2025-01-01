import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
class PnPProcessor(Node):
    def __init__(self):
        super().__init__('pnp_processor')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.image_sub = self.create_subscription(
            CompressedImage, # 임포트 된 메시지 타입 
            '/oakd/rgb/preview/image_raw/compressed', # 토픽리스트에서 조회한 토픽 주소
            self.image_callback, # 정의한 콜백함수
            10)
        self.image = np.empty(shape=[1])
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # 오도메트리 토픽 이름
            self.odom_callback,
            qos_profile
        )
        self.bridge = CvBridge()
        self.robot_position = (0.0, 0.0)  # 초기 로봇 위치 (x, y)

        # 실제 이미지 파일 경로
        self.real_image_paths = [
            "ext_orig.png",
            "human_orig.png"  # 23x18 실제 이미지 (흑백 변환 필요)
                 # 18x18 실제 이미지 (흑백 변환 필요)
        ]

        # SIFT 생성
        self.sift = cv2.SIFT_create()
        self.real_kp_des = self.load_and_process_real_images(self.real_image_paths)

        # BFMatcher 생성
        self.bf = cv2.BFMatcher()

    def load_and_process_real_images(self, paths):
        keypoints_descriptors = {}
        for path in paths:
            real_img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if real_img is None:
                self.get_logger().error(f"Failed to load real image: {path}")
                continue
            kp, des = self.sift.detectAndCompute(real_img, None)
            keypoints_descriptors[path] = (kp, des)
        return keypoints_descriptors

    def odom_callback(self, msg):
        # 오도메트리 메시지에서 로봇의 x, y 좌표를 업데이트
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
# d:
# - -2.6200926303863525
# - -38.589866638183594
# - -0.0010925641981884837
# - 0.00021615292644128203
# - 262.2897644042969
# - -2.7986717224121094
# - -36.708839416503906
# - 255.18719482421875
        
##k:
# - 202.6661376953125
# - 0.0
# - 123.86566162109375
# - 0.0
# - 202.6661376953125
# - 124.75257873535156
# - 0.0
# - 0.0
# - 1.0

    def image_callback(self, msg):
        start_time = time.time()
        try:
            # ROS2 Image 메시지를 OpenCV 이미지로 변환 
            frame = self.bridge.compressed_imgmsg_to_cv2(msg)

            # 프레임을 흑백으로 변환
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            K= np.array([
                    [202.6661376953125, 0.0, 123.86566162109375],
                    [0.0, 202.6661376953125, 124.75257873535156],
                    [0.0, 0.0, 1.0]
                ])
            D= np.array([
                    -2.6200926303863525, -38.589866638183594, -0.0010925641981884837, 0.00021615292644128203,
                    262.2897644042969, -2.7986717224121094, -36.708839416503906, 255.18719482421875
                ])
            undistorted_image = cv2.undistort(gray_frame, K, D)
            
            
            # 카메라 이미지에서 SIFT 특징 추출
            h, w = gray_frame.shape[:2]  # 입력 이미지의 크기
            image_size = (w, h)          # OpenCV는 (width, height) 순서로 필요
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(K, D, image_size, 1, image_size)

            # 왜곡 보정된 이미지 생성
            undistorted_image = cv2.undistort(gray_frame, K, D, None, new_camera_matrix)

            # SIFT 특징 추출
            kp2, des2 = self.sift.detectAndCompute(undistorted_image, None)
            
            
            for real_path, (kp1, des1) in self.real_kp_des.items():
                # 매칭
                matches = self.bf.knnMatch(des1, des2, k=2)

                # 좋은 매칭 필터링
                good_matches = []
                for m, n in matches:
                    if m.distance < 0.70 * n.distance:
                        good_matches.append(m)
                if  "ext"  in real_path:
                    match_threshold = 10  # human의 임계값
                elif "human" in real_path:
                    match_threshold = 40  # fire의 임계값
                else:
                    self.get_logger().info(f"Unknown object type for {real_path}, skipping.")
                    continue

                # 매칭 결과 확인
                if len(good_matches) < match_threshold:
                    self.get_logger().info(
                        f"Not enough matches for {real_path} (found {len(good_matches)}, required {match_threshold})"
                    )
                    continue

                # 실제 이미지와 카메라 이미지의 매칭 좌표 추출
                # 실제 이미지와 카메라 이미지의 매칭 좌표 추출
                obj_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                src_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                # obj_pts를 명확히 2차원 배열로 변환
                obj_pts_reshaped = obj_pts.reshape(-1, 2)  # (N, 2) 형태로 변환
                src_pts_reshaped = src_pts.reshape(-1, 2)  # 백터 변환을 위해 정규화 (예: 1/100 스케일)

                # 실제 크기 정의 (센티미터 단위)
                real_sizes = {
                    "human": (0.18, 0.23),  # height, width (cm)
                    "ext": (0.18, 0.18)    # height, width (cm)
                }

                # 객체의 크기에 따라 obj_pts를 스케일링하여 3D 좌표 생성
                if "human" in real_path:
                    object_size_m = real_sizes["human"]
                elif "ext" in real_path:
                    object_size_m = real_sizes["ext"]
                else:
                    self.get_logger().info(f"Unknown object type for {real_path}, skipping.")
                    continue

                object_height_pixels = max(obj_pts_reshaped[:, 1]) - min(obj_pts_reshaped[:, 1])
                object_width_pixels = max(obj_pts_reshaped[:, 0]) - min(obj_pts_reshaped[:, 0])

                # 센티미터 단위를 픽셀 단위로 변환
                scale_x = object_size_m[1] / object_width_pixels  # 실제 너비 대비 픽셀 너비
                scale_y = object_size_m[0] / object_height_pixels  # 실제 높이 대비 픽셀 높이

                # 실제 크기를 반영한 3D 좌표 생성
                scaled_obj_pts = obj_pts_reshaped * np.array([scale_x, scale_y])  # 스케일링
                obj_pts_3d = np.hstack((scaled_obj_pts, np.zeros((scaled_obj_pts.shape[0], 1)))).astype(np.float32)  # Z 축 추가

                # PnP 계산 전에 크기 확인
                if obj_pts_3d.shape[0] != src_pts_reshaped.shape[0]:
                    self.get_logger().error(f"Mismatch in points: obj_pts_3d({obj_pts_3d.shape[0]}) vs src_pts_reshaped({src_pts_reshaped.shape[0]})")
                    continue
                # PnP 계산

                try:
                    retval, rvec, tvec, inliers = cv2.solvePnPRansac(obj_pts_3d, src_pts_reshaped, new_camera_matrix, None)
                    # 결과 출력
                    R, _ = cv2.Rodrigues(rvec)
                    self.get_logger().info(f"Results for {real_path}:")
                    self.get_logger().info(f"Rotation Matrix:\n{R}")
                    self.get_logger().info(f"Translation Vector: {tvec.T}")

                    # 로봇 위치와 조합하여 세계 좌표 계산
                    robot_position_vector = np.array([[self.robot_position[0]],  # x
                                                    [self.robot_position[1]],  # y
                                                    [0.0]])                   # z (평면 가정)
                    # 조정된 좌표 계산 (행렬 곱)
                    adjusted_coords_vector = np.dot(R,robot_position_vector) +  tvec
                    # 최종 조정된 좌표
                    adjusted_coords = (
                        adjusted_coords_vector[0][0],  # X 좌표
                        adjusted_coords_vector[1][0],  # Y 좌표
                    )
                    print(robot_position_vector)
                    print(tvec)
                    # 로봇과 감지된 물체 간 거리 계산
                    distance = np.sqrt((adjusted_coords[0] - self.robot_position[0])**2 + (adjusted_coords[1] - self.robot_position[1])**2)
                    
                    if distance >= 5:
                        self.get_logger().info(f"Object at {adjusted_coords} is too far (distance: {distance}), skipping detection.")
                        continue
                                        # 감지된 데이터 전송
                    qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
                    object_type = "human" if "human" in real_path else "ext" if "ext_orig" in real_path else None
                    if object_type is None:
                        self.get_logger().info(f"Unknown object type for {real_path}, skipping.")
                        continue
                    qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
                    publisher_pose = self.create_publisher(String, '/imagepose', qos_profile)
                    pose_message = f"{object_type},x:{adjusted_coords[0]:.2f},y:{adjusted_coords[1]:.2f}"
                    publisher_pose.publish(String(data=pose_message))
                    self.get_logger().info(f"Published Pose: {pose_message}")
                    # # 감지된 데이터 전송
                    # qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
                    # if object_type == "human" and not hasattr(self, 'human_detected'):
                    #     self.human_detected = True
                    #     publisher_pose = self.create_publisher(String, '/imagepose', qos_profile)
                    #     pose_message = f"{object_type},x:{adjusted_coords[0]:.2f},y:{adjusted_coords[1]:.2f}"
                    #     publisher_pose.publish(String(data=pose_message))
                    #     self.get_logger().info(f"Published Pose: {pose_message}")
                    # elif object_type == "ext" and not hasattr(self, 'ext_detected'):
                    #     self.ext_detected = True
                    #     publisher_pose = self.create_publisher(String, '/imagepose', qos_profile)
                    #     pose_message = f"{object_type},x:{adjusted_coords[0]:.2f},y:{adjusted_coords[1]:.2f}"
                    #     #pose_message = f"{object_type},x:0,y:0"
                    #     publisher_pose.publish(String(data=pose_message))
                    #     self.get_logger().info(f"Published Pose: {pose_message}")
                                    
                    
                   # self.get_logger().info(f"Adjusted World Coordinates: {adjusted_coords} (distance: {distance})")
                   # self.get_logger().info(f"Adjusted World Coordinates: {adjusted_coords}")

                except cv2.error as e:
                    self.get_logger().error(f"PnP failed for {real_path}: {e}")

            # OpenCV 이미지 출력
            cv2.imshow("Camera", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
        finally:
            elapsed_time = time.time() - start_time
            time_to_wait = max(0.5 - elapsed_time, 0)
            time.sleep(time_to_wait)

def main(args=None):
    rclpy.init(args=args)
    pnp_processor = PnPProcessor()

    try:
        rclpy.spin(pnp_processor)
    except KeyboardInterrupt:
        pass

    pnp_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







                # obj_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                # src_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                # # obj_pts를 명확히 2차원 배열로 변환
                # obj_pts_reshaped = obj_pts.reshape(-1, 2)  # (N, 2) 형태로 변환
                # src_pts_reshaped = src_pts.reshape(-1, 2) / 100.0
                # # 실제 크기 정의 (센티미터 단위)
                # real_sizes = {
                #     "human": (0.18, 0.23),  # height, width (cm)
                #     "fire": (0.18, 0.18)    # height, width (cm)
                # }

                # # 객체의 크기에 따라 obj_pts를 스케일링하여 3D 좌표 생성
                # if "human" in real_path:
                #     object_size_cm = real_sizes["human"]
                # elif "fire" in real_path:
                #     object_size_cm = real_sizes["fire"]
                # else:
                #     self.get_logger().info(f"Unknown object type for {real_path}, skipping.")
                #     continue
                # object_height_pixels = max(obj_pts_reshaped[:, 1]) - min(obj_pts_reshaped[:, 1])
                # object_width_pixels = max(obj_pts_reshaped[:, 0]) - min(obj_pts_reshaped[:, 0])

                # # 센티미터 단위를 픽셀 단위로 변환
                # scale_x = object_size_cm[1] / object_width_pixels  # 실제 너비 대비 픽셀 너비
                # scale_y = object_size_cm[0] / object_height_pixels  # 실제 높이 대비 픽셀 높이
                # # 실제 크기를 반영한 3D 좌표 생성
                # scaled_obj_pts = obj_pts_reshaped * np.array([scale_x, scale_y])  # 스케일링
                
                # scaled_obj_pts = scaled_obj_pts.reshape(-1, 2) # 명확히 (N, 2) 형태 유지
                # obj_pts_3d = np.hstack((scaled_obj_pts, np.zeros((scaled_obj_pts.shape[0], 1)))).astype(np.float32)

                # # PnP 계산 전에 크기 확인
                # if obj_pts_3d.shape[0] != src_pts_reshaped.shape[0]:
                #     self.get_logger().error(f"Mismatch in points: obj_pts_3d({obj_pts_3d.shape[0]}) vs src_pts_reshaped({src_pts_reshaped.shape[0]})")
                #     continue