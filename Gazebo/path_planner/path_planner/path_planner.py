import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SpawnEntity
import math
import random
import itertools
import time

class PathNode(Node):
    def __init__(self):
        super().__init__('path_node')

        # Nav2 액션 서버와 통신하기 위한 액션 클라이언트 생성
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.start = (0.0, 0.0)
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # 8개의 고정 좌표 (방 위치)
        self.room_positions = [
            (-4.0, 4.0), (-4.0, 0.5), (-4.0, -3.0), (0.1, -3.8),
            (3.85, -3.2), (4.0, 0.0), (4.0, 3.5), (-0.0, 4.0)
        ]
        # 새로운 방 위치 8
        self.already_room = [
            (-4.9, 4.9), (-4.9, -1.4), (-4.9, -4.5), (2.7, -4.5),
            (4.5, -4.5), (4.5, -1.4), (4.5, 4.9), (0.9, 4.9)
        ]

        self.colors = ['red', 'green', 'blue','grey','orange','pink','purple','yellow']  # 박스
        # 색상 → 좌표 매핑 딕셔너리
        self.color_to_pos = dict(zip(self.colors, self.room_positions))
        self.pos_to_color = {v: k for k, v in self.color_to_pos.items()}


        # 3개 색상 랜덤 선택
        self.selected_colors = random.sample(self.colors, 3)
        # 색상에 해당하는 좌표 추출
        self.selected_positions = [self.color_to_pos[c] for c in self.selected_colors]

        self.path, self.total_cost = self.compute_optimal_path(self.start, self.selected_positions)
        self.index = 1
        self.spawn_all_boxes()
        for c in self.selected_colors:
            pos = self.color_to_pos[c]
            self.get_logger().info(f"선택된 박스: {c.upper()} → 위치: {pos}")
        route_info = []
        for i, pos in enumerate(self.path):
            if i == 0:
                label = "시작"
            elif i == len(self.path) - 1:
                label = "복귀"
            else:
                label = self.pos_to_color.get(pos, "unknown")
            route_info.append(f"  {i}. ({pos[0]:.2f}, {pos[1]:.2f}) - {label}")

        route_str = "\n".join(route_info)
        self.get_logger().info(f"최적 경로:\n{route_str}\n(총 거리: {self.total_cost:.2f}m)")
        self.send_next_goal()

    def spawn_all_boxes(self):
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("spawn_entity 서비스 대기 중...")

        for color, (x, y) in zip(self.colors, self.already_room):
            sdf_path = f"/home/dev/turtlebot3_ws/src/turtlebot3_gazebo/models/box_{color}/model.sdf"
            with open(sdf_path, 'r') as f:
                sdf_content = f.read()

            req = SpawnEntity.Request()
            req.name = f"init_box_{color}_{int(time.time())}"
            req.xml = sdf_content
            req.robot_namespace = ''
            req.initial_pose.position.x = x
            req.initial_pose.position.y = y
            req.initial_pose.position.z = 0.3
            req.reference_frame = 'map'

            self.spawn_client.call_async(req)
            time.sleep(0.2)  # 겹침 방지용 딜레이

    def distance(self, p1, p2):
        # 두 지점 사이의 유클리디안 거리 계산
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def compute_optimal_path(self, start, waypoints):
        best_order = None
        min_total = float('inf')

        # 모든 지점 순서에 대해 순열 생성 후 거리 계산
        for perm in itertools.permutations(waypoints):
            route = [start] + list(perm) + [start]  # 시작→지점들→다시 시작 위치
            total = sum(self.distance(route[i], route[i+1]) for i in range(len(route)-1))

            # 가장 짧은 거리의 경로 선택
            if total < min_total:
                min_total = total
                best_order = route

        return best_order, min_total

    def send_next_goal(self):
        # 경로 끝까지 도달하면 종료
        if self.index >= len(self.path):
            self.get_logger().info("모든 목표 완료.")
            return

        # 현재 목표 좌표 추출
        x, y = self.path[self.index]

        # 목표 메시지 생성
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose

        # 현재 목표 위치 로그 출력
        self.get_logger().info(f"[{self.index}] 이동 시작: ({x:.2f}, {y:.2f})")

        # 액션 서버 준비될 때까지 대기
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 액션 서버 대기 중...')

        # 비동기로 목표 전송
        future = self.client.send_goal_async(goal_msg)

        # 전송 완료 후 응답 처리 콜백 등록
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('목표 거부됨')
            return

        self.get_logger().info('목표 수락됨')

        # 결과 도착을 기다리는 비동기 콜백 등록
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        if self.index >= len(self.path):
            self.get_logger().warn(f"[{self.index}] 경로 초과로 콜백 무시됨")
            return
        # 목표 도착 완료 처리
        x, y = self.path[self.index]
        pos = (x, y)

        # 색상 결정 (복귀는 따로 표시)
        if self.index == len(self.path) - 1:
            color = "복귀"
        else:
            color = self.pos_to_color.get(pos, 'unknown')

        self.get_logger().info(f"[{self.index}] 목표 도착 완료 → 위치: ({x:.2f}, {y:.2f}) | 색상: {color}")

        # 박스 3개만 소환 (복귀 지점에는 X)
        if self.index < len(self.path) - 1:
            self.spawn_box(x, y, color)
            self.get_logger().info(f"[{self.index}] {color} 박스 소환 at {pos}")

        self.index += 1
        time.sleep(1.0)
        self.send_next_goal()

    def spawn_box(self, x, y, color):
            while not self.spawn_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("spawn_entity 서비스 대기 중...")

            sdf_path = f"/home/dev/turtlebot3_ws/src/turtlebot3_gazebo/models/box_{color}/model.sdf"
            with open(sdf_path, 'r') as f:
                sdf_content = f.read()

            req = SpawnEntity.Request()
            req.name = f"box_{color}_{int(time.time())}"
            req.xml = sdf_content
            req.robot_namespace = ''
            offset = 1.0
            angle = math.atan2(
                self.path[self.index][1] - self.path[self.index - 1][1],
                self.path[self.index][0] - self.path[self.index - 1][0]
            )

            req.initial_pose.position.x = x + offset * math.cos(angle + math.pi/2)
            req.initial_pose.position.y = y + offset * math.sin(angle + math.pi/2)
            req.initial_pose.position.z = 0.3

            req.reference_frame = 'map'

            self.spawn_client.call_async(req)

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)

    # 노드 생성 및 실행
    node = PathNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
