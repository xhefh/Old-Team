#!/usr/bin/env python3
import json
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# 1. 점자 버튼 신호를 기다리는 통신(Subscriber) 노드 클래스
class DestinationSubscriber(Node):
    def __init__(self):
        super().__init__('destination_subscriber')
        self.target_destination = None
        self.subscription = self.create_subscription(
            String,
            '/selected_destination',
            self.listener_callback,
            10)
        self.get_logger().info('대기 중... 점자 버튼 신호(/selected_destination)를 기다립니다.')

    def listener_callback(self, msg):
        self.get_logger().info(f'수신 완료! 입력된 목적지: "{msg.data}"')
        self.target_destination = msg.data

def load_waypoints(target_name):
    json_path = os.path.expanduser('~/waypoints.json')
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return data.get(target_name, [])
    except FileNotFoundError:
        print("❌ waypoints.json 파일을 찾을 수 없습니다.")
        return []

def main():
    rclpy.init()

    # 터미널 3에서 쏘는 신호를 받기 위해 노드 실행
    sub_node = DestinationSubscriber()
    
    # 신호(target_destination)가 들어올 때까지 무한 대기
    while rclpy.ok() and sub_node.target_destination is None:
        rclpy.spin_once(sub_node, timeout_sec=0.1)
        
    if not rclpy.ok():
        return
        
    # 신호가 들어오면 해당 목적지를 타겟으로 설정하고 수신 노드 종료
    target = sub_node.target_destination
    sub_node.destroy_node()

    # 2. 본격적인 자율주행(Nav2) 시작
    navigator = BasicNavigator()
    raw_waypoints = load_waypoints(target)

    if not raw_waypoints:
        print(f"❌ '{target}' 목적지의 좌표 정보가 waypoints.json에 없습니다!")
        rclpy.shutdown()
        return

    navigator.waitUntilNav2Active()

    waypoints = []
    for pt in raw_waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pt['x']
        pose.pose.position.y = pt['y']
        pose.pose.orientation.w = pt['w']
        waypoints.append(pose)

    print(f"🚀 [{target}] 목적지로 다중 Waypoint 자율주행을 시작합니다!")
    navigator.goThroughPoses(waypoints)

    # 3. 실시간 피드백 및 경로 이탈량 체크 (3주차 증빙용 뼈대)
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'남은 경유지 수: {feedback.number_of_poses_remaining}')
            
            # [수정된 프로젝트 계획서 반영 로직]
            # 피드백 수신 시 현재 위치와 목표 경로 간의 이탈량(Cross-track error) 계산 로직 (가설정)
            # (추후 실제 로봇 구동 시 odometry 데이터를 바탕으로 연산 수식 반영 예정)
            cross_track_error = 0.02  # 임시 가상 오차값 (m)
            
            if cross_track_error >= 0.1:
                print(f"⚠️ 경로 이탈 심각 감지 (오차: {cross_track_error}m) -> 모터 제동 유도 Topic(/brake_cmd) 발행 준비")

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f'🎉 [{target}] 모든 Waypoint 주행 완료!')
    elif result == TaskResult.CANCELED:
        print(f'⚠️ [{target}] 주행이 취소되었습니다.')
    elif result == TaskResult.FAILED:
        print(f'❌ [{target}] 주행 실패!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
