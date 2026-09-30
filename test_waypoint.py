#!/usr/bin/env python3
import json
import os
import sys
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def load_waypoints(target_name):
    json_path = os.path.expanduser('~/waypoints.json')
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return data.get(target_name, [])

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # 터미널 인자로 목적지 전달받음 (기본값: 'toilet')
    target = sys.argv[1] if len(sys.argv) > 1 else 'toilet'
    
    raw_waypoints = load_waypoints(target)

    if not raw_waypoints:
        print(f"❌ '{target}' 목적지의 좌표 정보가 waypoints.json에 없습니다.")
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

    print(f"🚀 [{target}] 목적지로 다중 Waypoint 자율주행을 시작합니다...")
    navigator.goThroughPoses(waypoints)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'남은 경유지 수: {feedback.number_of_poses_remaining}')

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