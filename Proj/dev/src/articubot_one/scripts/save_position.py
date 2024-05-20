#!/usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

last_odom = None
times = 0
class NavigatorWithPositionSaving(Node):
    def __init__(self):
        super().__init__()
        self.current_pose = None
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10  # 필요에 따라 큐 크기를 조절하세요
        )
   def odom_callback(self, msg):
        # 오도메트리 메시지로부터 위치 정보 추출
        self.current_pose = msg.pose.pose

    def saveCurrentPose(self, pose):
        self.current_pose = pose

    def getCurrentPose(self):
        return self.current_pose

if __name__ == '__main__':
    odom_subscriber = NavigatorWithPositionSaving()

    navigator = BasicNavigator()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    pose = odom_subscriber.getCurrentPose()
    goal_pose.pose.position.x = pose.position.x
    goal_pose.pose.position.y = pose.position.y
    goal_pose.pose.orientation.w = pose.orientation.w

    navigator.goToPose(goal_pose)
