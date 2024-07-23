# 전체 동작 및 설명:
# 이 코드는 ROS2와 PyQt5를 사용하여 로봇 제어 GUI를 구현합니다.
# /map 토픽에서 OccupancyGrid 메시지를 구독하여 맵을 표시하고, /odom 토픽에서 Odometry 메시지를 구독하여 로봇의 현재 위치를 지도 상에 표시합니다.
# 사용자가 GUI의 지도를 클릭하여 로봇의 목표 위치를 설정하면 /move_base_simple/goal 토픽에 PoseStamped 메시지를 발행합니다.

import sys  # 시스템 관련 모듈
import serial  # 시리얼 통신 모듈
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit, QGridLayout, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QGraphicsEllipseItem  # PyQt5 위젯
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QPointF  # PyQt5 핵심 모듈
from PyQt5.QtGui import QPixmap, QImage  # PyQt5 그래픽 모듈
import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드
from std_msgs.msg import String, Int32, Float32  # ROS2 표준 메시지 타입
from nav_msgs.msg import Odometry, OccupancyGrid  # 네비게이션 메시지 타입
from geometry_msgs.msg import PoseStamped, Twist  # 기하학적 메시지 타입
from threading import Thread  # 스레딩 모듈
import numpy as np  # 수치 연산 모듈

class ControlPanel(QWidget):  # 컨트롤 패널 클래스
    def __init__(self, node, parent=None):
        super(ControlPanel, self).__init__(parent)
        self.node = node  # ROS 노드

        self.init_ui()  # UI 초기화

        # ROS2 Subscribers and Publishers
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.update_robot_position, 10)  # 오도메트리 구독
        self.map_sub = self.node.create_subscription(OccupancyGrid, '/map', self.update_map, 10)  # 맵 구독
        self.goal_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)  # 목표 위치 발행

    def init_ui(self):  # UI 초기화 함수
        main_layout = QHBoxLayout()  # 메인 레이아웃
        self.setLayout(main_layout)  # 레이아웃 설정

        left_layout = QVBoxLayout()  # 왼쪽 레이아웃
        self.map_view = QGraphicsView()  # 맵 뷰
        self.map_scene = QGraphicsScene()  # 맵 씬
        self.map_view.setScene(self.map_scene)  # 맵 뷰에 씬 설정
        self.map_view.setRenderHint(QtGui.QPainter.Antialiasing)  # 앤티앨리어싱 설정
        left_layout.addWidget(self.map_view)  # 왼쪽 레이아웃에 맵 뷰 추가

        self.terminal_output = QTextEdit()  # 터미널 출력
        self.terminal_output.setReadOnly(True)  # 읽기 전용
        self.terminal_output.setStyleSheet("background-color: black; color: white;")  # 스타일 설정
        self.terminal_output.setFixedHeight(200)  # 고정 높이 설정
        left_layout.addWidget(self.terminal_output)  # 왼쪽 레이아웃에 터미널 출력 추가

        self.robot_marker = QGraphicsEllipseItem(-5, -5, 10, 10)  # 로봇 마커
        self.robot_marker.setBrush(Qt.red)  # 마커 색상 설정
        self.map_scene.addItem(self.robot_marker)  # 맵 씬에 마커 추가

        self.map_view.mousePressEvent = self.set_navigation_goal  # 마우스 클릭 이벤트 설정

        main_layout.addLayout(left_layout)  # 메인 레이아웃에 왼쪽 레이아웃 추가

        right_layout = QVBoxLayout()  # 오른쪽 레이아웃

        # Add other control buttons and status indicators here

        main_layout.addLayout(right_layout)  # 메인 레이아웃에 오른쪽 레이아웃 추가

    def update_robot_position(self, msg):  # 로봇 위치 업데이트 함수
        pos = msg.pose.pose.position
        self.robot_marker.setPos(pos.x * 100, pos.y * 100)  # 1미터를 100픽셀로 가정하여 위치 설정

    def update_map(self, msg):  # 맵 업데이트 함수
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))  # 맵 데이터 변환
        data = np.flipud(data)  # 상하 반전
        data = (data * 255).astype(np.uint8)  # 0-255로 스케일링
        qimg = QImage(data, width, height, QImage.Format_Grayscale8)  # QImage 생성
        pixmap = QPixmap.fromImage(qimg)  # QPixmap 생성
        if not hasattr(self, 'map_pixmap_item'):
            self.map_pixmap_item = QGraphicsPixmapItem(pixmap)  # 맵 픽스맵 아이템 생성
            self.map_scene.addItem(self.map_pixmap_item)  # 맵 씬에 추가
        else:
            self.map_pixmap_item.setPixmap(pixmap)  # 기존 픽스맵 업데이트

    def set_navigation_goal(self, event):  # 네비게이션 목표 설정 함수
        scene_pos = self.map_view.mapToScene(event.pos())  # 씬 좌표로 변환
        goal = PoseStamped()  # 목표 위치 메시지 생성
        goal.header.frame_id = 'map'
        goal.pose.position.x = scene_pos.x() / 100  # 다시 미터로 변환
        goal.pose.position.y = scene_pos.y() / 100
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)  # 목표 위치 발행
        self.log_to_terminal(f"Goal set to: x={goal.pose.position.x}, y={goal.pose.position.y}")  # 로그 출력

    def log_to_terminal(self, message):  # 터미널 로그 출력 함수
        self.terminal_output.append(message)  # 메시지 추가
        self.terminal_output.ensureCursorVisible()  # 커서 가시성 유지

class MainWindow(QMainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Robot Control Panel")  # 윈도우 타이틀 설정

        screen_geometry = QApplication.primaryScreen().geometry()  # 화면 해상도 가져오기
        screen_width = screen_geometry.width()
        screen_height = screen_geometry.height()

        window_width = int(screen_width * 0.9)  # 윈도우 너비 설정
        window_height = int(screen_height * 0.9)  # 윈도우 높이 설정
        self.setGeometry(0, 0, window_width, window_height)  # 윈도우 위치와 크기 설정

        self.control_panel = ControlPanel(node, self)  # 컨트롤 패널 생성

        main_layout = QVBoxLayout()  # 메인 레이아웃
        main_layout.addWidget(self.control_panel)  # 메인 레이아웃에 컨트롤 패널 추가

        container = QWidget()  # 컨테이너 위젯
        container.setLayout(main_layout)  # 컨테이너 레이아웃 설정
        self.setCentralWidget(container)  # 중앙 위젯 설정

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = Node("robot_control_panel")  # ROS 2 노드 생성
    app = QApplication(sys.argv)  # QApplication 생성
    main_window = MainWindow(node)  # 메인 윈도우 생성
    main_window.show()  # 메인 윈도우 표시

    try:
        sys.exit(app.exec_())  # QApplication 이벤트 루프 실행
    finally:
        rclpy.shutdown()  # ROS 2 종료

if __name__ == "__main__":
    main()  # 메인 함수 실행