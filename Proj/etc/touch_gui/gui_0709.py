import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped  # PoseStamped 임포트 추가

class ControlPanel(QWidget):
    def __init__(self, node, parent=None):
        super(ControlPanel, self).__init__(parent)
        self.node = node

        # 메인 레이아웃 설정
        main_layout = QHBoxLayout()  # 전체 레이아웃을 가로로 설정
        left_layout = QVBoxLayout()  # 왼쪽 레이아웃을 세로로 설정

        # 맵을 표시할 공간 설정 (현재는 예제이므로 QLabel로 대체)
        self.map_label = QLabel("Map Area")
        self.map_label.setStyleSheet("background-color: lightgray;")
        left_layout.addWidget(self.map_label)  # 왼쪽 레이아웃에 맵 영역 추가

        # 터미널 출력 영역 추가
        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("background-color: black; color: white;")
        left_layout.addWidget(self.terminal_output)

        # 오른쪽 버튼 레이아웃 (세로로 배치)
        right_layout = QVBoxLayout()

        # 리프트 제어 그룹박스
        lift_group = QGroupBox("Lift Control")
        lift_layout = QVBoxLayout()
        self.lift_up_button = QPushButton("Lift Up")
        self.lift_up_button.setStyleSheet("font-size: 24px; height: 100px;")
        self.lift_down_button = QPushButton("Lift Down")
        self.lift_down_button.setStyleSheet("font-size: 24px; height: 100px;")
        lift_layout.addWidget(self.lift_up_button)
        lift_layout.addWidget(self.lift_down_button)
        lift_group.setLayout(lift_layout)
        right_layout.addWidget(lift_group)

        # 네비게이션 그룹박스
        nav_group = QGroupBox("Navigation")
        nav_layout = QVBoxLayout()
        self.toggle_nav_button = QToolButton()
        self.toggle_nav_button.setCheckable(True)
        self.toggle_nav_button.setText("Set Navigation Goal")
        self.toggle_nav_button.setStyleSheet("font-size: 24px; height: 100px;")
        nav_layout.addWidget(self.toggle_nav_button)
        nav_group.setLayout(nav_layout)
        right_layout.addWidget(nav_group)

        # 로봇 상태 표시 그룹박스
        status_group = QGroupBox("Robot Status")
        status_layout = QVBoxLayout()
        status_box_layout = QHBoxLayout()
        self.status_color_label = QLabel()
        self.status_color_label.setFixedSize(30, 30)
        self.status_color_label.setStyleSheet("background-color: black;")
        self.status_info_label = QLabel("No Signal")
        self.status_info_label.setStyleSheet("font-size: 18px;")
        status_box_layout.addWidget(self.status_color_label)
        status_box_layout.addWidget(self.status_info_label)
        status_layout.addLayout(status_box_layout)
        status_group.setLayout(status_layout)
        right_layout.addWidget(status_group)

        # 왼쪽, 오른쪽 레이아웃을 메인 레이아웃에 추가
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

        self.setLayout(main_layout)  # 위젯에 메인 레이아웃 설정

        # 버튼 클릭 이벤트 연결
        self.lift_up_button.pressed.connect(self.start_lift_up)  # 리프트 올리기 버튼 누름 이벤트 연결
        self.lift_up_button.released.connect(self.stop_lift)  # 리프트 올리기 버튼 놓음 이벤트 연결
        self.lift_down_button.pressed.connect(self.start_lift_down)  # 리프트 내리기 버튼 누름 이벤트 연결
        self.lift_down_button.released.connect(self.stop_lift)  # 리프트 내리기 버튼 놓음 이벤트 연결
        self.toggle_nav_button.clicked.connect(self.toggle_navigation)  # 네비게이션 토글 버튼 클릭 이벤트 연결

        # ROS 퍼블리셔 및 구독자 설정
        self.lift_pub = self.node.create_publisher(String, '/lift_command', 10)  # 리프트 명령 퍼블리셔
        self.nav_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)  # 네비게이션 명령 퍼블리셔
        self.status_sub = self.node.create_subscription(String, '/robot_status', self.update_status, 10)  # 로봇 상태 구독자
        self.velocity_sub = self.node.create_subscription(Odometry, '/odom', self.update_velocity, 10)  # 로봇 속도 구독자
        self.imu_sub = self.node.create_subscription(Imu, '/imu', self.update_imu, 10)  # IMU 데이터 구독자
        self.slam_sub = self.node.create_subscription(Float32, '/slam_remaining_distance', self.update_slam, 10)  # SLAM 거리 구독자

        # 상태 정보 초기화
        self.velocity = None
        self.imu_orientation = None
        self.slam_distance = None
        self.eta = None

        # QTimer 설정
        self.lift_timer = QTimer()
        self.lift_timer.timeout.connect(self.send_lift_command)  # 타이머 타임아웃 시 명령 전송
        self.current_lift_command = None  # 현재 리프트 명령 초기화

    def start_lift_up(self):
        self.log_to_terminal("Start Lift Up")
        self.current_lift_command = "up"
        self.lift_timer.start(100)

    def start_lift_down(self):
        self.log_to_terminal("Start Lift Down")
        self.current_lift_command = "down"
        self.lift_timer.start(100)

    def stop_lift(self):
        self.log_to_terminal("Stop Lift")
        self.lift_timer.stop()

    def send_lift_command(self):
        if self.current_lift_command:
            msg = String()
            msg.data = self.current_lift_command
            self.lift_pub.publish(msg)

    def toggle_navigation(self):
        nav_state = "enabled" if self.toggle_nav_button.isChecked() else "disabled"
        self.log_to_terminal(f"Navigation {nav_state}")

    def update_status(self, msg):
        status = msg.data
        self.log_to_terminal(f"Update Status: {status}")
        if status == "normal":
            self.status_color_label.setStyleSheet("background-color: green;")
            self.status_info_label.setText(f"Velocity: {self.velocity if self.velocity is not None else ''}\n"
                                           f"IMU: {self.imu_orientation if self.imu_orientation is not None else ''}\n"
                                           f"SLAM: {self.slam_distance if self.slam_distance is not None else ''}\n"
                                           f"ETA: {self.eta if self.eta is not None else ''}")
        elif status == "emergency":
            self.status_color_label.setStyleSheet("background-color: red;")
            self.status_info_label.setText("Emergency Stop")
        else:
            self.status_color_label.setStyleSheet("background-color: black;")
            self.status_info_label.setText("No Signal")

    def update_velocity(self, msg):
        self.velocity = msg.twist.twist.linear.x
        self.log_to_terminal(f"Update Velocity: {self.velocity}")

    def update_imu(self, msg):
        self.imu_orientation = msg.orientation.z
        self.log_to_terminal(f"Update IMU: {self.imu_orientation}")

    def update_slam(self, msg):
        self.slam_distance = msg.data
        self.log_to_terminal(f"Update SLAM: {self.slam_distance}")
        self.eta = self.calculate_eta()

    def calculate_eta(self):
        if self.velocity and self.slam_distance:
            return self.slam_distance / self.velocity
        return None

    def log_to_terminal(self, message):
        self.terminal_output.append(message)
        self.terminal_output.ensureCursorVisible()

class MainWindow(QMainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Robot Control Panel")

        # 전체 화면 크기를 감지하여 설정
        screen_geometry = QApplication.primaryScreen().geometry()  # 화면 해상도 감지
        available_geometry = QApplication.primaryScreen().availableGeometry()  # 사용 가능한 화면 해상도 감지
        screen_width = int(available_geometry.width() * 23 / 24) # 화면 너비
        screen_height = int(available_geometry.height() * 23 / 24)  # 상단바를 제외한 화면 높이
        self.setGeometry(0, 0, screen_width, screen_height)  # 전체 화면 크기로 설정

        # Control Panel 추가
        self.control_panel = ControlPanel(node, self)
        
        # 메인 레이아웃 설정
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.control_panel)  # 컨트롤 패널을 메인 레이아웃에 추가

        container = QWidget()
        container.setLayout(main_layout)  # 컨테이너에 메인 레이아웃 설정
        self.setCentralWidget(container)  # 중앙 위젯으로 컨테이너 설정

        # 초기 맵 크기 및 터미널 출력 크기 설정
        self.control_panel.map_label.setFixedSize(int(screen_width * 2 / 3), int(screen_height * 2 / 3))
        self.control_panel.terminal_output.setFixedSize(int(screen_width * 2 / 3), int(screen_height * 1 / 3))
        self.control_panel.setFixedSize(screen_width, screen_height)

def main(args=None):
    rclpy.init(args=args)
    node = Node("robot_control_panel")  # ROS 2 노드 초기화

    app = QApplication(sys.argv)  # QApplication 초기화
    main_window = MainWindow(node)  # 메인 윈도우 생성
    main_window.show()  # 메인 윈도우 표시

    try:
        sys.exit(app.exec_())  # QApplication 이벤트 루프 실행
    finally:
        rclpy.shutdown()  # ROS 2 종료

if __name__ == "__main__":
    main()
