import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QIcon
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32, Float32
from threading import Thread
import time

class ControlPanel(QWidget):
    def __init__(self, node, parent=None):
        super(ControlPanel, self).__init__(parent)
        self.node = node  # ROS 노드를 저장하는 변수
        # self.setup_serial_connection('/dev/ttyACM0', 115200)  # 시리얼 연결 설정
        # self.init_ui()  # UI 컴포넌트 초기화

        # 메인 레이아웃 설정
        main_layout = QHBoxLayout()  # 전체 레이아웃을 가로로 설정
        left_layout = QVBoxLayout()  # 왼쪽 레이아웃을 세로로 설정

        # 맵을 표시할 공간 설정 (현재는 예제이므로 QLabel로 대체)
        self.map_label = QLabel("Map Area")
        self.map_label.setStyleSheet("background-color: lightgray;")
        left_layout.addWidget(self.map_label)  # 왼쪽 레이아웃에 맵 영역 추가

        # 터미널 출력 영역 추가
        left_bottom_layout = QHBoxLayout()

        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("background-color: black; color: white;")
        left_bottom_layout.addWidget(self.terminal_output)

        # 방향 제어를 위한 그룹 박스 추가
        move_control_group = QGroupBox("Movement Control")
        move_layout = QVBoxLayout()
        self.up_button = QPushButton(QIcon("up_arrow.png"), "") # 상, 하, 좌, 우 버튼 추가
        self.down_button = QPushButton(QIcon("down_arrow.png"), "")
        self.left_button = QPushButton(QIcon("left_arrow.png"), "")
        self.right_button = QPushButton(QIcon("right_arrow.png"), "")
        self.up_button.clicked.connect(lambda: self.send_movement_command("up"))
        self.down_button.clicked.connect(lambda: self.send_movement_command("down"))
        self.left_button.clicked.connect(lambda: self.send_movement_command("left"))
        self.right_button.clicked.connect(lambda: self.send_movement_command("right"))
        move_layout.addWidget(self.up_button)
        move_layout.addWidget(self.down_button)
        move_layout.addWidget(self.left_button)
        move_layout.addWidget(self.right_button)
        move_control_group.setLayout(move_layout)
        left_bottom_layout.addWidget(move_control_group)

        left_layout.addLayout(left_bottom_layout)  # 왼쪽 레이아웃에 방향 제어 그룹 추가

        # 오른쪽 버튼 레이아웃 (세로로 배치)
        right_layout = QVBoxLayout()

        # 리프트 제어 그룹박스
        lift_group = QGroupBox("Lift Control")
        lift_layout = QHBoxLayout() # 리프트 제어 버튼들 가로

        # 레이아웃 조정하여 사전 설정 높이 버튼 추가
        preset_heights_group = QGroupBox("Preset Heights")  # 사전 설정 높이를 위한 새 그룹 생성
        preset_heights_layout = QVBoxLayout()  # 사전 설정 높이 버튼을 위한 세로 레이아웃
        # 세 개의 사전 설정 높이 버튼 생성
        self.height1_button = QPushButton("1 Height")
        self.height2_button = QPushButton("2 Height")
        self.height3_button = QPushButton("3 Height")
        self.height1_button.setStyleSheet("font-size: 18px; height: 50px;") # 버튼 스타일 설정
        self.height2_button.setStyleSheet("font-size: 18px; height: 50px;")
        self.height3_button.setStyleSheet("font-size: 18px; height: 50px;")
        preset_heights_layout.addWidget(self.height1_button) # 레이아웃에 버튼 추가
        preset_heights_layout.addWidget(self.height2_button)
        preset_heights_layout.addWidget(self.height3_button)
        preset_heights_group.setLayout(preset_heights_layout) # 그룹에 레이아웃 설정
        lift_layout.addWidget(preset_heights_group) # 오른쪽 레이아웃에 사전 설정 높이 그룹 추가

        updown_group = QGroupBox("Lift Up/Down")  # 리프트 올리기, 내리기 버튼을 위한 새 그룹 생성
        updown_layout = QVBoxLayout()  # 리프트 올리기, 내리기 버튼을 위한 세로 레이아웃
        self.lift_up_button = QPushButton("Lift Up")
        self.lift_down_button = QPushButton("Lift Down")
        self.lift_up_button.setStyleSheet("font-size: 18px; height: 50px;")
        self.lift_down_button.setStyleSheet("font-size: 18px; height: 50px;")
        updown_layout.addWidget(self.lift_up_button)
        updown_layout.addWidget(self.lift_down_button)
        updown_group.setLayout(updown_layout)
        lift_layout.addWidget(updown_group)

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
        self.height1_button.clicked.connect(lambda: self.send_lift_command("L_20"))
        self.height2_button.clicked.connect(lambda: self.send_lift_command("L_21"))
        self.height3_button.clicked.connect(lambda: self.send_lift_command("L_22"))
        self.lift_up_button.clicked.connect(lambda: self.send_lift_command("L_10"))
        self.lift_down_button.clicked.connect(lambda: self.send_lift_command("L_11"))
        # self.lift_preset_button.clicked.connect(self.move_to_preset_height)
        self.lift_up_button.pressed.connect(self.start_lift_up)  # 리프트 올리기 버튼 누름 이벤트 연결
        self.lift_up_button.released.connect(self.stop_lift)  # 리프트 올리기 버튼 놓음 이벤트 연결
        self.lift_down_button.pressed.connect(self.start_lift_down)  # 리프트 내리기 버튼 누름 이벤트 연결
        self.lift_down_button.released.connect(self.stop_lift)  # 리프트 내리기 버튼 놓음 이벤트 연결
        self.toggle_nav_button.clicked.connect(self.toggle_navigation)  # 네비게이션 토글 버튼 클릭 이벤트 연결

        # ROS 퍼블리셔 및 구독자 설정
        self.emergency_pub = self.node.create_publisher(Int32, '/ems_sig', 10)  # 비상정지 시그널 퍼블리셔 (1 : 평시 상태, 0 : 비상정지 명령)
        self.lift_pub = self.node.create_publisher(String, '/lift_command', 10)  # 리프트 명령 퍼블리셔
        self.nav_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)  # 네비게이션 명령 퍼블리셔
        self.status_sub = self.node.create_subscription(String, '/robot_status', self.update_status, 10)  # 로봇 상태 구독자
        self.velocity_sub = self.node.create_subscription(Odometry, '/odom', self.update_velocity, 10)  # 로봇 속도 구독자
        self.imu_sub = self.node.create_subscription(Imu, '/imu', self.update_imu, 10)  # IMU 데이터 구독자
        self.slam_sub = self.node.create_subscription(Float32, '/slam_remaining_distance', self.update_slam, 10)  # SLAM 거리 구독자
        # self.start_serial_read_thread()  # 시리얼 읽기 스레드 시작

        # 상태 정보 초기화
        self.velocity = None
        self.imu_orientation = None
        self.slam_distance = None
        self.eta = None

        # QTimer 설정
        self.lift_timer = QTimer()
        # self.lift_timer.timeout.connect(self.send_lift_command)  # 타이머 타임아웃 시 명령 전송
        self.current_lift_command = None  # 현재 리프트 명령 초기화
    
    # def setup_serial_connection(self, port, baud_rate): #시리얼 포트 함수
    #     try:
    #         self.ser = serial.Serial(port, baud_rate, timeout=1)  # 시리얼 포트 오픈
    #         self.ser.reset_input_buffer()  # 입력 버퍼 초기화
    #         self.log_to_terminal(f"시리얼 ok")  # 연결 시 터미널에 로그 출력
    #     except serial.SerialException as e:
    #         self.log_to_terminal(f"시리얼 연결 오류: {str(e)}")  # 연결 오류가 있을 경우 터미널에 로그 출력
    #         self.ser = None
    
    # def init_ui(self):
    #     # 여기에 UI 컴포넌트 및 레이아웃 초기화 코드 추가
    #     pass

    # def start_serial_read_thread(self): # 시리얼 수신 스레드
    #     if self.ser:
    #         self.read_thread = Thread(target=self.read_from_serial)  # 시리얼 데이터 읽기를 위한 스레드 생성
    #         self.read_thread.start()  # 스레드 시작

    # def send_lift_command(self, command): # 시리얼 송신
    #     if self.ser:
    #         try:
    #             self.ser.write(f"{command}\n".encode('utf-8'))  # 아두이노로 명령어 전송
    #             self.log_to_terminal(f"Arduino Send: {command}")
    #         except serial.SerialException as e:
    #             self.log_to_terminal(f"Error Arduino Sending : {str(e)}")  # 명령 전송 실패시 로그 출력

    # def read_from_serial(self): # 시리얼 스레드 수신
    #     while True:
    #         if self.ser and self.ser.in_waiting > 0:
    #             line = self.ser.readline().decode('utf-8').rstrip()  # 시리얼로부터 데이터 읽기
    #             self.process_serial_data(line)  # 읽은 데이터 처리

    # def process_serial_data(self, data): # 시리얼 데이터 처리 (아두이노 수신 값)
    #     if data.startswith("E_"):
    #         status = int(data.split("_")[1])
    #         if status == 1:
    #             self.emergency_pub.publish(Int32(data=1))  # 정상 상태 발행
    #         elif status == 0:
    #             self.emergency_pub.publish(Int32(data=0))  # 비상 상태 발행
    #         self.log_to_terminal(f"Arduino received : {data}")

    def move_to_preset_height(self): # 미리 설정된 높이로 이동
        presets = ['L_20', 'L_21', 'L_22']
        for cmd in presets:
            self.send_lift_command(cmd) # 각 높이 설정 명령을 시리얼로 전송
            time.sleep(1)  # 임시로 설정된 대기 시간

    def start_lift_up(self): # lift up 함수
        self.log_to_terminal("Start Lift Up") # 터미널에 로그 출력
        self.current_lift_command = "up" # 현재 명령을 'down'으로 설정
        self.lift_timer.start(100) # 100ms 간격으로 lift_timer를 시작

    def start_lift_down(self): # lift down 함수
        self.log_to_terminal("Start Lift Down")
        self.current_lift_command = "down"
        self.lift_timer.start(100)

    def stop_lift(self): # lift stop 함수
        self.log_to_terminal("Stop Lift")
        self.current_lift_command = None # 현재 명령을 None으로 초기화
        self.lift_timer.stop()

    def toggle_navigation(self): # navigation toggle 함수
        nav_state = "enabled" if self.toggle_nav_button.isChecked() else "disabled"
        self.log_to_terminal(f"Navigation {nav_state}") # 터미널에 상태 로그 출력

    def update_status(self, msg): # ROS에서 상태 업데이트 메시지를 받아 처리하는 함수
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

    def update_velocity(self, msg): # Robot velocity 업데이트 함수
        self.velocity = msg.twist.twist.linear.x
        self.log_to_terminal(f"Update Velocity: {self.velocity}")

    def update_imu(self, msg): # IMU 업데이트 함수
        self.imu_orientation = msg.orientation.z
        self.log_to_terminal(f"Update IMU: {self.imu_orientation}")

    def update_slam(self, msg): # SLAM 거리 업데이트 함수
        self.slam_distance = msg.data
        self.log_to_terminal(f"Update SLAM: {self.slam_distance}")
        self.eta = self.calculate_eta()

    def calculate_eta(self): # ETA 계산 함수 (예상 도착 시간)
        if self.velocity and self.slam_distance:
            return self.slam_distance / self.velocity
        return None

    def send_movement_command(self, direction):
        # 여기에서 ROS 메시지를 통해 로봇 이동 명령을 보냅니다.
        # 이는 rclpy를 사용하여 구현됩니다. 예를 들어:
        msg = Twist()
        if direction == "up":
            msg.linear.x = 1.0
        elif direction == "down":
            msg.linear.x = -1.0
        elif direction == "left":
            msg.angular.z = 1.0
        elif direction == "right":
            msg.angular.z = -1.0
        self.velocity_publisher.publish(msg)

    def log_to_terminal(self, message):
        self.terminal_output.append(message)
        self.terminal_output.ensureCursorVisible()

class MainWindow(QMainWindow): # 터미널 로그를 추가하는 함수
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