import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit, QGridLayout
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
        self.node = node  # ROS 노드를 저장
       
        # 초기 상태 변수 설정
        self.ser = None
        self.velocity = None
        self.imu_orientation = None
        self.slam_distance = None
        self.eta = None
        self.lift_timer = QTimer()
        self.current_lift_command = None  # 현재 리프트 명령 초기화

        self.init_ui()  # UI 초기화
       
        # UI 초기화 후에 로그 출력
        self.log_to_terminal("UI Set Success!")  # ROS 노드 초기화 로그 출력

        self.setup_serial_connection('/dev/ttyACM0', 115200)  # 시리얼 포트 연결 설정
        self.start_serial_read_thread()  # 시리얼 읽기 스레드 시작

        # ROS 통신 설정
        self.emergency_pub = self.node.create_publisher(Int32, '/ems_sig', 10)
        self.lift_pub = self.node.create_publisher(String, '/lift_command', 10)
        self.nav_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.status_sub = self.node.create_subscription(String, '/robot_status', self.update_status, 10)
        self.velocity_sub = self.node.create_subscription(Odometry, '/odom', self.update_velocity, 10)
        self.imu_sub = self.node.create_subscription(Imu, '/imu', self.update_imu, 10)
        self.slam_sub = self.node.create_subscription(Float32, '/slam_remaining_distance', self.update_slam, 10)

    def setup_serial_connection(self, port, baud_rate):  # 시리얼 연결 설정 함수
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1)  # 시리얼 포트 연결
            self.ser.reset_input_buffer()  # 입력 버퍼 리셋
            self.log_to_terminal(f"Serial Connected : {port} @ {baud_rate}")  # 연결 성공 로그 출력
        except serial.SerialException as e:
            self.log_to_terminal(f"Serial Connected Failed : {str(e)}")  # 연결 실패 로그 출력
            self.ser = None
   
    def init_ui(self):
        # 메인 레이아웃 설정
        main_layout = QHBoxLayout()  # 전체 레이아웃 수평
        self.setLayout(main_layout) # 메인 레이아웃 설정

        # 왼쪽 레이아웃 설정
        left_layout = QVBoxLayout() # 왼쪽 레이아웃 수직
        self.map_label = QLabel("Map Area") # 지도 영역 라벨 설정
        self.map_label.setStyleSheet("background-color: lightgray;")
        left_layout.addWidget(self.map_label)

        # 왼쪽 하단 레이아웃 설정
        # left_bottom_layout = QHBoxLayout() 
        self.terminal_output = QTextEdit() # 터미널 출력 영역 설정
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("background-color: black; color: white;")
        left_layout.addWidget(self.terminal_output)
        # left_bottom_layout.addWidget(self.terminal_output)

        # 이동 제어 버튼 설정
        move_control_group = QGroupBox("Movement Control")
        move_layout = QGridLayout() # 이동 제어 버튼을 그리드 레이아웃으로 설정
        move_control_group.setLayout(move_layout)

        self.up_button = QPushButton("Up")
        self.down_button = QPushButton("Down")
        self.left_button = QPushButton("Left")
        self.right_button = QPushButton("Right")
        move_layout.addWidget(self.up_button, 0, 1)
        move_layout.addWidget(self.left_button, 1, 0)
        move_layout.addWidget(self.down_button, 2, 1)
        move_layout.addWidget(self.right_button, 1, 2)
        self.up_button.clicked.connect(lambda: self.send_movement_command("up")) # 이동 제어 버튼에 클릭 이벤트 연결
        self.down_button.clicked.connect(lambda: self.send_movement_command("down"))
        self.left_button.clicked.connect(lambda: self.send_movement_command("left"))
        self.right_button.clicked.connect(lambda: self.send_movement_command("right"))

        # left_bottom_layout.addWidget(move_control_group) # 이동 제어 버튼을 왼쪽 하단 레이아웃에 추가
        # move_control_group.setLayout(move_layout)
        # left_layout.addLayout(left_bottom_layout)

        left_layout.addWidget(move_control_group)
        main_layout.addLayout(left_layout)

        # 오른쪽 레이아웃 설정
        right_layout = QVBoxLayout()

        # 리프트 제어 그룹 설정
        lift_group = QGroupBox("Lift Control")
        lift_layout = QVBoxLayout()
        lift_group.setLayout(lift_layout)

        #preset_heights_group = QGroupBox("Preset Heights")
        #preset_heights_layout = QVBoxLayout()

        self.height1_button = QPushButton("1 Height")
        self.height2_button = QPushButton("2 Height")
        self.height3_button = QPushButton("3 Height")
        lift_layout.addWidget(self.height1_button)
        lift_layout.addWidget(self.height2_button)
        lift_layout.addWidget(self.height3_button)
        self.height1_button.setStyleSheet("font-size: 18px; height: 50px;")
        self.height2_button.setStyleSheet("font-size: 18px; height: 50px;")
        self.height3_button.setStyleSheet("font-size: 18px; height: 50px;") 
        self.height1_button.clicked.connect(lambda: self.send_lift_command("L_20"))
        self.height2_button.clicked.connect(lambda: self.send_lift_command("L_21"))
        self.height3_button.clicked.connect(lambda: self.send_lift_command("L_22"))

        # preset_heights_layout.addWidget(self.height1_button)
        # preset_heights_layout.addWidget(self.height2_button)
        # preset_heights_layout.addWidget(self.height3_button)
        # preset_heights_group.setLayout(preset_heights_layout)
        right_layout.addWidget(lift_group)

        updown_group = QGroupBox("Lift Up/Down")
        updown_layout = QVBoxLayout()
        self.lift_up_button = QPushButton("Lift Up")
        self.lift_down_button = QPushButton("Lift Down")
        self.lift_up_button.setStyleSheet("font-size: 18px; height: 50px;")
        self.lift_down_button.setStyleSheet("font-size: 18px; height: 50px;")
        self.lift_up_button.pressed.connect(lambda: self.start_lift("L_10", "Start Lift Up"))
        self.lift_up_button.released.connect(self.stop_lift)
        self.lift_down_button.pressed.connect(lambda: self.start_lift("L_11", "Start Lift Down"))
        self.lift_down_button.released.connect(self.stop_lift)
        updown_layout.addWidget(self.lift_up_button)
        updown_layout.addWidget(self.lift_down_button)
        updown_group.setLayout(updown_layout)

        lift_layout.addWidget(updown_group)
        # lift_group.setLayout(lift_layout)
        right_layout.addWidget(lift_group)

        # 네비게이션 제어 그룹 설정
        nav_group = QGroupBox("Navigation")
        nav_layout = QVBoxLayout()
        self.toggle_nav_button = QToolButton()
        self.toggle_nav_button.setCheckable(True)
        self.toggle_nav_button.setText("Set Navigation Goal")
        self.toggle_nav_button.setStyleSheet("font-size: 24px; height: 100px;")
        self.toggle_nav_button.clicked.connect(self.toggle_navigation)
        nav_layout.addWidget(self.toggle_nav_button)
        nav_group.setLayout(nav_layout)
        right_layout.addWidget(nav_group)

        # 로봇 상태 표시 그룹 설정
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

        # 메인 레이아웃에 서브 레이아웃 추가
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)

    def start_serial_read_thread(self):  # 시리얼 읽기 스레드 시작
        if self.ser:
            self.read_thread = Thread(target=self.read_from_serial)  # 시리얼 읽기 스레드 생성
            self.read_thread.start()  # 스레드 시작
            self.log_to_terminal("시리얼 읽기 스레드 시작")  # 스레드 시작 로그 출력

    def send_lift_command(self, command):  # 리프트 명령 전송
        if self.ser:
            try:
                self.ser.write(f"{command}\n".encode('utf-8'))  # 시리얼 포트로 명령 전송
                self.log_to_terminal(f"Arduino Send: {command}")
            except serial.SerialException as e:
                self.log_to_terminal(f"Error Arduino Sending : {str(e)}")  # 명령 전송 실패 로그 출력

    def read_from_serial(self):  # 시리얼 포트로부터 읽기
        while True:
            if self.ser and self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').rstrip()  # 시리얼 포트로부터 데이터 읽기
                self.process_serial_data(line)  # 읽은 데이터 처리

    def process_serial_data(self, data):  # 시리얼 데이터 처리
        if data.startswith("E_"):
            status = int(data.split("_")[1])
            if status == 1:
                self.emergency_pub.publish(Int32(data=1))  # 비상 상태 발생
            elif status == 0:
                self.emergency_pub.publish(Int32(data=0))  # 비상 상태 해제
            self.log_to_terminal(f"Arduino received : {data}")

    def move_to_preset_height(self, command, log_message):  # 미리 설정된 높이로 이동
        self.send_lift_command(command)
        self.log_to_terminal(log_message)

    def start_lift(self, command, log_message):  # 리프트 이동 시작 (올리기/내리기)
        self.log_to_terminal(log_message)
        self.current_lift_command = command
        self.lift_timer.timeout.connect(lambda: self.send_lift_command(self.current_lift_command))
        self.lift_timer.start(100)
        self.log_to_terminal("Lift Timer 시작됨")  # QTimer 시작 로그 출력

    def stop_lift(self):  # 리프트 멈추기
        self.log_to_terminal("Stop Lift")
        self.current_lift_command = None
        self.lift_timer.stop()

    def toggle_navigation(self):  # 네비게이션 토글
        nav_state = "enabled" if self.toggle_nav_button.isChecked() else "disabled"
        self.log_to_terminal(f"Navigation {nav_state}")

    def update_status(self, msg):  # ROS로부터 로봇 상태 업데이트
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

    def update_velocity(self, msg):  # ROS로부터 속도 업데이트
        self.velocity = msg.twist.twist.linear.x
        self.log_to_terminal(f"Update Velocity: {self.velocity}")

    def update_imu(self, msg):  # ROS로부터 IMU 업데이트
        self.imu_orientation = msg.orientation.z
        self.log_to_terminal(f"Update IMU: {self.imu_orientation}")

    def update_slam(self, msg):  # ROS로부터 SLAM 업데이트
        self.slam_distance = msg.data
        self.log_to_terminal(f"Update SLAM: {self.slam_distance}")
        self.eta = self.calculate_eta()

    def calculate_eta(self):  # ETA(도착 예상 시간) 계산
        if self.velocity and self.slam_distance:
            return self.slam_distance / self.velocity
        return None

    def send_movement_command(self, direction):  # 이동 명령 전송
        msg = Twist()
        if direction == "up":
            msg.linear.x = 1.0
        elif direction == "down":
            msg.linear.x = -1.0
        elif direction == "left":
            msg.angular.z = 1.0
        elif direction == "right":
            msg.angular.z = -1.0
        self.node.create_publisher(Twist, '/cmd_vel', 10).publish(msg)  # 이동 명령 전송

    def log_to_terminal(self, message):  # 터미널에 로그 출력
        self.terminal_output.append(message)
        self.terminal_output.ensureCursorVisible()

class MainWindow(QMainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Robot Control Panel")

        # 화면 해상도에 따라 메인 윈도우 크기 동적 조정
        screen_geometry = QApplication.primaryScreen().geometry()
        screen_width = screen_geometry.width()
        screen_height = screen_geometry.height()

        # 메인 윈도우의 크기를 화면 해상도의 90%로 설정
        window_width = int(screen_width * 0.9)
        window_height = int(screen_height * 0.9)
        self.setGeometry(0, 0, window_width, window_height)


        # 컨트롤 패널 추가 및 크기 조정
        self.control_panel = ControlPanel(node, self)

        # 메인 레이아웃 설정
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.control_panel)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # 컨트롤 패널 내부의 레이아웃 크기 조정
        self.control_panel.adjust_layouts(window_width, window_height)

        # # 초기 크기 설정
        # self.control_panel.map_label.setFixedSize(int(screen_width * 2 / 3), int(screen_height * 2 / 3))
        # self.control_panel.terminal_output.setFixedSize(int(screen_width * 13 / 30), int(screen_height * 3 / 10))
        # self.control_panel.setFixedSize(screen_width, screen_height)

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
    main()