import sys
import serial
import subprocess
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit, QGridLayout
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32, Float32
from threading import Thread, Lock
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
        self.ems_signal = 1  # 비상 상태 해제 상태로 초기화

        self.status_labels = {
            "EMS Signal": QLabel(),
            "Lift Signal": QLabel(),
            "Arduino Connection": QLabel()
        }

        self.serial_buffer = []
        self.serial_lock = Lock()

        self.init_ui()  # UI 초기화

        # UI 초기화 후에 로그 출력
        self.log_to_terminal("UI Set Success!")  # ROS 노드 초기화 로그 출력

        self.setup_serial_connection('/dev/ttyACM0', 115200)  # 시리얼 포트 연결 설정
        self.start_serial_read_thread()  # 시리얼 읽기 스레드 시작
        self.start_serial_process_thread()  # 시리얼 처리 스레드 시작

        # ROS 통신 설정
        self.emergency_pub = self.node.create_publisher(Int32, '/ems_sig', 10)
        self.lift_pub = self.node.create_publisher(String, '/lift_command', 10)
        self.nav_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.velocity_sub = self.node.create_subscription(Odometry, '/odom', self.update_velocity, 10)
        self.imu_sub = self.node.create_subscription(Imu, '/imu', self.update_imu, 10)
        self.slam_sub = self.node.create_subscription(Float32, '/slam_remaining_distance', self.update_slam, 10)

    def setup_serial_connection(self, port, baud_rate):  # 시리얼 연결 설정 함수
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1)  # 시리얼 포트 연결
            self.ser.reset_input_buffer()  # 입력 버퍼 리셋
            self.update_status_label("Arduino Connection", "Connected", "green")
            self.log_to_terminal(f"[Serial Connected] : {port} @ {baud_rate}")  # 연결 성공 로그 출력
        except serial.SerialException as e:
            self.update_status_label("Arduino Connection", "Error", "red")
            self.log_to_terminal(f"[Serial Connected Failed] : {str(e)}")  # 연결 실패 로그 출력
            self.ser = None

    def init_ui(self):
        # 메인 레이아웃 설정
        main_layout = QHBoxLayout()  # 전체 레이아웃 수평
        self.setLayout(main_layout)  # 메인 레이아웃 설정

        # 왼쪽 레이아웃 설정
        left_layout = QVBoxLayout()  # 왼쪽 레이아웃 수직
        self.map_label = QLabel("Map Area")  # 지도 영역 라벨 설정
        self.map_label.setStyleSheet("background-color: lightgray;")
        left_layout.addWidget(self.map_label)

        # 왼쪽 하단 레이아웃 설정
        self.terminal_output = QTextEdit()  # 터미널 출력 영역 설정
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("background-color: black; color: white;")
        self.terminal_output.setFixedHeight(200)  # 터미널 높이 조정
        left_layout.addWidget(self.terminal_output)

        # 이동 제어 버튼 설정
        move_control_group = QGroupBox("Movement Control")
        move_layout = QGridLayout()  # 이동 제어 버튼을 그리드 레이아웃으로 설정
        move_control_group.setLayout(move_layout)

        self.forward_button = QPushButton("Forward")
        self.backward_button = QPushButton("Backward")
        self.left_button = QPushButton("Left")
        self.right_button = QPushButton("Right")
        self.stop_button = QPushButton("Stop")
        move_layout.addWidget(self.forward_button, 0, 1)
        move_layout.addWidget(self.left_button, 1, 0)
        move_layout.addWidget(self.stop_button, 1, 1)
        move_layout.addWidget(self.right_button, 1, 2)
        move_layout.addWidget(self.backward_button, 2, 1)
        self.forward_button.clicked.connect(lambda: self.send_movement_command("forward"))  # 이동 제어 버튼에 클릭 이벤트 연결
        self.backward_button.clicked.connect(lambda: self.send_movement_command("backward"))
        self.left_button.clicked.connect(lambda: self.send_movement_command("left"))
        self.right_button.clicked.connect(lambda: self.send_movement_command("right"))
        self.stop_button.clicked.connect(lambda: self.send_movement_command("stop"))

        # 비상정지 버튼 추가
        self.emergency_stop_button = QToolButton()
        self.emergency_stop_button.setCheckable(True)
        self.emergency_stop_button.setText("EMS")
        self.emergency_stop_button.setStyleSheet("font-size: 24px; height: 100px;")
        self.emergency_stop_button.clicked.connect(self.handle_emergency_stop)

        # EMS 버튼을 Movement Control 옆에 추가
        left_control_layout = QHBoxLayout()
        left_control_layout.addWidget(move_control_group)
        left_control_layout.addWidget(self.emergency_stop_button)
        left_layout.addLayout(left_control_layout)

        main_layout.addLayout(left_layout)

        # 오른쪽 레이아웃 설정
        right_layout = QVBoxLayout()

        # 리프트 제어 그룹 설정
        lift_group = QGroupBox("Lift Control")
        lift_layout = QVBoxLayout()
        lift_group.setLayout(lift_layout)

        self.height1_button = QPushButton("1 Height")
        self.height2_button = QPushButton("2 Height")
        self.height3_button = QPushButton("3 Height")
        self.height1_button.setStyleSheet("font-size: 18px;")
        self.height2_button.setStyleSheet("font-size: 18px;")
        self.height3_button.setStyleSheet("font-size: 18px;")
        self.height1_button.clicked.connect(lambda: self.send_lift_command("L_20", "1 Point"))
        self.height2_button.clicked.connect(lambda: self.send_lift_command("L_21", "2 Point"))
        self.height3_button.clicked.connect(lambda: self.send_lift_command("L_22", "3 Point"))
        lift_layout.addWidget(self.height1_button)
        lift_layout.addWidget(self.height2_button)
        lift_layout.addWidget(self.height3_button)

        right_layout.addWidget(lift_group)

        # Lift Up/Down 버튼을 별도 그룹으로 설정
        lift_updown_group = QGroupBox("Lift Up/Down")
        lift_updown_layout = QVBoxLayout()
        self.lift_up_button = QPushButton("Lift Up")
        self.lift_down_button = QPushButton("Lift Down")
        self.lift_up_button.setStyleSheet("font-size: 18px;")
        self.lift_down_button.setStyleSheet("font-size: 18px;")
        self.lift_up_button.pressed.connect(lambda: self.start_lift("L_10", "Lift Up"))
        self.lift_up_button.released.connect(self.stop_lift)
        self.lift_down_button.pressed.connect(lambda: self.start_lift("L_11", "Lift Down"))
        self.lift_down_button.released.connect(self.stop_lift)
        lift_updown_layout.addWidget(self.lift_up_button)
        lift_updown_layout.addWidget(self.lift_down_button)
        lift_updown_group.setLayout(lift_updown_layout)

        right_layout.addWidget(lift_updown_group)

        # 네비게이션 제어 그룹 설정
        nav_group = QGroupBox("Navigation")
        nav_layout = QVBoxLayout()
        self.toggle_nav_button = QToolButton()
        self.toggle_nav_button.setCheckable(True)
        self.toggle_nav_button.setText("Set Navigation Goal")
        self.toggle_nav_button.setStyleSheet("font-size: 18px;")
        self.toggle_nav_button.clicked.connect(self.toggle_navigation)
        nav_layout.addWidget(self.toggle_nav_button)
        nav_group.setLayout(nav_layout)
        right_layout.addWidget(nav_group)

        # 로봇 상태 표시 그룹 설정
        status_group = QGroupBox("Robot Status")
        status_layout = QVBoxLayout()

        for key, label in self.status_labels.items():
            label.setStyleSheet("font-size: 14px; background-color: black; color: white; padding: 5px;")
            status_layout.addWidget(QLabel(key))
            status_layout.addWidget(label)

        self.update_status_label("EMS Signal", "-", "black")
        self.update_status_label("Lift Signal", "-", "black")
        self.update_status_label("Arduino Connection", "Disconnected", "black")

        status_group.setLayout(status_layout)
        right_layout.addWidget(status_group)

        # 메인 레이아웃에 서브 레이아웃 추가
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

    def update_status_label(self, label_name, text, color):
        label = self.status_labels.get(label_name, None)
        if label:
            label.setText(f"{text}")
            label.setStyleSheet(f"font-size: 14px; padding: 5px; color: white; background-color: {color}; border-radius: 10px;")

    def start_serial_read_thread(self):  # 시리얼 읽기 스레드 시작
        if self.ser:
            self.read_thread = Thread(target=self.read_from_serial)  # 시리얼 읽기 스레드 생성
            self.read_thread.start()  # 스레드 시작
            self.log_to_terminal("Serial Reading Thread Start")  # 스레드 시작 로그 출력

    def start_serial_process_thread(self):  # 시리얼 처리 스레드 시작
        self.process_thread = Thread(target=self.process_serial_buffer)
        self.process_thread.start()

    def send_lift_command(self, command, label):  # 리프트 명령 전송
        self.update_status_label("Lift Signal", label, "green")
        if self.ser:
            try:
                self.ser.write(f"{command}\n".encode('utf-8'))  # 시리얼 포트로 명령 전송
                self.log_to_terminal(f"[Arduino Send] : {command}")
                QTimer.singleShot(5000, lambda: self.update_status_label("Lift Signal", "-", "black"))
            except serial.SerialException as e:
                self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")  # 명령 전송 실패 로그 출력

    def read_from_serial(self):  # 시리얼 포트로부터 읽기
        while True:
            if self.ser and self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').rstrip()  # 시리얼 포트로부터 데이터 읽기
                with self.serial_lock:
                    self.serial_buffer.append(line)

    def process_serial_buffer(self):
        while True:
            with self.serial_lock:
                if self.serial_buffer:
                    data = self.serial_buffer.pop(0)
                    self.process_serial_data(data)
            time.sleep(0.1)  # 데이터 처리 주기 조절

    def process_serial_data(self, data):  # 시리얼 데이터 처리
        if data.startswith("E_"):
            try:
                status = int(data.split("_")[1])
                self.ems_signal = status  # 로컬 변수 업데이트
                if status == 1:
                    self.emergency_pub.publish(Int32(data=1))  # 비상 상태 해제
                    self.update_status_label("EMS Signal", "Good: 1", "green")
                elif status == 0:
                    self.emergency_pub.publish(Int32(data=0))  # 비상 상태 설정
                    self.update_status_label("EMS Signal", "Emergency: 0", "red")
                self.log_to_terminal(f"Arduino received : EMS_{data}")
            except (ValueError, IndexError):
                self.log_to_terminal(f"Invalid data received: {data}")

    def move_to_preset_height(self, command, log_message):  # 미리 설정된 높이로 이동
        self.send_lift_command(command, log_message)
        self.log_to_terminal(log_message)

    def start_lift(self, command, log_message):  # 리프트 이동 시작 (올리기/내리기)
        self.update_status_label("Lift Signal", log_message, "green")
        self.log_to_terminal(log_message)
        self.current_lift_command = command
        self.lift_timer.timeout.connect(lambda: self.send_lift_command(self.current_lift_command, log_message))
        self.lift_timer.start(100)
        self.log_to_terminal("Lift Timer Started")  # QTimer 시작 로그 출력

    def stop_lift(self):  # 리프트 멈추기
        self.log_to_terminal("Stop Lift")
        self.current_lift_command = None
        self.lift_timer.stop()
        QTimer.singleShot(5000, lambda: self.update_status_label("Lift Signal", "-", "black"))

    def toggle_navigation(self):  # 네비게이션 토글
        nav_state = "Navigating" if self.toggle_nav_button.isChecked() else "Idle"
        color = "green" if self.toggle_nav_button.isChecked() else "black"
        self.update_status_label("Navigation Status", nav_state, color)
        self.log_to_terminal(f"Navigation {nav_state}")

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
        if direction == "forward":
            msg.linear.x = 0.2
        elif direction == "backward":
            msg.linear.x = -0.2
        elif direction == "left":
            msg.angular.z = 0.4
        elif direction == "right":
            msg.angular.z = -0.4
        elif direction == "stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.node.create_publisher(Twist, '/cmd_vel', 10).publish(msg)  # 이동 명령 전송

    def handle_emergency_stop(self):  # 비상 정지 스위치 핸들러
        sender = self.sender()
        if sender.isChecked():
            self.emergency_pub.publish(Int32(data=0))  # 비상 상태 설정
            self.ems_signal = 0  # 로컬 변수 업데이트
            if self.ser:
                try:
                    self.ser.write("E_0\n".encode('utf-8'))
                except serial.SerialException as e:
                    self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")  # 명령 전송 실패 로그 출력
        else:
            self.emergency_pub.publish(Int32(data=1))  # 비상 상태 해제
            self.ems_signal = 1  # 로컬 변수 업데이트
            if self.ser:
                try:
                    self.ser.write("E_1\n".encode('utf-8'))
                except serial.SerialException as e:
                    self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")  # 명령 전송 실패 로그 출력

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