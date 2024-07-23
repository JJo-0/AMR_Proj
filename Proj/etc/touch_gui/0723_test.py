import sys  # 시스템 관련 모듈
import serial  # 시리얼 통신 모듈
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout,
    QToolButton, QLabel, QGroupBox, QTextEdit, QGridLayout
)  # PyQt5 위젯
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer  # PyQt5 핵심 모듈
import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드
from std_msgs.msg import String, Int32, Float32  # ROS2 표준 메시지 타입
from sensor_msgs.msg import Imu  # IMU 센서 메시지 타입
from nav_msgs.msg import Odometry  # 오도메트리 메시지 타입
from geometry_msgs.msg import PoseStamped, Twist  # 기하학적 메시지 타입
from threading import Thread, Lock  # 스레딩 및 락 모듈
import time  # 시간 모듈

class SerialReader(QThread):  # 시리얼 읽기 스레드 클래스
    new_data = pyqtSignal(str)  # 새 데이터 신호

    def __init__(self, ser, lock):  # 초기화
        super().__init__()  # 부모 클래스 초기화
        self.ser = ser  # 시리얼 객체
        self.lock = lock  # 락 객체
        self.running = True  # 스레드 실행 플래그

    def run(self):  # 스레드 실행 함수
        while self.running:  # 실행 중일 때
            try:
                if self.ser and self.ser.in_waiting > 0:  # 시리얼 데이터 대기 중
                    line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()  # 시리얼 데이터 읽기, 디코딩 에러 무시
                    self.new_data.emit(line)  # 새 데이터 신호 발생
            except serial.SerialException as e:  # 시리얼 예외 처리
                self.new_data.emit(f"Serial error: {e}")  # 에러 메시지 발생
            time.sleep(0.1)  # 0.1초 대기

    def stop(self):  # 스레드 중지 함수
        self.running = False  # 실행 플래그 중지
        self.wait()  # 스레드 대기

class ControlPanel(QWidget):  # 컨트롤 패널 클래스
    def __init__(self, node, parent=None):  # 초기화
        super(ControlPanel, self).__init__(parent)  # 부모 클래스 초기화
        self.node = node  # ROS 노드

        self.ser = None  # 시리얼 객체 초기화
        self.velocity = None  # 속도 변수 초기화
        self.imu_orientation = None  # IMU 방향 변수 초기화
        self.slam_distance = None  # SLAM 거리 변수 초기화
        self.eta = None  # 도착 예상 시간 변수 초기화
        self.current_lift_command = None  # 현재 리프트 명령 초기화
        self.ems_signal = 1  # 비상 상태 초기화
        self.lift_timer = QTimer()  # 리프트 타이머 초기화

        self.status_labels = {  # 상태 라벨 초기화
            "EMS Signal": QLabel(),
            "Lift Signal": QLabel(),
            "Arduino Connection": QLabel()
        }

        self.serial_buffer = []  # 시리얼 버퍼 리스트 초기화
        self.serial_lock = Lock()  # 시리얼 락 객체

        self.init_ui()  # UI 초기화

        self.log_to_terminal("UI Set Success!")  # 로그 메시지

        self.setup_serial_connection('/dev/ttyACM0', 115200)  # 시리얼 연결 설정
        self.start_serial_read_thread()  # 시리얼 읽기 스레드 시작
        self.start_serial_process_thread()  # 시리얼 처리 스레드 시작

        self.emergency_pub = self.node.create_publisher(Int32, '/ems_sig', 10)  # 비상 신호 퍼블리셔
        self.lift_pub = self.node.create_publisher(String, '/lift_command', 10)  # 리프트 명령 퍼블리셔
        self.nav_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)  # 네비게이션 목표 퍼블리셔
        self.velocity_sub = self.node.create_subscription(Odometry, '/odom', self.update_velocity, 10)  # 속도 서브스크립션
        self.imu_sub = self.node.create_subscription(Imu, '/imu', self.update_imu, 10)  # IMU 서브스크립션
        self.slam_sub = self.node.create_subscription(Float32, '/slam_remaining_distance', self.update_slam, 10)  # SLAM 서브스크립션

    def setup_serial_connection(self, port, baud_rate):  # 시리얼 연결 설정 함수
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1)  # 시리얼 포트 연결
            self.ser.reset_input_buffer()  # 입력 버퍼 초기화
            self.update_status_label("Arduino Connection", "Connected", "green")  # 상태 라벨 업데이트
            self.log_to_terminal(f"[Serial Connected] : {port} @ {baud_rate}")  # 로그 메시지
        except serial.SerialException as e:  # 시리얼 예외 처리
            self.update_status_label("Arduino Connection", "Error", "red")  # 상태 라벨 업데이트
            self.log_to_terminal(f"[Serial Connected Failed] : {str(e)}")  # 로그 메시지
            self.ser = None  # 시리얼 객체 초기화

    def init_ui(self):  # UI 초기화 함수
        main_layout = QHBoxLayout()  # 메인 레이아웃
        self.setLayout(main_layout)  # 레이아웃 설정

        left_layout = QVBoxLayout()  # 왼쪽 레이아웃
        self.map_label = QLabel("Map Area")  # 지도 라벨
        self.map_label.setStyleSheet("background-color: lightgray;")  # 라벨 스타일
        self.map_label.setFixedHeight(400)  # 고정 높이 설정
        left_layout.addWidget(self.map_label)  # 레이아웃에 위젯 추가

        self.terminal_output = QTextEdit()  # 터미널 출력
        self.terminal_output.setReadOnly(True)  # 읽기 전용
        self.terminal_output.setStyleSheet("background-color: black; color: white;")  # 스타일
        self.terminal_output.setFixedHeight(50)  # 고정 높이
        left_layout.addWidget(self.terminal_output)  # 레이아웃에 추가

        move_control_group = QGroupBox("Movement Control")  # 이동 제어 그룹
        move_layout = QGridLayout()  # 그리드 레이아웃
        move_control_group.setLayout(move_layout)  # 레이아웃 설정
        move_control_group.setFixedHeight(200)  # 고정 높이 설정

        self.forward_button = self.create_button("Forward", self.start_movement, "forward", 50)
        self.backward_button = self.create_button("Backward", self.start_movement, "backward", 50)
        self.left_button = self.create_button("Left", self.start_movement, "left", 50)
        self.right_button = self.create_button("Right", self.start_movement, "right", 50)
        self.stop_button = self.create_button("Stop", self.stop_movement, height=50)

        move_layout.addWidget(self.backward_button, 0, 1)
        move_layout.addWidget(self.left_button, 1, 0)
        move_layout.addWidget(self.stop_button, 1, 1)
        move_layout.addWidget(self.right_button, 1, 2)
        move_layout.addWidget(self.forward_button, 2, 1)

        self.emergency_stop_button = QToolButton()
        self.emergency_stop_button.setCheckable(True)
        self.emergency_stop_button.setText("EMS")
        self.emergency_stop_button.setStyleSheet("font-size: 24px; height: 100px; background-color: green;")
        self.emergency_stop_button.clicked.connect(self.handle_emergency_stop)
        self.emergency_stop_button.setFixedSize(150, 150)  
        left_control_layout = QHBoxLayout()
        left_control_layout.addWidget(move_control_group)
        left_control_layout.addWidget(self.emergency_stop_button)
        left_layout.addLayout(left_control_layout)

        main_layout.addLayout(left_layout)
        right_layout = QVBoxLayout()

        lift_group = QGroupBox("Lift Control")
        lift_layout = QVBoxLayout()
        lift_group.setLayout(lift_layout)

        self.height1_button = self.create_lift_button("1 Height", "L_20", "1 Point", 50)
        self.height2_button = self.create_lift_button("2 Height", "L_21", "2 Point", 50)
        self.height3_button = self.create_lift_button("3 Height", "L_22", "3 Point", 50)
        lift_layout.addWidget(self.height1_button)
        lift_layout.addWidget(self.height2_button)
        lift_layout.addWidget(self.height3_button)

        right_layout.addWidget(lift_group)

        lift_updown_group = QGroupBox("Lift Up/Down")
        lift_updown_layout = QVBoxLayout()
        self.lift_up_button = self.create_lift_button("Lift Up", "L_10", "Lift Up", 50)
        self.lift_down_button = self.create_lift_button("Lift Down", "L_11", "Lift Down", 50)
        lift_updown_layout.addWidget(self.lift_up_button)
        lift_updown_layout.addWidget(self.lift_down_button)
        lift_updown_group.setLayout(lift_updown_layout)

        right_layout.addWidget(lift_updown_group)

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

        status_group = QGroupBox("Robot Status")
        status_layout = QVBoxLayout()

        self.status_labels = {
            "EMS Signal": self.create_status_label(),
            "Lift Signal": self.create_status_label(),
            "Arduino Connection": self.create_status_label()
        }

        for key, label in self.status_labels.items():
            status_layout.addWidget(QLabel(key))
            status_layout.addWidget(label)

        status_group.setLayout(status_layout)
        right_layout.addWidget(status_group)
        main_layout.addLayout(right_layout)

        self.update_status_label("EMS Signal", "Good: 1", "green")
        self.update_status_label("Lift Signal", "-", "black")
        self.update_status_label("Arduino Connection", "Disconnected", "black")

        def create_button(self, text, func, *args, height=24, label=None):
            button = QPushButton(text)
            button.setStyleSheet(f"font-size: 24px; height: {height}px;")
            button.setFixedHeight(height)
            button.pressed.connect(lambda: func(*args))
            button.released.connect(self.stop_movement)
            return button

        def create_lift_button(self, text, command, label, height=50):
            button = QPushButton(text)
            button.setStyleSheet("font-size: 24px;")
            button.setFixedHeight(height)
            button.pressed.connect(lambda: self.send_lift_command(command, label))
            return button

        def create_status_label(self):
            label = QLabel()
            label.setStyleSheet("font-size: 14px; background-color: black; color: white; padding: 5px;")
            return label

        def update_status_label(self, label_name, text, color):
            label = self.status_labels.get(label_name)
            if label:
                label.setText(text)
                label.setStyleSheet(f"font-size: 14px; padding: 5px; color: white; background-color: {color}; border-radius: 10px;")

        def start_serial_read_thread(self):
            if self.ser:
                self.read_thread = Thread(target=self.read_from_serial)
                self.read_thread.start()
                self.log_to_terminal("Serial Reading Thread Start")

        def start_serial_process_thread(self):
            self.process_thread = Thread(target=self.process_serial_buffer)
            self.process_thread.start()

        def send_lift_command(self, command, label):
            self.update_status_label("Lift Signal", label, "green")
            if self.ser:
                try:
                    self.ser.write(f"{command}\n".encode('utf-8'))
                    self.log_to_terminal(f"[Arduino Send] : {command}")
                    QTimer.singleShot(5000, lambda: self.update_status_label("Lift Signal", "-", "black"))
                except serial.SerialException as e:
                    self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")

        def start_lift_command(self, command, label):
            self.lift_timer.timeout.connect(lambda: self.send_lift_command(command, label))
            self.lift_timer.start(500)  # 0.5초마다 실행

        def stop_lift_command(self):
            self.lift_timer.stop()

        def read_from_serial(self):
            while True:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                    with self.serial_lock:
                        self.serial_buffer.append(line)

        def process_serial_buffer(self):
            while True:
                with self.serial_lock:
                    if self.serial_buffer:
                        data = self.serial_buffer.pop(0)
                        self.process_serial_data(data)
                time.sleep(0.1)

        def process_serial_data(self, data):
            if data.startswith("E_"):
                try:
                    status = int(data.split("_")[1])
                    self.ems_signal = status
                    self.trigger_ems_signal(status)
                    self.log_to_terminal(f"Arduino received : EMS_{data}")
                except (ValueError, IndexError) as e:
                    self.log_to_terminal(f"Invalid data received: {data}")

        def move_to_preset_height(self, command, log_message):
            self.send_lift_command(command, log_message)
            self.log_to_terminal(log_message)

        def toggle_navigation(self):
            nav_state = "Navigating" if self.toggle_nav_button.isChecked() else "Idle"
            color = "green" if self.toggle_nav_button.isChecked() else "black"
            self.update_status_label("Navigation Status", nav_state, color)
            self.log_to_terminal(f"Navigation {nav_state}")

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

        def start_movement(self, direction):
            self.send_movement_command(direction)
            QTimer.singleShot(100, lambda: self.trigger_ems_signal(1))

        def stop_movement(self):
            self.send_movement_command("stop")
            QTimer.singleShot(100, lambda: self.trigger_ems_signal(0))

        def send_movement_command(self, direction):
            msg = Twist()
            if direction == "forward":
                msg.linear.x = 0.1
            elif direction == "backward":
                msg.linear.x = -0.1
            elif direction == "left":
                msg.angular.z = 0.2
            elif direction == "right":
                msg.angular.z = -0.2
            elif direction == "stop":
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            self.node.create_publisher(Twist, '/cmd_vel', 10).publish(msg)

        def trigger_ems_signal(self, status):
            self.emergency_pub.publish(Int32(data=status))
            ems_text = "Good: 1" if status == 1 else "Emergency: 0"
            ems_color = "green" if status == 1 else "red"
            self.update_status_label("EMS Signal", ems_text, ems_color)
            self.emergency_stop_button.setChecked(status == 0)
            if status == 0:
                self.emergency_stop_button.setStyleSheet("font-size: 24px; height: 100px; background-color: darkred;")
            else:
                self.emergency_stop_button.setStyleSheet("font-size: 24px; height: 100px; background-color: green;")

        def handle_emergency_stop(self):
            sender = self.sender()
            if sender.isChecked():
                self.trigger_ems_signal(0)
                if self.ser:
                    try:
                        self.ser.write("E_0\n".encode('utf-8'))
                    except serial.SerialException as e:
                        self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")
            else:
                self.trigger_ems_signal(1)
                if self.ser:
                    try:
                        self.ser.write("E_1\n".encode('utf-8'))
                        self.log_to_terminal(f"[Arduino Send] : E_1")
                    except serial.SerialException as e:
                        self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")

        def log_to_terminal(self, message):
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