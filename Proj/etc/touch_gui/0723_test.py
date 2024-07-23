import sys  # 시스템 관련 모듈
import serial  # 시리얼 통신 모듈
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit, QGridLayout)  # PyQt5 위젯
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

        self.forward_button = QPushButton("Forward")  # 앞으로 버튼
        self.backward_button = QPushButton("Backward")  # 뒤로 버튼
        self.left_button = QPushButton("Left")  # 왼쪽 버튼
        self.right_button = QPushButton("Right")  # 오른쪽 버튼
        self.stop_button = QPushButton("Stop")  # 정지 버튼
        move_layout.addWidget(self.backward_button, 0, 1)
        move_layout.addWidget(self.left_button, 1, 0)
        move_layout.addWidget(self.stop_button, 1, 1)
        move_layout.addWidget(self.right_button, 1, 2)
        move_layout.addWidget(self.forward_button, 2, 1)

        self.forward_button.pressed.connect(lambda: self.start_movement("forward"))  # 버튼 이벤트 연결
        self.forward_button.released.connect(self.stop_movement)  # 버튼 이벤트 연결
        self.backward_button.pressed.connect(lambda: self.start_movement("backward"))  # 버튼 이벤트 연결
        self.backward_button.released.connect(self.stop_movement)  # 버튼 이벤트 연결
        self.left_button.pressed.connect(lambda: self.start_movement("left"))  # 버튼 이벤트 연결
        self.left_button.released.connect(self.stop_movement)  # 버튼 이벤트 연결
        self.right_button.pressed.connect(lambda: self.start_movement("right"))  # 버튼 이벤트 연결
        self.right_button.released.connect(self.stop_movement)  # 버튼 이벤트 연결
        self.stop_button.clicked.connect(lambda: self.send_movement_command("stop"))  # 버튼 이벤트 연결
        self.forward_button.setFixedHeight(50)  # 버튼 높이 설정
        self.backward_button.setFixedHeight(50)  # 버튼 높이 설정
        self.left_button.setFixedHeight(50)  # 버튼 높이 설정
        self.right_button.setFixedHeight(50)  # 버튼 높이 설정
        self.stop_button.setFixedHeight(50)  # 버튼 높이 설정

        self.emergency_stop_button = QToolButton()  # 비상 정지 버튼
        self.emergency_stop_button.setCheckable(True)  # 체크 가능
        self.emergency_stop_button.setText("EMS")  # 텍스트 설정
        self.emergency_stop_button.setStyleSheet("font-size: 24px; height: 100px;")  # 스타일 설정
        self.emergency_stop_button.clicked.connect(self.handle_emergency_stop)  # 이벤트 연결
        self.emergency_stop_button.setFixedHeight(150)  # 버튼 높이 설정
        left_control_layout = QHBoxLayout()  # 왼쪽 제어 레이아웃
        left_control_layout.addWidget(move_control_group)  # 이동 제어 그룹 추가
        left_control_layout.addWidget(self.emergency_stop_button)  # 비상 정지 버튼 추가
        left_layout.addLayout(left_control_layout)  # 왼쪽 레이아웃에 추가

        main_layout.addLayout(left_layout)  # 메인 레이아웃에 추가

        right_layout = QVBoxLayout()  # 오른쪽 레이아웃

        lift_group = QGroupBox("Lift Control")  # 리프트 제어 그룹
        lift_layout = QVBoxLayout()  # 수직 레이아웃
        lift_group.setLayout(lift_layout)  # 레이아웃 설정

        self.height1_button = QPushButton("1 Height")  # 1 높이 버튼
        self.height2_button = QPushButton("2 Height")  # 2 높이 버튼
        self.height3_button = QPushButton("3 Height")  # 3 높이 버튼
        self.height1_button.setStyleSheet("font-size: 18px;")  # 스타일 설정
        self.height2_button.setStyleSheet("font-size: 18px;")  # 스타일 설정
        self.height3_button.setStyleSheet("font-size: 18px;")  # 스타일 설정
        self.height1_button.clicked.connect(lambda: self.send_lift_command("L_20", "1 Point"))  # 버튼 이벤트 연결
        self.height2_button.clicked.connect(lambda: self.send_lift_command("L_21", "2 Point"))  # 버튼 이벤트 연결
        self.height3_button.clicked.connect(lambda: self.send_lift_command("L_22", "3 Point"))  # 버튼 이벤트 연결
        lift_layout.addWidget(self.height1_button)  # 레이아웃에 추가
        lift_layout.addWidget(self.height2_button)  # 레이아웃에 추가
        lift_layout.addWidget(self.height3_button)  # 레이아웃에 추가

        right_layout.addWidget(lift_group)  # 오른쪽 레이아웃에 추가

        lift_updown_group = QGroupBox("Lift Up/Down")  # 리프트 상하 그룹
        lift_updown_layout = QVBoxLayout()  # 수직 레이아웃
        self.lift_up_button = QPushButton("Lift Up")  # 리프트 업 버튼
        self.lift_down_button = QPushButton("Lift Down")  # 리프트 다운 버튼
        self.lift_up_button.setStyleSheet("font-size: 18px;")  # 스타일 설정
        self.lift_down_button.setStyleSheet("font-size: 18px;")  # 스타일 설정
        self.lift_up_button.clicked.connect(lambda: self.send_lift_command("L_10", "Lift Up"))  # 버튼 이벤트 연결
        self.lift_down_button.clicked.connect(lambda: self.send_lift_command("L_11", "Lift Down"))  # 버튼 이벤트 연결
        lift_updown_layout.addWidget(self.lift_up_button)  # 레이아웃에 추가
        lift_updown_layout.addWidget(self.lift_down_button)  # 레이아웃에 추가
        lift_updown_group.setLayout(lift_updown_layout)  # 레이아웃 설정

        right_layout.addWidget(lift_updown_group)  # 오른쪽 레이아웃에 추가

        nav_group = QGroupBox("Navigation")  # 네비게이션 그룹
        nav_layout = QVBoxLayout()  # 수직 레이아웃
        self.toggle_nav_button = QToolButton()  # 네비게이션 버튼
        self.toggle_nav_button.setCheckable(True)  # 체크 가능
        self.toggle_nav_button.setText("Set Navigation Goal")  # 텍스트 설정
        self.toggle_nav_button.setStyleSheet("font-size: 18px;")  # 스타일 설정
        self.toggle_nav_button.clicked.connect(self.toggle_navigation)  # 이벤트 연결
        nav_layout.addWidget(self.toggle_nav_button)  # 레이아웃에 추가
        nav_group.setLayout(nav_layout)  # 레이아웃 설정
        right_layout.addWidget(nav_group)  # 오른쪽 레이아웃에 추가

        status_group = QGroupBox("Robot Status")  # 로봇 상태 그룹
        status_layout = QVBoxLayout()  # 수직 레이아웃

        for key, label in self.status_labels.items():  # 상태 라벨 설정
            label.setStyleSheet("font-size: 14px; background-color: black; color: white; padding: 5px;")  # 스타일 설정
            status_layout.addWidget(QLabel(key))  # 키 라벨 추가
            status_layout.addWidget(label)  # 상태 라벨 추가

        self.update_status_label("EMS Signal", "Good: 1", "green")  # 초기 상태 업데이트
        self.update_status_label("Lift Signal", "-", "black")  # 초기 상태 업데이트
        self.update_status_label("Arduino Connection", "Disconnected", "black")  # 초기 상태 업데이트

        status_group.setLayout(status_layout)  # 레이아웃 설정
        right_layout.addWidget(status_group)  # 오른쪽 레이아웃에 추가
        main_layout.addLayout(left_layout)  # 메인 레이아웃에 추가
        main_layout.addLayout(right_layout)  # 메인 레이아웃에 추가

    def update_status_label(self, label_name, text, color):  # 상태 라벨 업데이트 함수
        label = self.status_labels.get(label_name, None)  # 라벨 가져오기
        if label:  # 라벨이 존재하면
            label.setText(f"{text}")  # 텍스트 설정
            label.setStyleSheet(f"font-size: 14px; padding: 5px; color: white; background-color: {color}; border-radius: 10px;")  # 스타일 설정

    def start_serial_read_thread(self):  # 시리얼 읽기 스레드 시작 함수
        if self.ser:  # 시리얼 객체가 존재하면
            self.read_thread = Thread(target=self.read_from_serial)  # 읽기 스레드 생성
            self.read_thread.start()  # 스레드 시작
            self.log_to_terminal("Serial Reading Thread Start")  # 로그 메시지

    def start_serial_process_thread(self):  # 시리얼 처리 스레드 시작 함수
        self.process_thread = Thread(target=self.process_serial_buffer)  # 처리 스레드 생성
        self.process_thread.start()  # 스레드 시작

    def send_lift_command(self, command, label):  # 리프트 명령 전송 함수
        self.update_status_label("Lift Signal", label, "green")  # 상태 라벨 업데이트
        if self.ser:  # 시리얼 객체가 존재하면
            try:
                self.ser.write(f"{command}\n".encode('utf-8'))  # 명령 전송
                self.log_to_terminal(f"[Arduino Send] : {command}")  # 로그 메시지
                QTimer.singleShot(5000, lambda: self.update_status_label("Lift Signal", "-", "black"))  # 5초 후 상태 라벨 초기화
            except serial.SerialException as e:  # 시리얼 예외 처리
                self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")  # 에러 로그 메시지

    def read_from_serial(self):  # 시리얼 읽기 함수
        while True:  # 계속 실행
            if self.ser and self.ser.in_waiting > 0:  # 시리얼 데이터 대기 중
                line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()  # 시리얼 데이터 읽기, 디코딩 에러 무시
                with self.serial_lock:  # 락 사용
                    self.serial_buffer.append(line)  # 버퍼에 추가

    def process_serial_buffer(self):  # 시리얼 버퍼 처리 함수
        while True:  # 계속 실행
            with self.serial_lock:  # 락 사용
                if self.serial_buffer:  # 버퍼에 데이터가 있으면
                    data = self.serial_buffer.pop(0)  # 데이터 꺼내기
                    self.process_serial_data(data)  # 데이터 처리
            time.sleep(0.1)  # 0.1초 대기

    def process_serial_data(self, data):  # 시리얼 데이터 처리 함수
        if data.startswith("E_"):  # 데이터가 E_로 시작하면
            try:
                status = int(data.split("_")[1])  # 상태 값 파싱
                self.ems_signal = status  # 상태 변수 업데이트
                if status == 1:  # 상태가 1이면
                    self.emergency_pub.publish(Int32(data=1))  # 비상 해제 신호 전송
                    self.update_status_label("EMS Signal", "Good: 1", "green")  # 상태 라벨 업데이트
                    self.emergency_stop_button.setChecked(False)  # 비상 정지 버튼 해제
                elif status == 0:  # 상태가 0이면
                    self.emergency_pub.publish(Int32(data=0))  # 비상 신호 전송
                    self.update_status_label("EMS Signal", "Emergency: 0", "red")  # 상태 라벨 업데이트
                    self.emergency_stop_button.setChecked(True)  # 비상 정지 버튼 설정
                self.log_to_terminal(f"Arduino received : EMS_{data}")  # 로그 메시
            except (ValueError, IndexError) as e:  # 예외 처리
                self.log_to_terminal(f"Invalid data received: {data}") # 에러 로그 메시지

    def move_to_preset_height(self, command, log_message):  # 미리 설정된 높이로 이동 함수
        self.send_lift_command(command, log_message)  # 리프트 명령 전송
        self.log_to_terminal(log_message)  # 로그 메시지

    def toggle_navigation(self):  # 네비게이션 토글 함수
        nav_state = "Navigating" if self.toggle_nav_button.isChecked() else "Idle"  # 네비게이션 상태
        color = "green" if self.toggle_nav_button.isChecked() else "black"  # 색상 설정
        self.update_status_label("Navigation Status", nav_state, color)  # 상태 라벨 업데이트
        self.log_to_terminal(f"Navigation {nav_state}")  # 로그 메시지

    def update_velocity(self, msg):  # 속도 업데이트 함수
        self.velocity = msg.twist.twist.linear.x  # 속도 설정
        self.log_to_terminal(f"Update Velocity: {self.velocity}")  # 로그 메시지

    def update_imu(self, msg):  # IMU 업데이트 함수
        self.imu_orientation = msg.orientation.z  # IMU 방향 설정
        self.log_to_terminal(f"Update IMU: {self.imu_orientation}")  # 로그 메시지

    def update_slam(self, msg):  # SLAM 업데이트 함수
        self.slam_distance = msg.data  # SLAM 거리 설정
        self.log_to_terminal(f"Update SLAM: {self.slam_distance}")  # 로그 메시지
        self.eta = self.calculate_eta()  # ETA 계산

    def calculate_eta(self):  # ETA 계산 함수
        if self.velocity and self.slam_distance:  # 속도와 SLAM 거리가 있으면
            return self.slam_distance / self.velocity  # ETA 계산
        return None  # 없으면 None 반환

    def start_movement(self, direction):  # 이동 시작 함수
        self.emergency_pub.publish(Int32(data=1))  # EMS 신호를 1로 설정
        self.update_status_label("EMS Signal", "1 : Good", "green") 
        self.emergency_stop_button.setChecked(False)  # EMS 버튼 상태 해제
        self.send_movement_command(direction)  # 이동 명령 전송

    def stop_movement(self):  # 이동 멈추기 함수
        self.emergency_pub.publish(Int32(data=0))  # EMS 신호를 0로 설정
        self.update_status_label("EMS Signal", "0 : Emergency", "red") 
        self.emergency_stop_button.setChecked(True)  # EMS 버튼 상태 설정
        self.send_movement_command("stop")  # 이동 정지 명령 전송

    def send_movement_command(self, direction):  # 이동 명령 전송 함수
        msg = Twist()  # 메시지 생성
        if direction == "forward":  # 앞으로 이동
            msg.linear.x = 0.1  # 속도 설정
        elif direction == "backward":  # 뒤로 이동
            msg.linear.x = -0.1  # 속도 설정
        elif direction == "left":  # 왼쪽 회전
            msg.angular.z = 0.2  # 회전 속도 설정
        elif direction == "right":  # 오른쪽 회전
            msg.angular.z = -0.2  # 회전 속도 설정
        elif direction == "stop":  # 정지
            msg.linear.x = 0.0  # 속도 초기화
            msg.angular.z = 0.0  # 회전 속도 초기화
        self.node.create_publisher(Twist, '/cmd_vel', 10).publish(msg)  # 명령 전송

    def handle_emergency_stop(self):  # 비상 정지 처리 함수
        sender = self.sender()  # 신호 보낸 객체
        if sender.isChecked():  # 버튼이 눌리면
            self.emergency_pub.publish(Int32(data=0))  # 비상 신호 전송
            self.update_status_label("EMS Signal", "0 : Emergency", "red") 
            self.ems_signal = 0  # 비상 상태 설정
            if self.ser:  # 시리얼 객체가 있으면
                try:
                    self.ser.write("E_0\n".encode('utf-8'))  # 시리얼 전송
                except serial.SerialException as e:  # 시리얼 예외 처리
                    self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")  # 에러 로그 메시지
        else:  # 버튼이 해제되면
            self.emergency_pub.publish(Int32(data=1))  # 비상 해제 신호 전송
            self.update_status_label("EMS Signal", "1 : Good", "green")
            self.ems_signal = 1  # 비상 상태 해제
            if self.ser:  # 시리얼 객체가 있으면
                try:
                    self.ser.write("E_1\n".encode('utf-8'))  # 시리얼 전송
                    self.log_to_terminal(f"[Arduino Send] : E_1")  # 아두이노로 보낸 메시지 로그 출력
                except serial.SerialException as e:  # 시리얼 예외 처리
                    self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")  # 에러 로그 메시지

    def log_to_terminal(self, message):  # 터미널 로그 출력 함수
        self.terminal_output.append(message)  # 메시지 추가
        self.terminal_output.ensureCursorVisible()  # 커서 가시성 유지

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