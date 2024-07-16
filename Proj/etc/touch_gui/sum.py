import sys  # 시스템 관련 기능을 제공하는 모듈
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QGroupBox, QTextEdit, QToolButton, QMainWindow  # PyQt5 GUI 구성 요소 임포트
from gpiozero import DigitalOutputDevice, DistanceSensor  # GPIO 제어를 위한 gpiozero 모듈 임포트
from threading import Thread  # 멀티스레딩을 위한 threading 모듈 임포트
import time  # 시간 지연을 위한 time 모듈 임포트
import rclpy  # ROS 2 클라이언트 라이브러리
from rclpy.node import Node  # ROS 2 노드 클래스 임포트
from std_msgs.msg import String  # ROS 표준 메시지 타입 임포트
from sensor_msgs.msg import Imu  # IMU 센서 메시지 타입 임포트
from nav_msgs.msg import Odometry  # 오도메트리 메시지 타입 임포트
from geometry_msgs.msg import PoseStamped  # PoseStamped 메시지 타입 임포트
from std_msgs.msg import Float32  # ROS 표준 Float32 메시지 타입 임포트

# GPIO 제어 클래스 정의
class GPIOControl(QWidget):
    def __init__(self):
        super().__init__()  # 부모 클래스 초기화
        self.initUI()  # UI 초기화
        self.initGPIO()  # GPIO 초기화
        self.startDistanceThread()  # 거리 측정 스레드 시작

    def initUI(self):
        layout = QVBoxLayout()  # 세로 레이아웃 설정

        self.upButton = QPushButton('UP', self)  # 'UP' 버튼 생성
        self.upButton.clicked.connect(self.handleUpButton)  # 'UP' 버튼 클릭 시 handleUpButton 메소드 연결
        layout.addWidget(self.upButton)  # 레이아웃에 'UP' 버튼 추가

        self.downButton = QPushButton('DOWN', self)  # 'DOWN' 버튼 생성
        self.downButton.clicked.connect(self.handleDownButton)  # 'DOWN' 버튼 클릭 시 handleDownButton 메소드 연결
        layout.addWidget(self.downButton)  # 레이아웃에 'DOWN' 버튼 추가

        self.stopButton = QPushButton('STOP', self)  # 'STOP' 버튼 생성
        self.stopButton.clicked.connect(self.handleStopButton)  # 'STOP' 버튼 클릭 시 handleStopButton 메소드 연결
        layout.addWidget(self.stopButton)  # 레이아웃에 'STOP' 버튼 추가

        self.autoButton = QPushButton('Auto Adjust', self)  # 'Auto Adjust' 버튼 생성
        self.autoButton.clicked.connect(self.handleAutoButton)  # 'Auto Adjust' 버튼 클릭 시 handleAutoButton 메소드 연결
        layout.addWidget(self.autoButton)  # 레이아웃에 'Auto Adjust' 버튼 추가

        self.distanceLabel = QLabel('Distance: -- cm', self)  # 거리 표시 라벨 생성
        layout.addWidget(self.distanceLabel)  # 레이아웃에 거리 라벨 추가

        self.setLayout(layout)  # 레이아웃 설정

    def initGPIO(self):
        self.pin23 = DigitalOutputDevice(23)  # 핀 23을 디지털 출력으로 설정
        self.pin24 = DigitalOutputDevice(24)  # 핀 24을 디지털 출력으로 설정
        self.sensor = DistanceSensor(echo=27, trigger=17)  # 초음파 거리 센서 설정 (에코 핀 27, 트리거 핀 17)
        self.measurementActive = False  # 거리 측정 활성화 플래그
        self.distanceThreadActive = True  # 거리 측정 스레드 활성화 플래그

    def handleUpButton(self):
        self.pin23.on()  # 핀 23 켜기
        self.pin24.off()  # 핀 24 끄기

    def handleDownButton(self):
        self.pin23.off()  # 핀 23 끄기
        self.pin24.on()  # 핀 24 켜기

    def handleStopButton(self):
        self.pin23.off()  # 핀 23 끄기
        self.pin24.off()  # 핀 24 끄기

    def handleAutoButton(self):
        self.measurementActive = True  # 거리 측정 활성화
        self.autoThread = Thread(target=self.autoAdjust)  # 거리 자동 조절 스레드 생성
        self.autoThread.start()  # 스레드 시작

    def autoAdjust(self):
        try:
            self.handleUpButton()  # 'UP' 동작 시작
            while self.measurementActive:
                distance = self.sensor.distance * 100  # 거리 측정 (센티미터로 변환)
                if 16 <= distance <= 17:  # 거리가 16~17cm 이내일 경우
                    self.handleStopButton()  # 'STOP' 동작
                    self.measurementActive = False  # 거리 측정 비활성화
                time.sleep(0.1)  # 0.1초 대기
        except KeyboardInterrupt:
            self.measurementActive = False  # 거리 측정 비활성화

    def startDistanceThread(self):
        self.distanceThread = Thread(target=self.updateDistance)  # 거리 업데이트 스레드 생성
        self.distanceThread.start()  # 스레드 시작

    def updateDistance(self):
        while self.distanceThreadActive:
            distance = self.sensor.distance * 100  # 거리 측정 (센티미터로 변환)
            self.distanceLabel.setText(f'Distance: {distance:.2f} cm')  # 거리 라벨 업데이트
            time.sleep(0.1)  # 0.1초 대기

    def closeEvent(self, event):
        self.measurementActive = False  # 거리 측정 비활성화
        self.distanceThreadActive = False  # 거리 측정 스레드 비활성화
        if hasattr(self, 'autoThread'):
            self.autoThread.join()  # 자동 조절 스레드 종료 대기
        if hasattr(self, 'distanceThread'):
            self.distanceThread.join()  # 거리 업데이트 스레드 종료 대기
        self.pin23.close()  # 핀 23 닫기
        self.pin24.close()  # 핀 24 닫기
        self.sensor.close()  # 센서 닫기
        event.accept()  # 이벤트 수락

# ROS 제어 클래스 정의
class ControlPanel(QWidget):
    def __init__(self, node, parent=None):
        super(ControlPanel, self).__init__(parent)  # 부모 클래스 초기화
        self.node = node  # ROS 노드 설정

        main_layout = QHBoxLayout()  # 전체 레이아웃을 가로로 설정
        left_layout = QVBoxLayout()  # 왼쪽 레이아웃을 세로로 설정

        self.map_label = QLabel("Map Area")  # 맵을 표시할 공간 생성
        self.map_label.setStyleSheet("background-color: lightgray;")  # 맵 라벨 배경색 설정
        left_layout.addWidget(self.map_label)  # 왼쪽 레이아웃에 맵 라벨 추가

        self.terminal_output = QTextEdit()  # 터미널 출력 영역 생성
        self.terminal_output.setReadOnly(True)  # 터미널 출력 영역 읽기 전용 설정
        self.terminal_output.setStyleSheet("background-color: black; color: white;")  # 터미널 출력 영역 스타일 설정
        left_layout.addWidget(self.terminal_output)  # 왼쪽 레이아웃에 터미널 출력 영역 추가

        right_layout = QVBoxLayout()  # 오른쪽 레이아웃을 세로로 설정

        lift_group = QGroupBox("Lift Control")  # 리프트 제어 그룹박스 생성
        lift_layout = QVBoxLayout()  # 리프트 제어 레이아웃 세로로 설정
        self.lift_up_button = QPushButton("Lift Up")  # 'Lift Up' 버튼 생성
        self.lift_up_button.setStyleSheet("font-size: 24px; height: 100px;")  # 버튼 스타일 설정
        self.lift_down_button = QPushButton("Lift Down")  # 'Lift Down' 버튼 생성
        self.lift_down_button.setStyleSheet("font-size: 24px; height: 100px;")  # 버튼 스타일 설정
        lift_layout.addWidget(self.lift_up_button)  # 리프트 레이아웃에 'Lift Up' 버튼 추가
        lift_layout.addWidget(self.lift_down_button)  # 리프트 레이아웃에 'Lift Down' 버튼 추가
        lift_group.setLayout(lift_layout)  # 그룹박스에 리프트 레이아웃 설정
        right_layout.addWidget(lift_group)  # 오른쪽 레이아웃에 그룹박스 추가

        nav_group = QGroupBox("Navigation")  # 네비게이션 그룹박스 생성
        nav_layout = QVBoxLayout()  # 네비게이션 레이아웃 세로로 설정
        self.toggle_nav_button = QToolButton()  # 네비게이션 토글 버튼 생성
        self.toggle_nav_button.setCheckable(True)  # 버튼 토글 가능 설정
        self.toggle_nav_button.setText("Set Navigation Goal“)  # 버튼 텍스트 설정
        self.toggle_nav_button.setStyleSheet(“font-size: 24px; height: 100px;”)  # 버튼 스타일 설정
        nav_layout.addWidget(self.toggle_nav_button)  # 네비게이션 레이아웃에 버튼 추가
        nav_group.setLayout(nav_layout)  # 그룹박스에 네비게이션 레이아웃 설정
        right_layout.addWidget(nav_group)  # 오른쪽 레이아웃에 그룹박스 추가
            status_group = QGroupBox("Robot Status")  # 로봇 상태 표시 그룹박스 생성
    status_layout = QVBoxLayout()  # 상태 표시 레이아웃 세로로 설정
    status_box_layout = QHBoxLayout()  # 상태 표시 박스 레이아웃 가로로 설정
    self.status_color_label = QLabel()  # 상태 색상 라벨 생성
    self.status_color_label.setFixedSize(30, 30)  # 상태 색상 라벨 크기 설정
    self.status_color_label.setStyleSheet("background-color: black;")  # 상태 색상 라벨 배경색 설정
    self.status_info_label = QLabel("No Signal")  # 상태 정보 라벨 생성
    self.status_info_label.setStyleSheet("font-size: 18px;")  # 상태 정보 라벨 스타일 설정
    status_box_layout.addWidget(self.status_color_label)  # 상태 박스 레이아웃에 상태 색상 라벨 추가
    status_box_layout.addWidget(self.status_info_label)  # 상태 박스 레이아웃에 상태 정보 라벨 추가
    status_layout.addLayout(status_box_layout)  # 상태 레이아웃에 상태 박스 레이아웃 추가
    status_group.setLayout(status_layout)  # 그룹박스에 상태 레이아웃 설정
    right_layout.addWidget(status_group)  # 오른쪽 레이아웃에 그룹박스 추가

    main_layout.addLayout(left_layout)  # 메인 레이아웃에 왼쪽 레이아웃 추가
    main_layout.addLayout(right_layout)  # 메인 레이아웃에 오른쪽 레이아웃 추가

    self.setLayout(main_layout)  # 메인 레이아웃 설정

    self.lift_up_button.pressed.connect(self.start_lift_up)  # 'Lift Up' 버튼 눌림 이벤트 연결
    self.lift_up_button.released.connect(self.stop_lift)  # 'Lift Up' 버튼 놓음 이벤트 연결
    self.lift_down_button.pressed.connect(self.start_lift_down)  # 'Lift Down' 버튼 눌림 이벤트 연결
    self.lift_down_button.released.connect(self.stop_lift)  # 'Lift Down' 버튼 놓음 이벤트 연결
    self.toggle_nav_button.clicked.connect(self.toggle_navigation)  # 네비게이션 토글 버튼 클릭 이벤트 연결

    self.lift_pub = self.node.create_publisher(String, '/lift_command', 10)  # 리프트 명령 퍼블리셔 생성
    self.nav_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)  # 네비게이션 명령 퍼블리셔 생성
    self.status_sub = self.node.create_subscription(String, '/robot_status', self.update_status, 10)  # 로봇 상태 구독자 생성
    self.velocity_sub = self.node.create_subscription(Odometry, '/odom', self.update_velocity, 10)  # 로봇 속도 구독자 생성
    self.imu_sub = self.node.create_subscription(Imu, '/imu', self.update_imu, 10)  # IMU 데이터 구독자 생성
    self.slam_sub = self.node.create_subscription(Float32, '/slam_remaining_distance', self.update_slam, 10)  # SLAM 거리 구독자 생성

    self.velocity = None  # 속도 정보 초기화
    self.imu_orientation = None  # IMU 정보 초기화
    self.slam_distance = None  # SLAM 거리 초기화
    self.eta = None  # 예상 도착 시간 초기화

    self.lift_timer = QTimer()  # 리프트 타이머 생성
    self.lift_timer.timeout.connect(self.send_lift_command)  # 타이머 타임아웃 시 명령 전송
    self.current_lift_command = None  # 현재 리프트 명령 초기화

    def start_lift_up(self):
        self.log_to_terminal("Start Lift Up")  # 터미널에 로그 출력
        self.current_lift_command = "up"  # 리프트 명령 설정
        self.lift_timer.start(100)  # 타이머 시작

    def start_lift_down(self):
        self.log_to_terminal("Start Lift Down")  # 터미널에 로그 출력
        self.current_lift_command = "down"  # 리프트 명령 설정
        self.lift_timer.start(100)  # 타이머 시작

    def stop_lift(self):
        self.log_to_terminal("Stop Lift")  # 터미널에 로그 출력
        self.lift_timer.stop()  # 타이머 정지

    def send_lift_command(self):
        if self.current_lift_command:  # 현재 명령이 있으면
            msg = String()  # String 메시지 생성
            msg.data = self.current_lift_command  # 메시지 데이터 설정
            self.lift_pub.publish(msg)  # 메시지 퍼블리시

    def toggle_navigation(self):
        nav_state = "enabled" if self.toggle_nav_button.isChecked() else "disabled"  # 네비게이션 상태 설정
        self.log_to_terminal(f"Navigation {nav_state}")  # 터미널에 로그 출력

    def update_status(self, msg):
        status = msg.data  # 상태 데이터 수신
        self.log_to_terminal(f"Update Status: {status}")  # 터미널에 로그 출력
        if status == "normal":  # 상태가 'normal'일 경우
            self.status_color_label.setStyleSheet("background-color: green;")  # 상태 색상 설정
            self.status_info_label.setText(f"Velocity: {self.velocity if self.velocity is not None else ''}\n"
                                        f"IMU: {self.imu_orientation if self.imu_orientation is not None else ''}\n"
                                        f"SLAM: {self.slam_distance if self.slam_distance is not None else ''}\n"
                                        f"ETA: {self.eta if self.eta is not None else ''}")  # 상태 정보 라벨 설정
        elif status == "emergency":  # 상태가 'emergency'일 경우
            self.status_color_label.setStyleSheet("background-color: red;")  # 상태 색상 설정
            self.status_info_label.setText("Emergency Stop")  # 상태 정보 라벨 설정
        else:  # 상태가 'no signal'일 경우
            self.status_color_label.setStyleSheet("background-color: black;")  # 상태 색상 설정
            self.status_info_label.setText("No Signal")  # 상태 정보 라벨 설정

    def update_velocity(self, msg):
        self.velocity = msg.twist.twist.linear.x  # 속도 데이터 수신
        self.log_to_terminal(f"Update Velocity: {self.velocity}")  # 터미널에 로그 출력

    def update_imu(self, msg):
        self.imu_orientation = msg.orientation.z  # IMU 데이터 수신
        self.log_to_terminal(f"Update IMU: {self.imu_orientation}")  # 터미널에 로그 출력

    def update_slam(self, msg):
        self.slam_distance = msg.data  # SLAM 데이터 수신
        self.log_to_terminal(f"Update SLAM: {self.slam_distance}")  # 터미널에 로그 출력
        self.eta = self.calculate_eta()  # ETA 계산

    def calculate_eta(self):
        if self.velocity and self.slam_distance:  # 속도와 SLAM 데이터가 있으면
            return self.slam_distance / self.velocity  # ETA 계산
        return None  # 데이터가 없으면 None 반환

    def log_to_terminal(self, message):
        self.terminal_output.append(message)  # 터미널에 메시지 추가
        self.terminal_output.ensureCursorVisible()  # 커서가 보이도록 설정

class MainWindow(QMainWindow):
    def init(self, node):
    super(MainWindow, self).init()  # 부모 클래스 초기화
    self.setWindowTitle(“Robot Control Panel”)  # 창 제목 설정
    screen_geometry = QApplication.primaryScreen().geometry()  # 화면 해상도 가져오기
    available_geometry = QApplication.primaryScreen().availableGeometry()  # 사용 가능한 화면 해상도 가져오기
    screen_width = int(available_geometry.width() * 23 / 24)  # 화면 너비 계산
    screen_height = int(available_geometry.height() * 23 / 24)  # 화면 높이 계산
    self.setGeometry(0, 0, screen_width, screen_height)  # 창 크기 설정

    self.control_panel = ControlPanel(node, self)  # ControlPanel 객체 생성
    self.gpio_control = GPIOControl()  # GPIOControl 객체 생성

    main_layout = QVBoxLayout()  # 메인 레이아웃 세로로 설정
    main_layout.addWidget(self.control_panel)  # 메인 레이아웃에 ControlPanel 추가
    main_layout.addWidget(self.gpio_control)  # 메인 레이아웃에 GPIOControl 추가
    container = QWidget()  # 메인 컨테이너 위젯 생성
    container.setLayout(main_layout)  # 메인 컨테이너에 메인 레이아웃 설정
    self.setCentralWidget(container)  # 중앙 위젯으로 설정

    self.control_panel.map_label.setFixedSize(int(screen_width * 2 / 3), int(screen_height * 2 / 3))  # 맵 라벨 크기 설정
    self.control_panel.terminal_output.setFixedSize(int(screen_width * 2 / 3), int(screen_height * 1 / 3))  # 터미널 출력 크기 설정
    self.control_panel.setFixedSize(screen_width, screen_height)  # ControlPanel 크기 설정
    def main(args=None):
        rclpy.init(args=args)  # ROS 2 초기화
        node = Node(“robot_control_panel”)  # 노드 생성
        app = QApplication(sys.argv)  # QApplication 객체 생성
        main_window = MainWindow(node)  # MainWindow 객체 생성
        main_window.show()  # 창 표시

        try:
            sys.exit(app.exec_())  # 이벤트 루프 실행
        finally:
            rclpy.shutdown()  # ROS 2 종료
    
if name == “main”:
    main()  # 메인 함수 실행