import os  # 파일 경로 처리를 위한 모듈
import subprocess  # 서브프로세스를 실행하기 위한 모듈
import sys  # 시스템 관련 기능을 위한 모듈
import time  # 시간 관련 기능을 위한 모듈
import serial  # 시리얼 통신을 위한 모듈
import json  # JSON 파일 처리를 위한 모듈
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit, QGridLayout)  # PyQt5 GUI 위젯들
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QMutex, QCoreApplication, QProcess  # PyQt5 코어 기능들
import rclpy  # ROS 2 파이썬 라이브러리
from rclpy.node import Node  # ROS 2 노드 클래스
from rclpy.action import ActionClient 
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32, String  # ROS 2 표준 메시지
from geometry_msgs.msg import PoseStamped, Twist  # ROS 2 지오메트리 메시지
from threading import Thread, Lock  # 스레딩 및 락

class SerialReader(QThread):
    """시리얼 데이터 읽기용 스레드 클래스"""
    new_data = pyqtSignal(str)  # 새로운 데이터를 받았음을 알리는 시그널

    def __init__(self, ser, lock):
        super().__init__()
        self.ser = ser  # 시리얼 포트 객체
        self.lock = lock  # 락 객체
        self.running = True  # 스레드 실행 상태

    def run(self):
        """스레드 실행 함수"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()  # 시리얼 데이터 읽기
                    self.new_data.emit(line)  # 새로운 데이터 시그널 발생
            except serial.SerialException as e:
                self.new_data.emit(f"Serial error: {e}")  # 시리얼 예외 처리
            time.sleep(0.1)  # 0.1초 대기

    def stop(self):
        """스레드 중지 함수"""
        self.running = False
        self.wait()

class ControlPanel(QWidget):
    """로봇 제어 패널 위젯 클래스"""
    def __init__(self, node, parent=None):
        super(ControlPanel, self).__init__(parent)
        self.node = node  # ROS 2 노드

        self.rviz_process = None  # RViz 프로세스 객체를 저장할 변수

        self.ser = None  # 시리얼 포트 객체
        self.current_lift_command = None  # 현재 리프트 명령
        self.ems_signal = 1  # 응급 정지 신호
        self.goal_positions = {"goal_1": None, "goal_2": None, "goal_3": None}  # 목표 위치들
        self.current_goal_index = None  # 현재 목표 인덱스

        self.status_labels = {
            "EMS": QLabel(),
            "Lift": QLabel(),
            "Arduino": QLabel()
        }  # 상태 레이블들

        self.serial_buffer = []  # 시리얼 버퍼
        self.serial_lock = Lock()  # 시리얼 락
        self.lift_command_timer = QTimer()  # 리프트 명령 타이머
        self.lift_command_timer.timeout.connect(self.send_lift_command_periodic)

        self.init_ui()  # UI 초기화
        self.log_to_terminal("UI Set Success!")

        self.setup_serial_connection('/dev/ttyACM0', 115200)  # 시리얼 연결 설정
        self.start_serial_read_thread()  # 시리얼 읽기 스레드 시작
        self.start_serial_process_thread()  # 시리얼 처리 스레드 시작

        self.emergency_pub = self.node.create_publisher(Int32, '/ems_sig', 10)  # 응급 정지 퍼블리셔
        self.goal_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)  # 네비게이션 퍼블리셔
        self.pose_pub = self.node.create_publisher(Int32, '/pose_cmd', 10)  # /pose_cmd 퍼블리셔
        
        self.start_ros2_launch()

    def setup_serial_connection(self, port, baud_rate):
        """시리얼 연결 설정"""
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1)
            self.ser.reset_input_buffer()
            self.update_status_label("Arduino", "C", "green")
            self.log_to_terminal(f"[Serial Connected] : {port} @ {baud_rate}")
        except serial.SerialException as e:
            self.update_status_label("Arduino", "E", "red")
            self.log_to_terminal(f"[Serial Connected Failed] : {str(e)}")
            self.ser = None

    def init_ui(self):
        """UI 초기화"""
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        left_layout = QVBoxLayout()

        self.emergency_stop_button = QToolButton()
        self.emergency_stop_button.setCheckable(True)
        self.emergency_stop_button.setText("EMS")
        self.emergency_stop_button.setStyleSheet("font-size: 40px; height: 300px; background-color: red; border-radius: 10px; font-weight: bold;")
        self.emergency_stop_button.setFixedWidth(100)
        self.emergency_stop_button.clicked.connect(self.handle_emergency_stop)
        left_layout.addWidget(self.emergency_stop_button)

        status_group = QGroupBox("Robot Status")
        status_layout = QVBoxLayout()

        for key, label in self.status_labels.items():
            label.setStyleSheet("font-size: 20px; background-color: white; color: white; padding: 5px; border-radius: 10px; font-weight: bold;")
            status_layout.addWidget(QLabel(key))
            status_layout.addWidget(label)

        status_group.setLayout(status_layout)
        left_layout.addWidget(status_group)

        self.spin_button = QPushButton("Spin")
        self.spin_button.clicked.connect(lambda: self.publish_pose_cmd(0))
        self.spin_button.setStyleSheet("font-size: 20px; height: 30px; background-color: lightgray; border-radius: 10px;")
        left_layout.addWidget(self.spin_button)

        main_layout.addLayout(left_layout)

        right_layout = QVBoxLayout()

        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("background-color: lightgrey; color: black;")
        self.terminal_output.setFixedHeight(20)
        right_layout.addWidget(self.terminal_output)

        self.nav_terminal_output = QTextEdit()
        self.nav_terminal_output.setReadOnly(True)
        self.nav_terminal_output.setStyleSheet("background-color: lightgrey; color: black; font-weight: bold;")
        self.nav_terminal_output.setFixedHeight(150)
        right_layout.addWidget(self.nav_terminal_output)

        lift_group = QGroupBox("Lift Control")
        lift_layout = QHBoxLayout()
        lift_group.setLayout(lift_layout)

        lift_height_group = QVBoxLayout()
        self.height1_button = QPushButton("1 Height")
        self.height2_button = QPushButton("2 Height")
        self.height3_button = QPushButton("3 Height")
        self.height1_button.setStyleSheet("font-size: 30px; height: 60px; background-color: lightgrey; border-radius: 10px;")
        self.height2_button.setStyleSheet("font-size: 30px; height: 60px; background-color: lightgrey; border-radius: 10px;")
        self.height3_button.setStyleSheet("font-size: 30px; height: 60px; background-color: lightgrey; border-radius: 10px;")
        self.height1_button.clicked.connect(lambda: self.send_lift_command("L_20", "1"))
        self.height2_button.clicked.connect(lambda: self.send_lift_command("L_21", "2"))
        self.height3_button.clicked.connect(lambda: self.send_lift_command("L_22", "3"))
        lift_height_group.addWidget(self.height3_button)
        lift_height_group.addWidget(self.height2_button)
        lift_height_group.addWidget(self.height1_button)

        lift_updown_group = QVBoxLayout()
        self.lift_up_button = QPushButton("Lift Up")
        self.lift_down_button = QPushButton("Lift Down")
        self.lift_up_button.setStyleSheet("font-size: 30px; height: 90px; background-color: lightgrey; border-radius: 10px;")
        self.lift_down_button.setStyleSheet("font-size: 30px; height: 90px; background-color: lightgrey; border-radius: 10px;")
        self.lift_up_button.pressed.connect(lambda: self.start_lift_command("L_10", "Up"))
        self.lift_up_button.released.connect(self.stop_lift_command)
        self.lift_down_button.pressed.connect(lambda: self.start_lift_command("L_11", "Down"))
        self.lift_down_button.released.connect(self.stop_lift_command)
        lift_updown_group.addWidget(self.lift_up_button)
        lift_updown_group.addWidget(self.lift_down_button)

        lift_layout.addLayout(lift_height_group)
        lift_layout.addLayout(lift_updown_group)

        right_layout.addWidget(lift_group)

        nav_group = QGroupBox("Navigation Goals")
        nav_layout = QVBoxLayout()

        save_goal_layout = QHBoxLayout()  # 가로 레이아웃
        self.save_goal_button_1 = QPushButton("Save Goal 1")
        self.save_goal_button_2 = QPushButton("Save Goal 2")
        self.save_goal_button_3 = QPushButton("Save Goal 3")
        self.save_goal_button_1.clicked.connect(lambda: self.publish_pose_cmd(1))
        self.save_goal_button_2.clicked.connect(lambda: self.publish_pose_cmd(2))
        self.save_goal_button_3.clicked.connect(lambda: self.publish_pose_cmd(3))
        self.save_goal_button_1.setStyleSheet("font-size: 30px; height: 90px; background-color: lightyellow; border-radius: 10px; font-weight: bold;")
        self.save_goal_button_2.setStyleSheet("font-size: 30px; height: 90px; background-color: lightyellow; border-radius: 10px; font-weight: bold;")
        self.save_goal_button_3.setStyleSheet("font-size: 30px; height: 90px; background-color: lightyellow; border-radius: 10px; font-weight: bold;")
        save_goal_layout.addWidget(self.save_goal_button_1)
        save_goal_layout.addWidget(self.save_goal_button_2)
        save_goal_layout.addWidget(self.save_goal_button_3)

        # Go Goal Buttons
        go_goal_layout = QHBoxLayout()  # 가로 레이아웃
        self.go_goal_button_1 = QPushButton("Go Goal 1")
        self.go_goal_button_2 = QPushButton("Go Goal 2")
        self.go_goal_button_3 = QPushButton("Go Goal 3")
        self.go_goal_button_1.clicked.connect(lambda: self.publish_pose_cmd(4))
        self.go_goal_button_2.clicked.connect(lambda: self.publish_pose_cmd(5))
        self.go_goal_button_3.clicked.connect(lambda: self.publish_pose_cmd(6))
        self.go_goal_button_1.setStyleSheet("font-size: 30px; height: 90px; background-color: lightgreen; border-radius: 10px; font-weight: bold;")
        self.go_goal_button_2.setStyleSheet("font-size: 30px; height: 90px; background-color: lightgreen; border-radius: 10px; font-weight: bold;")
        self.go_goal_button_3.setStyleSheet("font-size: 30px; height: 90px; background-color: lightgreen; border-radius: 10px; font-weight: bold;")
        go_goal_layout.addWidget(self.go_goal_button_1)
        go_goal_layout.addWidget(self.go_goal_button_2)
        go_goal_layout.addWidget(self.go_goal_button_3)

        nav_layout.addLayout(save_goal_layout)
        nav_layout.addLayout(go_goal_layout)

        nav_group.setLayout(nav_layout)
        right_layout.addWidget(nav_group)

        move_control_group = QGroupBox("Movement Control")
        move_layout = QGridLayout()
        self.forward_button = QPushButton("Forward")
        self.backward_button = QPushButton("Backward")
        self.left_button = QPushButton("Left")
        self.right_button = QPushButton("Right")
        self.stop_button = QPushButton("Stop")
        move_layout.addWidget(self.backward_button, 0, 1)
        move_layout.addWidget(self.left_button, 1, 2)
        move_layout.addWidget(self.stop_button, 1, 1)
        move_layout.addWidget(self.right_button, 1, 0)
        move_layout.addWidget(self.forward_button, 2, 1)
        self.forward_button.pressed.connect(lambda: self.start_movement("forward"))
        self.forward_button.released.connect(self.stop_movement)
        self.backward_button.pressed.connect(lambda: self.start_movement("backward"))
        self.backward_button.released.connect(self.stop_movement)
        self.left_button.pressed.connect(lambda: self.start_movement("left"))
        self.left_button.released.connect(self.stop_movement)
        self.right_button.pressed.connect(lambda: self.start_movement("right"))
        self.right_button.released.connect(self.stop_movement)
        self.stop_button.clicked.connect(lambda: self.send_movement_command("stop"))
        self.forward_button.setStyleSheet("font-size: 24px; height: 70px; background-color: black; color: white; border-radius: 10px; font-weight: bold;")
        self.backward_button.setStyleSheet("font-size: 24px; height: 70px; background-color: black; color: white; border-radius: 10px; font-weight: bold;")
        self.left_button.setStyleSheet("font-size: 24px; height: 70px; background-color: black; color: white; border-radius: 10px; font-weight: bold;")
        self.right_button.setStyleSheet("font-size: 24px; height: 70px; background-color: black; color: white; border-radius: 10px; font-weight: bold;")
        self.stop_button.setStyleSheet("font-size: 24px; height: 70px; background-color: black; color: white; border-radius: 10px; font-weight: bold;")
        move_control_group.setLayout(move_layout)
        right_layout.addWidget(move_control_group)

        main_layout.addLayout(right_layout)

        self.update_status_label("EMS", "1", "green")
        self.update_status_label("Lift", "-", "black")
        self.update_status_label("Arduino", "E", "red")

    def publish_pose_cmd(self, cmd_value):
        """/pose_cmd 토픽으로 명령을 퍼블리시"""
        msg = Int32()
        msg.data = cmd_value
        self.pose_pub.publish(msg)
        if cmd_value == 0 :
            self.log_to_terminal(f"Spinning: {cmd_value}")
        elif cmd_value <= 3 :
            self.log_to_terminal(f"Saved Goal: {cmd_value}")
        else : 
            self.log_to_terminal(f"Go Goal Set: {cmd_value - 2}")

    def send_lift_command(self, command, label):
        """리프트 명령 전송"""
        self.update_status_label("Lift", label, "blue")
        if self.ser:
            try:
                self.ser.write(f"{command}\n".encode('utf-8'))
                self.log_to_terminal(f"[Arduino Send] : {command}")
                QTimer.singleShot(5000, lambda: self.update_status_label("Lift", "-", "black"))
            except serial.SerialException as e:
                self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")

    def send_lift_command_periodic(self):
        """주기적으로 리프트 명령 전송"""
        if self.current_lift_command:
            self.send_lift_command(*self.current_lift_command)

    def start_lift_command(self, command, label):
        """리프트 명령 시작"""
        self.current_lift_command = (command, label)
        self.send_lift_command(command, label)
        self.lift_command_timer.start(500)  # 0.5초마다 주기적으로 실행

    def stop_lift_command(self):
        """리프트 명령 중지"""
        self.current_lift_command = None
        self.lift_command_timer.stop()
        self.update_status_label("Lift", "-", "black")

    def get_current_height_from_arduino(self):
        """아두이노로부터 현재 높이 값 가져오기"""
        if self.ser:
            try:
                self.ser.write("A_00\n".encode('utf-8'))  # 높이 요청 명령 전송
                time.sleep(0.5)  # 응답 대기 시간
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                    if line.startswith("A_"):
                        height = int(line.split("_")[1])  # 높이 값 파싱
                        return height
            except serial.SerialException as e:
                self.log_to_terminal(f"[Arduino Reading Error] : {str(e)}")
        return None

    def update_status_label(self, label_name, text, color):
        """상태 레이블 업데이트"""
        label = self.status_labels.get(label_name, None)
        if label:
            label.setText(f"{text}")
            label.setStyleSheet(f"font-size: 20px; padding: 5px; color: white; background-color: {color}; border-radius: 10px; font-weight: bold;")
            label.setAlignment(Qt.AlignCenter)

    def start_serial_read_thread(self):
        """시리얼 읽기 스레드 시작"""
        if self.ser:
            self.read_thread = Thread(target=self.read_from_serial)
            self.read_thread.start()
            self.log_to_terminal("Aduino Serial Reading Thread Start")

    def start_serial_process_thread(self):
        """시리얼 처리 스레드 시작"""
        self.process_thread = Thread(target=self.process_serial_buffer)
        self.process_thread.start()

    def read_from_serial(self):
        """시리얼 데이터 읽기"""
        while True:
            if self.ser and self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                with self.serial_lock:
                    self.serial_buffer.append(line)

    def process_serial_buffer(self):
        """시리얼 버퍼 처리"""
        while True:
            with self.serial_lock:
                if self.serial_buffer:
                    data = self.serial_buffer.pop(0)
                    self.process_serial_data(data)
            time.sleep(0.1)

    def process_serial_data(self, data):
        """시리얼 데이터 처리"""
        if data.startswith("E_"):
            try:
                status = int(data.split("_")[1])
                prev_ems_signal = self.ems_signal
                self.ems_signal = status
                if status == 0:
                    self.emergency_pub.publish(Int32(data=0))
                    self.update_status_label("EMS", str(status), "red")
                    self.emergency_stop_button.setChecked(True)

                elif status == 1:
                    self.emergency_pub.publish(Int32(data=1))
                    self.update_status_label("EMS", "1", "green")
                    self.emergency_stop_button.setChecked(False)

                elif status == 2:
                    if (prev_ems_signal == 0): 
                        self.ems_signal = 0
                    self.emergency_pub.publish(Int32(data=0))
                    self.update_status_label("EMS", str(status), "red")
                    self.emergency_stop_button.setChecked(True)

                elif status == 3:
                    if prev_ems_signal == 2: 
                        self.emergency_pub.publish(Int32(data=1))
                        self.update_status_label("EMS", "1", "green")
                        self.emergency_stop_button.setChecked(False)
                        if self.ser:
                            try:
                                self.ser.write("E_1\n".encode('utf-8'))
                                self.log_to_terminal(f"[Arduino Send] : E_1")
                            except serial.SerialException as e:
                                self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")                
                self.log_to_terminal(f"Arduino received : EMS_{data}")
            except (ValueError, IndexError) as e:
                self.log_to_terminal(f"Invalid data received: {data}")

    def start_movement(self, direction):
        """로봇 이동 시작"""
        self.emergency_pub.publish(Int32(data=1))
        self.update_status_label("EMS", "1", "green")
        self.emergency_stop_button.setChecked(False)
        self.emergency_stop_button.setStyleSheet("font-size: 40px; height: 300px; background-color: red; border-radius: 10px; font-weight: bold;")
        self.emergency_stop_button.repaint()
        self.send_movement_command(direction)

    def stop_movement(self):
        """로봇 이동 중지"""
        self.emergency_pub.publish(Int32(data=0))
        self.update_status_label("EMS", "0", "red")
        self.emergency_stop_button.setChecked(True)
        self.emergency_stop_button.setStyleSheet("font-size: 40px; height: 300px; background-color: darkred; border-radius: 10px; font-weight: bold;")
        self.emergency_stop_button.repaint()
        self.send_movement_command("stop")

    def send_movement_command(self, direction):
        """이동 명령 전송"""
        msg = Twist()
        if direction == "forward":
            msg.linear.x = 0.25
        elif direction == "backward":
            msg.linear.x = -0.25
        elif direction == "left":
            msg.angular.z = 0.4
        elif direction == "right":
            msg.angular.z = -0.4
        elif direction == "stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.node.create_publisher(Twist, '/cmd_vel', 10).publish(msg)

    def handle_emergency_stop(self):
        """응급 정지 처리"""
        sender = self.sender()
        if sender.isChecked():
            self.emergency_pub.publish(Int32(data=0))
            self.update_status_label("EMS", "0", "red")
            self.ems_signal = 0
            if self.ser:
                try:
                    self.ser.write("E_0\n".encode('utf-8'))
                    self.emergency_stop_button.setStyleSheet("font-size: 40px; height: 300px; background-color: darkred; border-radius: 10px; font-weight: bold;")
                    self.emergency_stop_button.repaint()
                except serial.SerialException as e:
                    self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")
        else:
            self.emergency_pub.publish(Int32(data=1))
            self.update_status_label("EMS", "1", "green")
            self.ems_signal = 1
            if self.ser:
                try:
                    self.ser.write("E_1\n".encode('utf-8'))
                    self.log_to_terminal(f"[Arduino Send] : E_1")
                    self.emergency_stop_button.setStyleSheet("font-size: 40px; height: 300px; background-color: red; border-radius: 10px; font-weight: bold;")
                    self.emergency_stop_button.repaint()
                except serial.SerialException as e:
                    self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")

    def log_to_terminal(self, message):
        """터미널 로그"""
        self.terminal_output.append(message)
        self.terminal_output.ensureCursorVisible()

    def start_ros2_launch(self):
        """ros2 launch omo_r1mini_navigation2 navigation2.launch.py 실행 및 로그 출력"""
        self.process = QProcess()
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.handle_ros2_output)
        self.process.start("ros2", ["launch", "omo_r1mini_navigation2", "navigation2.launch.py"])

    def handle_ros2_output(self):
        data = self.process.readAllStandardOutput().data().decode()
        self.nav_terminal_output.append(data)
        self.nav_terminal_output.ensureCursorVisible()

class MainWindow(QMainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Robot Control Panel")

        # 화면 해상도에 따라 메인 윈도우 크기 동적 조정
        screen_geometry = QApplication.primaryScreen().geometry()
        screen_width = screen_geometry.width()
        screen_height = screen_geometry.height()
        self.setGeometry(screen_width // 2, 0, screen_width // 2, screen_height)
        
        # # 전체 화면
        # self.showFullScreen()

        # 컨트롤 패널 추가 및 크기 조정
        self.control_panel = ControlPanel(node, self)

        # 메인 레이아웃 설정
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.control_panel)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # RViz 실행
        self.launch_rviz()

    def launch_rviz(self):
        """RViz 실행"""
        config_path = "/desktop/AMR_Proj/Proj/etc/touch_gui/my_rviz.rviz"  # RViz 설정 파일 경로
        env = os.environ.copy()
        env['LIBGL_ALWAYS_SOFTWARE'] = '1'
        self.control_panel.rviz_process = subprocess.Popen(["rviz2", "-d", config_path], env=env)  # RViz 프로세스를 시작하고 객체를 저장
        time.sleep(5)  # RViz 창이 뜰 시간을 줌

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
