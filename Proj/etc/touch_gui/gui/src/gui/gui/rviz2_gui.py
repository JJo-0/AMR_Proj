import subprocess
import sys
import time
import serial
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit, QGridLayout)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QMutex
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen, QColor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from omo_r1mini_interfaces.srv import Trigger
from geometry_msgs.msg import PoseStamped, Twist
from your_package_name.srv import SaveGoal
from threading import Thread, Lock

class SerialReader(QThread):
    new_data = pyqtSignal(str)

    def __init__(self, ser, lock):
        super().__init__()
        self.ser = ser
        self.lock = lock
        self.running = True

    def run(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                    self.new_data.emit(line)
            except serial.SerialException as e:
                self.new_data.emit(f"Serial error: {e}")
            time.sleep(0.1)

    def stop(self):
        self.running = False
        self.wait()

class ControlPanel(QWidget):
    def __init__(self, node, parent=None):
        super(ControlPanel, self).__init__(parent)
        self.node = node

        self.ser = None
        self.current_lift_command = None
        self.ems_signal = 1
        self.goal_positions = {}
        self.current_goal_index = None

        self.status_labels = {
            "EMS": QLabel(),
            "Lift": QLabel(),
            "Arduino": QLabel()
        }

        self.serial_buffer = []
        self.serial_lock = Lock()
        self.lift_command_timer = QTimer()
        self.lift_command_timer.timeout.connect(self.send_lift_command_periodic)

        self.init_ui()
        self.log_to_terminal("UI Set Success!")

        self.setup_serial_connection('/dev/ttyACM0', 115200)
        self.start_serial_read_thread()
        self.start_serial_process_thread()

        self.emergency_pub = self.node.create_publisher(Int32, '/ems_sig', 10)
        self.nav_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        self.create_goal_service()

        self.map_image = None
        self.map_mutex = QMutex()
        self.robot_pose = None

    def setup_serial_connection(self, port, baud_rate):
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
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        left_layout = QVBoxLayout()

        self.emergency_stop_button = QToolButton()
        self.emergency_stop_button.setCheckable(True)
        self.emergency_stop_button.setText("EMS")
        self.emergency_stop_button.setStyleSheet("font-size: 24px; height: 300px; background-color: lightcoral;")
        self.emergency_stop_button.setFixedWidth(100)

        self.emergency_stop_button.clicked.connect(self.handle_emergency_stop)
        left_layout.addWidget(self.emergency_stop_button)

        status_group = QGroupBox("Robot Status")
        status_layout = QVBoxLayout()

        for key, label in self.status_labels.items():
            label.setStyleSheet("font-size: 8px; background-color: white; color: white; padding: 5px;")
            status_layout.addWidget(QLabel(key))
            status_layout.addWidget(label)

        status_group.setLayout(status_layout)
        left_layout.addWidget(status_group)

        main_layout.addLayout(left_layout)

        right_layout = QVBoxLayout()

        self.exit_button = QPushButton("Exit")
        self.exit_button.setStyleSheet("font-size: 14px; height: 12px; background-color: gray; color: white;")
        self.exit_button.clicked.connect(self.exit_program)
        right_layout.addWidget(self.exit_button)

        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("background-color: black; color: white;")
        self.terminal_output.setFixedHeight(50)
        right_layout.addWidget(self.terminal_output)

        lift_group = QGroupBox("Lift Control")
        lift_layout = QHBoxLayout()
        lift_group.setLayout(lift_layout)

        lift_height_group = QVBoxLayout()
        self.height1_button = QPushButton("1 Height")
        self.height2_button = QPushButton("2 Height")
        self.height3_button = QPushButton("3 Height")
        self.height1_button.setStyleSheet("font-size: 18px; height: 40px; background-color: lightgrey;")
        self.height2_button.setStyleSheet("font-size: 18px; height: 40px; background-color: lightgrey;")
        self.height3_button.setStyleSheet("font-size: 18px; height: 40px; background-color: lightgrey;")
        self.height1_button.clicked.connect(lambda: self.send_lift_command("L_20", "1 Point"))
        self.height2_button.clicked.connect(lambda: self.send_lift_command("L_21", "2 Point"))
        self.height3_button.clicked.connect(lambda: self.send_lift_command("L_22", "3 Point"))
        lift_height_group.addWidget(self.height3_button)
        lift_height_group.addWidget(self.height2_button)
        lift_height_group.addWidget(self.height1_button)

        lift_updown_group = QVBoxLayout()
        self.lift_up_button = QPushButton("Lift Up")
        self.lift_down_button = QPushButton("Lift Down")
        self.lift_up_button.setStyleSheet("font-size: 20px; height: 60px; background-color: lightgrey;")
        self.lift_down_button.setStyleSheet("font-size: 20px; height: 60px; background-color: lightgrey;")
        self.lift_up_button.pressed.connect(lambda: self.start_lift_command("L_10", "Lift Up"))
        self.lift_up_button.released.connect(self.stop_lift_command)
        self.lift_down_button.pressed.connect(lambda: self.start_lift_command("L_11", "Lift Down"))
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
        self.save_goal_button_1.clicked.connect(lambda: self.save_nav_goal(1))
        self.save_goal_button_2.clicked.connect(lambda: self.save_nav_goal(2))
        self.save_goal_button_3.clicked.connect(lambda: self.save_nav_goal(3))
        self.save_goal_button_1.setStyleSheet("font-size: 14px; height: 30px; background-color: lightyellow;")
        self.save_goal_button_2.setStyleSheet("font-size: 14px; height: 30px; background-color: lightyellow;")
        self.save_goal_button_3.setStyleSheet("font-size: 14px; height: 30px; background-color: lightyellow;")
        save_goal_layout.addWidget(self.save_goal_button_1)
        save_goal_layout.addWidget(self.save_goal_button_2)
        save_goal_layout.addWidget(self.save_goal_button_3)

        # Go Goal Buttons
        go_goal_layout = QHBoxLayout()  # 가로 레이아웃
        self.go_goal_button_1 = QPushButton("Go Goal 1")
        self.go_goal_button_2 = QPushButton("Go Goal 2")
        self.go_goal_button_3 = QPushButton("Go Goal 3")
        self.go_goal_button_1.clicked.connect(lambda: self.go_nav_goal(1))
        self.go_goal_button_2.clicked.connect(lambda: self.go_nav_goal(2))
        self.go_goal_button_3.clicked.connect(lambda: self.go_nav_goal(3))
        self.go_goal_button_1.setStyleSheet("font-size: 14px; height: 30px; background-color: lightgreen;")
        self.go_goal_button_2.setStyleSheet("font-size: 14px; height: 30px; background-color: lightgreen;")
        self.go_goal_button_3.setStyleSheet("font-size: 14px; height: 30px; background-color: lightgreen;")
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
        move_layout.addWidget(self.left_button, 1, 0)
        move_layout.addWidget(self.stop_button, 1, 1)
        move_layout.addWidget(self.right_button, 1, 2)
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
        self.forward_button.setStyleSheet("font-size: 24px; height: 50px; background-color: black; color: white;")
        self.backward_button.setStyleSheet("font-size: 24px; height: 50px; background-color: black; color: white;")
        self.left_button.setStyleSheet("font-size: 24px; height: 50px; background-color: black; color: white;")
        self.right_button.setStyleSheet("font-size: 24px; height: 50px; background-color: black; color: white;")
        self.stop_button.setStyleSheet("font-size: 24px; height: 50px; background-color: black; color: white;")
        move_control_group.setLayout(move_layout)
        right_layout.addWidget(move_control_group)

        main_layout.addLayout(right_layout)

        self.update_status_label("EMS", "1", "green")
        self.update_status_label("Lift", "-", "black")
        self.update_status_label("Arduino", "E", "red")

    def send_lift_command(self, command, label):
        self.update_status_label("Lift", label, "green")
        if self.ser:
            try:
                self.ser.write(f"{command}\n".encode('utf-8'))
                self.log_to_terminal(f"[Arduino Send] : {command}")
                QTimer.singleShot(5000, lambda: self.update_status_label("Lift", "-", "black"))
            except serial.SerialException as e:
                self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")

    def send_lift_command_periodic(self):
        if self.current_lift_command:
            self.send_lift_command(*self.current_lift_command)

    def start_lift_command(self, command, label):
        self.current_lift_command = (command, label)
        self.send_lift_command(command, label)
        self.lift_command_timer.start(1000)  # 1초마다 주기적으로 실행

    def stop_lift_command(self):
        self.current_lift_command = None
        self.lift_command_timer.stop()
        self.update_status_label("Lift", "-", "black")

    def create_goal_service(self):
        self.save_goal_srv = self.node.create_service(SaveGoal, '/go_save_goal', self.save_goal_callback)

    def save_goal_callback(self, request, response):
        if self.robot_pose:
            x = self.robot_pose.position.x
            y = self.robot_pose.position.y
            z = self.robot_pose.orientation.z
            self.goal_positions[self.current_goal_index] = (x, y, z)
            response.success = True
            response.message = f"Goal {self.current_goal_index} saved: ({x}, {y}, {z})"
        else:
            response.success = False
            response.message = "No robot pose available"
        return response

    def save_nav_goal(self, goal_index):
        self.current_goal_index = goal_index
        client = self.node.create_client(SaveGoal, '/go_save_goal')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting again...')

        request = SaveGoal.Request()
        request.goal_number = goal_index
        future = client.call_async(request)
        future.add_done_callback(self.save_goal_response_callback)

    def save_goal_response_callback(self, future):
        try:
            response = future.result()
            self.log_to_terminal(response.message)
        except Exception as e:
            self.log_to_terminal(f'Service call failed: {str(e)}')

    def go_nav_goal(self, goal_index):
        client = self.node.create_client(SaveGoal, '/go_go_goal')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting again...')

        request = SaveGoal.Request()
        request.goal_number = goal_index
        future = client.call_async(request)
        future.add_done_callback(self.go_goal_response_callback)

    def go_goal_response_callback(self, future):
        try:
            response = future.result()
            self.log_to_terminal(response.message)
        except Exception as e:
            self.log_to_terminal(f'Service call failed: {str(e)}')

    def update_status_label(self, label_name, text, color):
        label = self.status_labels.get(label_name, None)
        if label:
            label.setText(f"{text}")
            label.setStyleSheet(f"font-size: 14px; padding: 5px; color: white; background-color: {color}; border-radius: 10px;")

    def start_serial_read_thread(self):
        if self.ser:
            self.read_thread = Thread(target=self.read_from_serial)
            self.read_thread.start()
            self.log_to_terminal("Serial Reading Thread Start")

    def start_serial_process_thread(self):
        self.process_thread = Thread(target=self.process_serial_buffer)
        self.process_thread.start()

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
                if status == 1:
                    self.emergency_pub.publish(Int32(data=1))
                    self.update_status_label("EMS", "1", "green")
                    self.emergency_stop_button.setChecked(False)
                elif status == 0:
                    self.emergency_pub.publish(Int32(data=0))
                    self.update_status_label("EMS", "0", "red")
                    self.emergency_stop_button.setChecked(True)
                self.log_to_terminal(f"Arduino received : EMS_{data}")
            except (ValueError, IndexError) as e:
                self.log_to_terminal(f"Invalid data received: {data}")

    def start_movement(self, direction):
        self.emergency_pub.publish(Int32(data=1))
        self.update_status_label("EMS", "1", "green")
        self.emergency_stop_button.setChecked(False)
        self.send_movement_command(direction)

    def stop_movement(self):
        self.emergency_pub.publish(Int32(data=0))
        self.update_status_label("EMS", "0", "red")
        self.emergency_stop_button.setChecked(True)
        self.send_movement_command("stop")

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

    def handle_emergency_stop(self):
        sender = self.sender()
        if sender.isChecked():
            self.emergency_pub.publish(Int32(data=0))
            self.update_status_label("EMS", "0", "red")
            self.ems_signal = 0
            if self.ser:
                try:
                    self.ser.write("E_0\n".encode('utf-8'))
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
                except serial.SerialException as e:
                    self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")

    def log_to_terminal(self, message):
        self.terminal_output.append(message)
        self.terminal_output.ensureCursorVisible()

    def exit_program(self):
        self.log_to_terminal("Exiting program...")
        QApplication.quit()

class MainWindow(QMainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Robot Control Panel")

        # 전체 화면 설정
        # self.showFullScreen()

        # 화면 해상도에 따라 메인 윈도우 크기 동적 조정
        screen_geometry = QApplication.primaryScreen().geometry()
        screen_width = screen_geometry.width()
        screen_height = screen_geometry.height()

        self.setGeometry(screen_width // 2, 0, screen_width // 2, screen_height * (9 / 10))

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
        config_path = "desktop/AMR_Proj/Proj/etc/touch_gui/touch_gui.rviz"  # RViz 설정 파일 경로
        subprocess.Popen(["rviz2", "-d", config_path])
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