"""
> /ems_sig 주석 되어 있음
"""
import sys
import serial
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit, QGridLayout, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QBrush, QPen, QColor, QPixmap
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from threading import Thread, Lock
import time
import numpy as np
import yaml
import os

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
        self.velocity = None
        self.imu_orientation = None
        self.slam_distance = None
        self.eta = None
        self.current_lift_command = None
        self.ems_signal = 1

        self.status_labels = {
            "EMS Signal": QLabel(),
            "Lift Signal": QLabel(),
            "Arduino Connection": QLabel()
        }

        self.serial_buffer = []
        self.serial_lock = Lock()

        self.init_ui()

        self.log_to_terminal("UI Set Success!")

        self.setup_serial_connection('/dev/ttyACM0', 115200)
        self.start_serial_read_thread()
        self.start_serial_process_thread()

        self.emergency_pub = self.node.create_publisher(Int32, '/ems_sig', 10)
        self.lift_pub = self.node.create_publisher(String, '/lift_command', 10)
        self.nav_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.velocity_sub = self.node.create_subscription(Odometry, '/odom', self.update_velocity, 10)
        self.imu_sub = self.node.create_subscription(Imu, '/imu', self.update_imu, 10)
        self.slam_sub = self.node.create_subscription(Float32, '/slam_remaining_distance', self.update_slam, 10)
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.update_position, 10)
        self.map_sub = self.node.create_subscription(OccupancyGrid, '/map', self.update_map, 10)

        self.load_map("/desktop/map_/map_.yaml")

    def setup_serial_connection(self, port, baud_rate):
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1)
            self.ser.reset_input_buffer()
            self.update_status_label("Arduino Connection", "Connected", "green")
            self.log_to_terminal(f"[Serial Connected] : {port} @ {baud_rate}")
        except serial.SerialException as e:
            self.update_status_label("Arduino Connection", "Error", "red")
            self.log_to_terminal(f"[Serial Connected Failed] : {str(e)}")
            self.ser = None

    def init_ui(self):
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        left_layout = QVBoxLayout()
        self.map_view = QGraphicsView()
        self.map_view.setStyleSheet("background-color: lightgray;")
        self.map_scene = QGraphicsScene()
        self.map_view.setScene(self.map_scene)
        self.robot_item = QGraphicsEllipseItem(-5, -5, 10, 10)
        self.robot_item.setBrush(QBrush(Qt.blue))
        self.map_scene.addItem(self.robot_item)
        left_layout.addWidget(self.map_view)

        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("background-color: black; color: white;")
        self.terminal_output.setFixedHeight(100)
        left_layout.addWidget(self.terminal_output)

        move_control_group = QGroupBox("Movement Control")
        move_layout = QGridLayout()
        move_control_group.setLayout(move_layout)

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

        self.emergency_stop_button = QToolButton()
        self.emergency_stop_button.setCheckable(True)
        self.emergency_stop_button.setText("EMS")
        self.emergency_stop_button.setStyleSheet("font-size: 24px; height: 100px;")
        self.emergency_stop_button.clicked.connect(self.handle_emergency_stop)
        left_control_layout = QHBoxLayout()
        left_control_layout.addWidget(move_control_group)
        left_control_layout.addWidget(self.emergency_stop_button)
        left_layout.addLayout(left_control_layout)

        main_layout.addLayout(left_layout)

        right_layout = QVBoxLayout()

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

        lift_updown_group = QGroupBox("Lift Up/Down")
        lift_updown_layout = QVBoxLayout()
        self.lift_up_button = QPushButton("Lift Up")
        self.lift_down_button = QPushButton("Lift Down")
        self.lift_up_button.setStyleSheet("font-size: 18px;")
        self.lift_down_button.setStyleSheet("font-size: 18px;")
        self.lift_up_button.clicked.connect(lambda: self.send_lift_command("L_10", "Lift Up"))
        self.lift_down_button.clicked.connect(lambda: self.send_lift_command("L_11", "Lift Down"))
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

        for key, label in self.status_labels.items():
            label.setStyleSheet("font-size: 14px; background-color: black; color: white; padding: 5px;")
            status_layout.addWidget(QLabel(key))
            status_layout.addWidget(label)

        self.update_status_label("EMS Signal", "-", "black")
        self.update_status_label("Lift Signal", "-", "black")
        self.update_status_label("Arduino Connection", "Disconnected", "black")
        status_group.setLayout(status_layout)
        right_layout.addWidget(status_group)
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

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

    def send_lift_command(self, command, label):
        self.update_status_label("Lift Signal", label, "green")
        if self.ser:
            try:
                self.ser.write(f"{command}\n".encode('utf-8'))
                self.log_to_terminal(f"[Arduino Send] : {command}")
                QTimer.singleShot(5000, lambda: self.update_status_label("Lift Signal", "-", "black"))
            except serial.SerialException as e:
                self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")

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
                    # self.emergency_pub.publish(Int32(data=1))
                    self.update_status_label("EMS Signal", "Good: 1", "green")
                    self.emergency_stop_button.setChecked(False)
                elif status == 0:
                    # self.emergency_pub.publish(Int32(data=0))
                    self.update_status_label("EMS Signal", "Emergency: 0", "red")
                    self.emergency_stop_button.setChecked(True)
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
        # self.emergency_pub.publish(Int32(data=1))
        self.update_status_label("EMS Signal", "1 : Good", "green")
        self.emergency_stop_button.setChecked(False)
        self.send_movement_command(direction)

    def stop_movement(self):
        # self.emergency_pub.publish(Int32(data=0))
        self.update_status_label("EMS Signal", "0 : Emergency", "red")
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
            # self.emergency_pub.publish(Int32(data=0))
            self.update_status_label("EMS Signal", "0 : Emergency", "red")
            self.ems_signal = 0
            if self.ser:
                try:
                    self.ser.write("E_0\n".encode('utf-8'))
                except serial.SerialException as e:
                    self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")
        else:
            # self.emergency_pub.publish(Int32(data=1))
            self.update_status_label("EMS Signal", "1 : Good", "green")
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

    def update_position(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_item.setPos(x * 100, y * 100)  # Assuming 1 unit = 1 meter and scaling by 100 for visibility
        self.log_to_terminal(f"Update Position: x={x}, y={y}")

    def update_map(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.map_scene.clear()
        self.map_scene.addItem(self.robot_item)

        map_data = np.array(msg.data).reshape((height, width))
        for y in range(height):
            for x in range(width):
                if map_data[y, x] == 0:
                    color = QColor(255, 255, 255)
                elif map_data[y, x] == 100:
                    color = QColor(0, 0, 0)
                else:
                    color = QColor(150, 150, 150)
                rect = self.map_scene.addRect(x * resolution * 100, -y * resolution * 100, resolution * 100, resolution * 100, QPen(color), QBrush(color))

        self.map_scene.setSceneRect(origin_x * 100, -origin_y * 100, width * resolution * 100, height * resolution * 100)

    def load_map(self, yaml_file):
        with open(yaml_file, 'r') as file:
            map_info = yaml.safe_load(file)
            image_path = os.path.abspath(map_info['image'])  # 절대 경로로 변환
            resolution = map_info['resolution']
            origin = map_info['origin']

            if not os.path.exists(image_path):
                self.log_to_terminal(f"Image file does not exist: {image_path}")
                return

            image = QPixmap(image_path)
            if image.isNull():
                self.log_to_terminal(f"Failed to load image from {image_path}")
                return

            self.map_scene.clear()
            self.map_scene.addPixmap(image)

            self.map_scene.setSceneRect(origin[0], origin[1], image.width() * resolution, image.height() * resolution)
            self.map_scene.addItem(self.robot_item)

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