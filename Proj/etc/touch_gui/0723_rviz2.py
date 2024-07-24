import sys
import serial
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit, QGridLayout, QScrollArea)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QMutex
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen, QColor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from threading import Thread, Lock
import time

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
        self.map_sub = self.node.create_subscription(OccupancyGrid, '/map', self.update_map, 10)
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.update_robot_position, 10)

        self.map_image = None
        self.map_mutex = QMutex()
        self.robot_pose = None

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
        scroll_area = QScrollArea()

        self.map_label = QLabel()
        self.map_label.setAlignment(Qt.AlignCenter)
        scroll_area.setWidget(self.map_label)
        left_layout.addWidget(scroll_area)

        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("background-color: black; color: white;")
        self.terminal_output.setFixedHeight(50)
        left_layout.addWidget(self.terminal_output)

        move_control_group = QGroupBox("Movement Control")
        move_layout = QGridLayout()
        move_control_group.setLayout(move_layout)
        move_control_group.setFixedHeight(200)

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
        self.forward_button.setStyleSheet("font-size: 24px; height: 50px;")
        self.backward_button.setStyleSheet("font-size: 24px; height: 50px;")
        self.left_button.setStyleSheet("font-size: 24px; height: 50px;")
        self.right_button.setStyleSheet("font-size: 24px; height: 50px;")
        self.stop_button.setStyleSheet("font-size: 24px; height: 50px;")

        self.emergency_stop_button = QToolButton()
        self.emergency_stop_button.setCheckable(True)
        self.emergency_stop_button.setText("EMS")
        self.emergency_stop_button.setStyleSheet("font-size: 24px; height: 100px;")
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

        self.height1_button = QPushButton("1 Height")
        self.height2_button = QPushButton("2 Height")
        self.height3_button = QPushButton("3 Height")
        self.height1_button.setStyleSheet("font-size: 24px; height: 50px;")
        self.height2_button.setStyleSheet("font-size: 24px; height: 50px;")
        self.height3_button.setStyleSheet("font-size: 24px; height: 50px;")
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
        self.lift_up_button.setStyleSheet("font-size: 24px; height: 50px;")
        self.lift_down_button.setStyleSheet("font-size: 24px; height: 50px;")
        self.lift_up_button.clicked.connect(lambda: self.send_lift_command("L_10", "Lift Up"))
        self.lift_down_button.clicked.connect(lambda: self.send_lift_command("L_11", "Lift Down"))
        lift_updown_layout.addWidget(self.lift_up_button)
        lift_updown_layout.addWidget(self.lift_down_button)
        lift_updown_group.setLayout(lift_updown_layout)

        right_layout.addWidget(lift_updown_group)

        status_group = QGroupBox("Robot Status")
        status_layout = QVBoxLayout()

        for key, label in self.status_labels.items():
            label.setStyleSheet("font-size: 14px; background-color: black; color: white; padding: 5px;")
            status_layout.addWidget(QLabel(key))
            status_layout.addWidget(label)

        self.update_status_label("EMS Signal", "Good: 1", "green")
        self.update_status_label("Lift Signal", "-", "black")
        self.update_status_label("Arduino Connection", "Disconnected", "black")

        status_group.setLayout(status_layout)
        right_layout.addWidget(status_group)
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

    def update_map(self, msg):
        width, height = msg.info.width, msg.info.height
        data = np.array(msg.data).reshape((height, width))
        data = np.uint8((data == 0) * 255)  # Occupied cells are black (0), free cells are white (255)

        qimage = QImage(data, width, height, QImage.Format_Grayscale8)
        pixmap = QPixmap.fromImage(qimage)
        self.map_mutex.lock()
        self.map_image = pixmap
        self.map_mutex.unlock()
        self.update_map_display()

    def update_robot_position(self, msg):
        self.robot_pose = msg.pose.pose
        self.update_map_display()

    def update_map_display(self):
        self.map_mutex.lock()
        if self.map_image:
            pixmap = self.map_image.copy()
            painter = QPainter(pixmap)
            painter.setPen(QPen(Qt.red, 5, Qt.SolidLine))

            if self.robot_pose:
                # Transform the robot position to map coordinates
                resolution = 0.05  # example resolution, use actual map resolution
                origin_x, origin_y = 0, 0  # example origin, use actual map origin

                x = int((self.robot_pose.position.x - origin_x) / resolution)
                y = int((self.robot_pose.position.y - origin_y) / resolution)
                y = pixmap.height() - y  # Invert y axis

                painter.drawEllipse(x - 5, y - 5, 10, 10)  # Draw the robot position as a circle

            painter.end()
            self.map_label.setPixmap(pixmap)
        self.map_mutex.unlock()

    def send_nav_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = z
        goal.pose.orientation.w = 1.0
        self.nav_pub.publish(goal)
        self.log_to_terminal(f"Navigation goal sent: ({x}, {y}, {z})")

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
                    self.emergency_pub.publish(Int32(data=1))
                    self.update_status_label("EMS Signal", "Good: 1", "green")
                    self.emergency_stop_button.setChecked(False)
                elif status == 0:
                    self.emergency_pub.publish(Int32(data=0))
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
        self.emergency_pub.publish(Int32(data=1))
        self.update_status_label("EMS Signal", "1 : Good", "green")
        self.emergency_stop_button.setChecked(False)
        self.send_movement_command(direction)

    def stop_movement(self):
        self.emergency_pub.publish(Int32(data=0))
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
            self.emergency_pub.publish(Int32(data=0))
            self.update_status_label("EMS Signal", "0 : Emergency", "red")
            self.ems_signal = 0
            if self.ser:
                try:
                    self.ser.write("E_0\n".encode('utf-8'))
                except serial.SerialException as e:
                    self.log_to_terminal(f"[Arduino Sending Error] : {str(e)}")
        else:
            self.emergency_pub.publish(Int32(data=1))
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

class MainWindow(QMainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Robot Control Panel")

        # 전체 화면 설정
        self.showFullScreen()

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
