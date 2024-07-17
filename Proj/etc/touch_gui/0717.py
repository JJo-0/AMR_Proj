import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QToolButton, QLabel, QGroupBox, QTextEdit
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gpiozero import DigitalOutputDevice, DistanceSensor
from threading import Thread
import time

class ControlPanel(QWidget):
    def __init__(self, node, parent=None):
        super(ControlPanel, self).__init__(parent)
        self.node = node

        # GPIO 초기화
        self.initGPIO()
        self.startDistanceThread()

        # 메인 레이아웃 설정
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()

        self.map_label = QLabel("Map Area")
        self.map_label.setStyleSheet("background-color: lightgray;")
        left_layout.addWidget(self.map_label)

        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("background-color: black; color: white;")
        left_layout.addWidget(self.terminal_output)

        right_layout = QVBoxLayout()

        lift_group = QGroupBox("Lift Control")
        lift_layout = QVBoxLayout()
        self.lift_up_button = QPushButton("Lift Up")
        self.lift_up_button.setStyleSheet("font-size: 24px; height: 100px;")
        self.lift_down_button = QPushButton("Lift Down")
        self.lift_down_button.setStyleSheet("font-size: 24px; height: 100px;")
        self.distance_label = QLabel("Distance: -- cm")
        lift_layout.addWidget(self.lift_up_button)
        lift_layout.addWidget(self.lift_down_button)
        lift_layout.addWidget(self.distance_label)
        lift_group.setLayout(lift_layout)
        right_layout.addWidget(lift_group)

        nav_group = QGroupBox("Navigation")
        nav_layout = QVBoxLayout()
        self.toggle_nav_button = QToolButton()
        self.toggle_nav_button.setCheckable(True)
        self.toggle_nav_button.setText("Set Navigation Goal")
        self.toggle_nav_button.setStyleSheet("font-size: 24px; height: 100px;")
        nav_layout.addWidget(self.toggle_nav_button)
        nav_group.setLayout(nav_layout)
        right_layout.addWidget(nav_group)

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

        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

        self.setLayout(main_layout)

        self.lift_up_button.pressed.connect(self.start_lift_up)
        self.lift_up_button.released.connect(self.stop_lift)
        self.lift_down_button.pressed.connect(self.start_lift_down)
        self.lift_down_button.released.connect(self.stop_lift)
        self.toggle_nav_button.clicked.connect(self.toggle_navigation)

        self.lift_pub = self.node.create_publisher(String, '/lift_command', 10)
        self.nav_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.status_sub = self.node.create_subscription(String, '/robot_status', self.update_status, 10)
        self.velocity_sub = self.node.create_subscription(Odometry, '/odom', self.update_velocity, 10)
        self.imu_sub = self.node.create_subscription(Imu, '/imu', self.update_imu, 10)
        self.slam_sub = self.node.create_subscription(Float32, '/slam_remaining_distance', self.update_slam, 10)

        self.velocity = None
        self.imu_orientation = None
        self.slam_distance = None
        self.eta = None

        self.lift_timer = QTimer()
        self.lift_timer.timeout.connect(self.send_lift_command)
        self.current_lift_command = None

    def initGPIO(self):
        self.pin23 = DigitalOutputDevice(23)
        self.pin24 = DigitalOutputDevice(24)
        self.sensor = DistanceSensor(echo=27, trigger=17)
        self.distanceThreadActive = True

    def start_lift_up(self):
        self.log_to_terminal("Start Lift Up")
        self.current_lift_command = "up"
        self.lift_timer.start(100)
        self.update_distance_label()

    def start_lift_down(self):
        self.log_to_terminal("Start Lift Down")
        self.current_lift_command = "down"
        self.lift_timer.start(100)
        self.update_distance_label()

    def stop_lift(self):
        self.log_to_terminal("Stop Lift")
        self.current_lift_command = None
        self.lift_timer.stop()

    def send_lift_command(self):
        if self.current_lift_command:
            msg = String()
            msg.data = self.current_lift_command
            self.lift_pub.publish(msg)
            if self.current_lift_command in ["up", "down"]:
                self.update_distance_label()

    def update_distance_label(self):
        distance = self.sensor.distance * 100
        self.distance_label.setText(f"Distance: {distance:.2f} cm")

    def startDistanceThread(self):
        self.distanceThread = Thread(target=self.updateDistance)
        self.distanceThread.start()

    def updateDistance(self):
        while self.distanceThreadActive:
            distance = self.sensor.distance * 100
            self.distance_label.setText(f"Distance: {distance:.2f} cm")
            time.sleep(0.1)

    def closeEvent(self, event):
        self.distanceThreadActive = False
        if hasattr(self, 'distanceThread'):
            self.distanceThread.join()
        self.pin23.close()
        self.pin24.close()
        self.sensor.close()
        event.accept()

    def toggle_navigation(self):
        nav_state = "enabled" if self.toggle_nav_button.isChecked() else "disabled"
        self.log_to_terminal(f"Navigation {nav_state}")

    def update_status(self, msg):
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

    def log_to_terminal(self, message):
        self.terminal_output.append(message)
        self.terminal_output.ensureCursorVisible()

class MainWindow(QMainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Robot Control Panel")

        screen_geometry = QApplication.primaryScreen().geometry()
        available_geometry = QApplication.primaryScreen().availableGeometry()
        screen_width = int(available_geometry.width() * 23 / 24)
        screen_height = int(available_geometry.height() * 23 / 24)
        self.setGeometry(0, 0, screen_width, screen_height)

        self.control_panel = ControlPanel(node, self)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.control_panel)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        self.control_panel.map_label.setFixedSize(int(screen_width * 2 / 3), int(screen_height * 2 / 3))
        self.control_panel.terminal_output.setFixedSize(int(screen_width * 2 / 3), int(screen_height * 1 / 3))
        self.control_panel.setFixedSize(screen_width, screen_height)

def main(args=None):
    rclpy.init(args=args)
    node = Node("robot_control_panel")
    app = QApplication(sys.argv)
    main_window = MainWindow(node)
    main_window.show()

    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
