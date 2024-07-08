import rclpy  # ROS 2 Python 클라이언트를 불러옴
from rclpy.node import Node  # 노드 생성 및 관리
from rclpy.logging import get_logger  # 로그 기능 사용
from rclpy.parameter import Parameter  # 파라미터 관리
from time import sleep  # 일시 정지 기능 사용
import time  # 시간 관련 기능 사용
import copy  # 객체 복사 기능 사용
import math  # 수학 함수 사용
import os  # OS 관련 기능 사용
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, TransformStamped  # 메시지 타입 불러옴
from nav_msgs.msg import Odometry  # 메시지 타입 불러옴
from sensor_msgs.msg import Imu, JointState  # 메시지 타입 불러옴
from tf2_ros import TransformBroadcaster  # 좌표 변환 브로드캐스터
from std_msgs.msg import Int32, Float64  # 메시지 타입 불러옴

# 로봇의 위치 정보를 저장하는 클래스
class OdomPose(object):
    x = 0.0  # x 좌표
    y = 0.0  # y 좌표
    theta = 0.0  # 회전 각도
    timestamp = 0  # 현재 시간
    pre_timestamp = 0  # 이전 시간

# 로봇의 속도 정보를 저장하는 클래스
class OdomVel(object):
    x = 0.0  # x 방향 속도
    y = 0.0  # y 방향 속도
    w = 0.0  # 각속도

# 조인트 상태를 저장하는 클래스
class Joint(object):
    joint_name = ['wheel_left_joint', 'wheel_right_joint']  # 조인트 이름
    joint_pos = [0.0, 0.0]  # 조인트 위치
    joint_vel = [0.0, 0.0]  # 조인트 속도

# 오일러 각을 쿼터니언으로 변환하는 함수
def quaternion_from_euler(roll, pitch, yaw):
    """
  Converts euler roll, pitch, yaw to quaternion (w in last place)
  quat = [x, y, z, w]
  """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr

    return q

# 로봇의 동작을 제어하는 클래스
class OMOR1MiniNode(Node):
    def __init__(self):
        super().__init__('omo_r1mini_motor_setting')
        # YAML 파일에서 파라미터 선언
        self.declare_parameters(
            namespace='',
            parameters=[
                ('motor.gear_ratio', None),
                ('sensor.enc_pulse', None),
            ])
        # 파라미터 값 불러오기
        self.gear_ratio = 20.0
        self.wheel_separation = 0.76
        self.wheel_radius = 0.193
        self.enc_pulse = 16384.0

        print('GEAR RATIO:\t\t%s' % (self.gear_ratio))  # 기어비 출력
        print('WHEEL SEPARATION:\t%s' % (self.wheel_separation))  # 바퀴 간 거리 출력
        print('WHEEL RADIUS:\t\t%s' % (self.wheel_radius))  # 바퀴 반지름 출력
        print('ENC_PULSES:\t\t%s' % (self.enc_pulse))  # 엔코더 펄스 출력

        self.distance_per_pulse = 2 * math.pi * self.wheel_radius / (self.enc_pulse/self.gear_ratio)
        print('DISTANCE PER PULSE \t:%s' % (self.distance_per_pulse))  # 펄스 당 거리 출력

        self.odom_pose = OdomPose()
        self.odom_pose.timestamp = self.get_clock().now()  # 현재 시간 저장
        self.odom_pose.pre_timestamp = self.get_clock().now()  # 이전 시간 저장
        self.odom_vel = OdomVel()
        self.joint = Joint()

        self.l_enc = 0.0
        self.r_enc = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        # 구독자 설정
        self.subEnc = self.create_subscription(Int32, 'enc', self.RsvEnc, 10)
        self.subIMU = self.create_subscription(Imu, 'imu/data', self.RsvImu, 10)
        self.subYaw = self.create_subscription(Float64, 'imu/yaw', self.RsvYaw, 10)
        self.pub_CmdVelMsg = self.create_publisher(Twist, 'diffbot_base_controller/cmd_vel_unstamped', 10)

        # 퍼블리셔 설정
        self.pub_JointStates = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_Odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_OdomTF = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(Pose, 'pose', 10)

        # 타이머 프로세스 설정
        self.timerProc = self.create_timer(0.01, self.update_robot)

    # 엔코더 값을 수신하는 콜백 함수
    def RsvEnc(self, msg):
        if msg.data % 100 == 0:
            self.r_enc = (msg.data % 1000000)/100
            self.l_enc = ((msg.data - self.r_enc * 100) / 1000000)
        elif msg.data % 100 == 1:
            self.r_enc = (msg.data % 1000000-1)/100
            self.l_enc = ((msg.data - self.r_enc * 100 -1) / 1000000)
            self.r_enc = -1 * self.r_enc
        elif msg.data % 100 == 10:
            self.r_enc = (msg.data % 1000000-10)/100
            self.l_enc = ((msg.data - self.r_enc * 100-10) / 1000000)
            self.l_enc = -1 * self.l_enc
        elif msg.data % 100 == 11:
            self.r_enc = (msg.data % 1000000-11)/100
            self.l_enc = ((msg.data - self.r_enc * 100 -11) / 1000000)
            self.r_enc = -1 * self.r_enc
            self.l_enc = -1 * self.l_enc

    # IMU 데이터를 수신하는 콜백 함수
    def RsvImu(self, msg):
        self.querternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # print("orientation : ", msg.orientation.x)

    # Yaw 값을 수신하는 콜백 함수
    def RsvYaw(self, msg):
        print(msg.data /180.0 * math.pi)

    # 오도메트리 업데이트 함수
    def update_odometry(self, odo_l, odo_r, trans_vel, orient_vel):
        self.odom_pose.timestamp = self.get_clock().now()
        dt = (self.odom_pose.timestamp - self.odom_pose.pre_timestamp).nanoseconds * 1e-9
        self.odom_pose.pre_timestamp = self.odom_pose.timestamp
        self.odom_pose.theta += orient_vel * dt

        d_x = trans_vel * math.cos(self.odom_pose.theta)
        d_y = trans_vel * math.sin(self.odom_pose.theta)
        self.odom_pose.x += d_x * dt
        self.odom_pose.y += d_y * dt
        #  %(odo_l, odo_r, trans_vel, orient_vel,
        #  self.odom_pose.x,self.odom_pose.y,self.odom_pose.theta))
        q = quaternion_from_euler(0, 0, self.odom_pose.theta)

        self.odom_vel.x = trans_vel
        self.odom_vel.y = 0.
        self.odom_vel.w = orient_vel

        timestamp_now = self.get_clock().now().to_msg()
        # 오도메트리 데이터 설정
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.header.stamp = timestamp_now
        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = trans_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = orient_vel

        self.pub_Odom.publish(odom)

        # odomTF 데이터 설정
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = odom.header.frame_id
        odom_tf.child_frame_id = odom.child_frame_id
        odom_tf.header.stamp = timestamp_now

        odom_tf.transform.translation.x = odom.pose.pose.position.x
        odom_tf.transform.translation.y = odom.pose.pose.position.y
        odom_tf.transform.translation.z = odom.pose.pose.position.z
        odom_tf.transform.rotation = odom.pose.pose.orientation
        self.pub_OdomTF.sendTransform(odom_tf)

    # 자세 상태 업데이트 함수
    def updatePoseStates(self):
        pose = Pose()
        pose.orientation.x = self.roll
        pose.orientation.y = self.pitch
        pose.orientation.z = self.yaw
        self.pub_pose.publish(pose)

    # 쿼터니언을 오일러 각으로 변환하는 함수
    def querternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2* (w* x + y * z)
        cosr_cosp = 1 - 2* (x * x + y * y)
        self.roll = math.atan2(sinr_cosp,cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.pitch = math.copysign(math.pi / 2, sinp)
        else:
            self.pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        print(self.yaw)

    # 조인트 상태 업데이트 함수
    def updateJointStates(self, odo_l, odo_r, trans_vel, orient_vel):
        odo_l /= 1000.
        odo_r /= 1000.

        wheel_ang_left = odo_l / self.wheel_radius  #
        wheel_ang_right = odo_r / self.wheel_radius  #

        wheel_ang_vel_left = (trans_vel - (self.wheel_separation / 2.0) * orient_vel) / self.wheel_radius
        wheel_ang_vel_right = (trans_vel + (self.wheel_separation / 2.0) * orient_vel) / self.wheel_radius

        self.joint.joint_pos = [wheel_ang_left, wheel_ang_right]
        self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]

        timestamp_now = self.get_clock().now().to_msg()

        joint_states = JointState()
        joint_states.header.frame_id = "base_link"
        joint_states.header.stamp = timestamp_now
        joint_states.name = self.joint.joint_name
        joint_states.position = self.joint.joint_pos
        joint_states.velocity = self.joint.joint_vel
        joint_states.effort = []
        self.pub_JointStates.publish(joint_states)

    # 로봇 업데이트 함수
    def update_robot(self):
        odo_l = self.distance_per_pulse * self.l_enc  # 좌측 엔코더 값으로 이동 거리 계산
        odo_r = self.distance_per_pulse * self.r_enc  # 우측 엔코더 값으로 이동 거리 계산
        trans_vel = (odo_r + odo_l) / 2.0  # 직선 속도 계산
        orient_vel = (odo_r - odo_l) / self.wheel_separation  # 회전 속도 계산

        self.update_odometry(odo_l, odo_r, trans_vel, orient_vel)  # 오도메트리 업데이트
        self.updateJointStates(odo_l, odo_r, trans_vel, orient_vel)  # 조인트 상태 업데이트
        self.updatePoseStates()  # 자세 상태 업데이트

# 메인 함수
def main(args=None):
    rclpy.init(args=args)
    omoR1MiniNode = OMOR1MiniNode()
    rclpy.spin(omoR1MiniNode)

    omoR1MiniNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
