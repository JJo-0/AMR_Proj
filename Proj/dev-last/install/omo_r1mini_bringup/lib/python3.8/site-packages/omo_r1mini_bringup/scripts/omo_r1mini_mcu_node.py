import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from time import sleep
import time
import copy
import math
import os
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32


class OdomPose(object):
    x = 0.0
    y = 0.0
    theta = 0.0
    timestamp = 0
    pre_timestamp = 0


class OdomVel(object):
    x = 0.0
    y = 0.0
    w = 0.0


class Joint(object):
    joint_name = ['wheel_left_joint', 'wheel_right_joint']
    joint_pos = [0.0, 0.0]
    joint_vel = [0.0, 0.0]


def quaternion_from_euler(roll, pitch, yaw):
    """
  Converts euler roll, pitch, yaw to quaternion (w in last place)
  quat = [x, y, z, w]
  Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
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


class OMOR1MiniNode(Node):
    def __init__(self):
        super().__init__('omo_r1mini_motor_setting')
        # Declare parameters from YAML
        self.declare_parameters(
            namespace='',
            parameters=[
                ('motor.gear_ratio', None),
                ('sensor.enc_pulse', None),
            ])
        # Get parameter values
        self.gear_ratio = 20.0
        self.wheel_separation = 0.76
        self.wheel_radius = 0.193
        self.enc_pulse = 16384.0

        print('GEAR RATIO:\t\t%s' % (self.gear_ratio))  # 1:20
        print('WHEEL SEPARATION:\t%s' % (self.wheel_separation))  # 0.76
        print('WHEEL RADIUS:\t\t%s' % (self.wheel_radius))  # 0.193
        print('ENC_PULSES:\t\t%s' % (self.enc_pulse))  # 240

        self.distance_per_pulse = 2 * math.pi * self.wheel_radius / (self.enc_pulse/self.gear_ratio)
        print('DISTANCE PER PULSE \t:%s' % (self.distance_per_pulse))

        self.odom_pose = OdomPose()
        self.odom_pose.timestamp = self.get_clock().now()
        self.odom_pose.pre_timestamp = self.get_clock().now()
        self.odom_vel = OdomVel()
        self.joint = Joint()

        self.l_enc = 0.0
        self.r_enc = 0.0
        self.linear_acceleration_x = 0.0
        self.linear_acceleration_y = 0.0
        self.linear_acceleration_z = 0.0

        # Set subscriber
        self.subEnc = self.create_subscription(Int32, 'enc', self.RsvEnc, 10)
        self.subIMU = self.create_subscription(Imu, 'imu/data', self.RsvImu, 10)
        self.pub_CmdVelMsg = self.create_publisher(Twist, 'diffbot_base_controller/cmd_vel_unstamped', 10)

        # Set publisher
        self.pub_JointStates = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_Odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_OdomTF = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(Pose, 'pose', 10)

        # Set timer proc
        self.timerProc = self.create_timer(0.01, self.update_robot)

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

        print("enc : ", msg.data)
        print("r_enc : ", self.r_enc)
        print("l_enc : ", self.l_enc)

    def RsvImu(self, msg):
        self.linear_acceleration_x = msg.linear_acceleration.x
        self.linear_acceleration_y = msg.linear_acceleration.y
        self.linear_acceleration_z = msg.linear_acceleration.z
        # print("orientation : ", msg.orientation.x)

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
        # Set odometry data
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

        # Set odomTF data
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = odom.header.frame_id
        odom_tf.child_frame_id = odom.child_frame_id
        odom_tf.header.stamp = timestamp_now

        odom_tf.transform.translation.x = odom.pose.pose.position.x
        odom_tf.transform.translation.y = odom.pose.pose.position.y
        odom_tf.transform.translation.z = odom.pose.pose.position.z
        odom_tf.transform.rotation = odom.pose.pose.orientation
        self.pub_OdomTF.sendTransform(odom_tf)

    def updatePoseStates(self, roll, pitch, yaw):
        # Added to publish pose orientation of IMU
        pose = Pose()
        pose.orientation.x = roll
        pose.orientation.y = pitch
        pose.orientation.z = yaw
        self.pub_pose.publish(pose)

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

    def update_robot(self):
        # 데이터 불러오기
        # with open('l_enc.pkl', 'rb') as file:
        #   loaded_data = pickle.load(file)
        odo_l = self.distance_per_pulse * self.l_enc  # 이런 구조 가능한지 모르겠다.....
        odo_r = self.distance_per_pulse * self.r_enc
        trans_vel = (odo_r + odo_l) / 2.0  # 현재 velocity를 받아야 함.
        orient_vel = (odo_r - odo_l) / self.wheel_radius
        roll_imu = self.linear_acceleration_x
        pitch_imu = self.linear_acceleration_y
        yaw_imu = self.linear_acceleration_z

        self.update_odometry(odo_l, odo_r, trans_vel, orient_vel)
        self.updateJointStates(odo_l, odo_r, trans_vel, orient_vel)
        self.updatePoseStates(roll_imu, pitch_imu, yaw_imu)


def main(args=None):
    rclpy.init(args=args)
    omoR1MiniNode = OMOR1MiniNode()
    rclpy.spin(omoR1MiniNode)

    omoR1MiniNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
