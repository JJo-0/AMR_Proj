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
from std_msgs.msg import Int32, Float64, String


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




class ComplementaryFilter():
  def __init__(self):
    self.theta = 0.
    self.pre_theta = 0.
    self.wheel_ang = 0.
    self.filter_coef = 2.5
    self.gyro_bias = 0.
    self.count_for_gyro_bias = 110

  def gyro_calibration(self, gyro):
    self.count_for_gyro_bias -= 1

    self.gyro_bias += gyro
    if self.count_for_gyro_bias == 1:
      self.gyro_bias /= 100
      print('Complete : Gyro calibration')


#   def calc_filter(self, gyro, d_time):

#     if self.count_for_gyro_bias != 1:
#       self.gyro_calibration(gyro)
#       return 0

#     gyro -= self.gyro_bias
#     self.pre_theta = self.theta
#     temp = -1/self.filter_coef * (-self.wheel_ang + self.pre_theta) + gyro
#     print(gyro)
#     self.theta = self.pre_theta + temp*d_time
#     #print self.theta*180/3.141, self.wheel_ang*180/3.141, gyro, d_time
#     return self.theta

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
        self.wheel_radius = 0.193 / 2
        self.enc_pulse = 1480.0

        print('GEAR RATIO:\t\t%s' % (self.gear_ratio))  # 1:20
        print('WHEEL SEPARATION:\t%s' % (self.wheel_separation))  # 0.76
        print('WHEEL RADIUS:\t\t%s' % (self.wheel_radius))  # 0.0965
        print('ENC_PULSES:\t\t%s' % (self.enc_pulse))  # 240

        # self.distance_per_pulse = 2 * math.pi * self.wheel_radius / self.enc_pulse / self.gear_ratio
        # self.distance_per_pulse = 0.0009 * 13 / 14
        self.distance_per_pulse = 0.193 * math.pi / 480.0
 
        self.calc_yaw = ComplementaryFilter()
        self.use_gyro = False
        self.odom_pose = OdomPose()
        self.odom_pose.timestamp = self.get_clock().now()
        self.odom_pose.pre_timestamp = self.get_clock().now()
        self.odom_vel = OdomVel()
        self.joint = Joint()

        self.l_enc = 0.0
        self.r_enc = 0.0
        
        self.odo_l = 0.0
        self.odo_r = 0.0
        
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.check = 0.0
        # self.odom_sum = 0.0
        
        self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w = 0.0, 0.0, 0.0, 0.0
        # self.encoder_setting = False
        # self.past_dist = [0, 0]
        # self.curr_dist = [0, 0]
        #self.count = 0
        # Set subscriber
        # self.subEncS = self.create_subscription(String, 'enc_str', self.RsvEncStr, 10)
        self.subEnc = self.create_subscription(Int32, 'enc', self.RsvEnc, 10)
        self.subIMU = self.create_subscription(Imu, 'imu/data', self.RsvImu, 10)
        self.past_yaw = 0.0
        self.trans_dist = 0.0
        self.orient_dist = 0.0
        self.calibration =  (math.pi+0.05) / math.pi
        self.d_theta = 0.0
        self.q = [0] * 4
        self.check = 0.0
        # self.subYaw = self.create_subscription(Float64, 'imu/yaw', self.RsvYaw, 10)

        # Set publisher

        self.pub_CmdVelMsg = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_JointStates = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_Odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_OdomTF = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(Pose, 'pose', 10)
        # Set timer proc
        

    def quaternion_from_euler(self, roll, pitch, yaw):
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

        self.q[0] = cy * cp * sr - sy * sp * cr
        self.q[1] = sy * cp * sr + cy * sp * cr
        self.q[2] = sy * cp * cr - cy * sp * sr
        self.q[3] = cy * cp * cr + sy * sp * sr

    def RsvEnc(self, msg):
        if msg.data % 100 == 0:
            self.r_enc = (msg.data % 1000000)/100
            self.l_enc = ((msg.data - self.r_enc * 100) / 1000000)
        elif msg.data % 100 == 1:
            self.r_enc = ((msg.data-1) % 1000000)/100
            self.l_enc = ((msg.data - self.r_enc * 100 -1) / 1000000)
            self.r_enc = -1 * self.r_enc
        elif msg.data % 100 == 10:
            self.r_enc = ((msg.data-10) % 1000000-10)/100
            self.l_enc = ((msg.data - self.r_enc * 100-10) / 1000000)
            self.l_enc = -1 * self.l_enc
        elif msg.data % 100 == 11:
            self.r_enc = ((msg.data-11) % 1000000)/100
            self.l_enc = ((msg.data - self.r_enc * 100 -11) / 1000000)
            self.r_enc = -1 * self.r_enc
            self.l_enc = -1 * self.l_enc

        self.odo_l = self.distance_per_pulse * self.l_enc  
        self.odo_r = self.distance_per_pulse * self.r_enc

        # self.odom_sum += self.odo_l
        # print(self.odom_sum)
        # if ((self.odo_r - self.odo_l) / self.wheel_separation) == 0 or (self.yaw - self.past_yaw) == 0:
        #     self.calibration = 0.0
        # else:
        #     self.calibration = (self.yaw - self.past_yaw) / ((self.odo_r - self.odo_l) / self.wheel_separation)
        
        self.trans_dist = ((self.odo_r + self.odo_l) / 2.0)  # 현재 velocity를 받아야 함.
        self.orient_dist = ((self.odo_r - self.odo_l) / self.wheel_separation)

        self.querternion_to_euler(self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w)
        
        if abs(self.yaw - self.past_yaw) > 5.5:
            self.d_theta = math.pi * 2 - abs(self.yaw) - abs(self.past_yaw) 
        else:
            self.d_theta = self.yaw - self.past_yaw

        self.d_theta = self.d_theta * self.calibration
        self.past_yaw = self.yaw
        self.update_robot()

    def split_int(self, str):
        parts = str.split('_')

        int1 = int(parts[0])
        int2 = int(parts[1])
        return [int1, int2]


    def RsvImu(self, msg):
        self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w

        # print("orientation : ", msg.orientation.x)

    def update_odometry(self):

        self.odom_pose.timestamp = self.get_clock().now()
        dt = (self.odom_pose.timestamp - self.odom_pose.pre_timestamp).nanoseconds * 1e-9
        self.odom_pose.pre_timestamp = self.odom_pose.timestamp
        trans_vel = self.trans_dist / dt 
        orient_vel = self.orient_dist / dt 

        # self.odom_pose.theta += self.d_theta
        self.odom_pose.theta += self.d_theta
        #self.check += self.orient_dist
        # print(self.odom_pose.theta - self.check)
        d_x = trans_vel * math.cos(self.odom_pose.theta)
        d_y = trans_vel * math.sin(self.odom_pose.theta)
        self.odom_pose.x += d_x * dt
        self.odom_pose.y += d_y * dt
        # if self.check > math.pi * 2:
         #   self.odom_pose.theta -= 2 * math.pi
        #    self.check -= 2 * math.pi
        #    print(self.check - self.odom_pose.theta)
        # 2print(self.odom_pose.theta - self.check)
        # print('ODO L:%.2f, R:%.2f, V:%.2f, W=%.2f --> X:%.2f, Y:%.2f, Theta:%.2f'
        #  %(odo_l, odo_r, trans_vel, orient_vel,
        #  self.odom_pose.x,self.odom_pose.y,self.odom_pose.theta))
        self.quaternion_from_euler(0, 0, self.odom_pose.theta)

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
        odom.pose.pose.orientation.x = self.q[0]
        odom.pose.pose.orientation.y = self.q[1]
        odom.pose.pose.orientation.z = self.q[2]
        odom.pose.pose.orientation.w = self.q[3]

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

        return trans_vel, orient_vel

    def updatePoseStates(self):
        # Added to publish pose orientation of IMU
        pose = Pose()
        pose.orientation.x = self.q[0]
        pose.orientation.y = self.q[1]
        pose.orientation.z = self.q[2]
        self.pub_pose.publish(pose)

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

    def updateJointStates(self, trans_vel, orient_vel):

        wheel_ang_left = self.odo_l / self.wheel_radius  #
        wheel_ang_right = self.odo_r / self.wheel_radius  #

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

        trans_vel, orient_vel = self.update_odometry()
        self.updateJointStates(trans_vel, orient_vel)
        self.updatePoseStates()



def main(args=None):
    rclpy.init(args=args)
    omoR1MiniNode = OMOR1MiniNode()
    rclpy.spin(omoR1MiniNode)

    omoR1MiniNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()