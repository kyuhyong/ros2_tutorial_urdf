from math import sin, cos, atan, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class OdomPose(object):
   x = 0.0
   y = 0.0
   theta = 0.0

class StatePublisher(Node):

  def __init__(self):
    rclpy.init()
    super().__init__('state_publisher')

    qos_profile = QoSProfile(depth=10)
    self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
    self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
    self.nodeName = self.get_name()
    self.get_logger().info("{0} started".format(self.nodeName))

    degree = pi / 180.0
    loop_hz = 30.0
    loop_rate = self.create_rate(loop_hz)

    # robot params
    self.w_separation = 0.17
    self.w_radius = 0.035
    self.dt = 1.0/loop_hz

    # robot state
    self.odom_pose = OdomPose()
    self.v_lin_x = 0.015   # m/s
    self.v_rot_z = 0.1 * pi/180  # deg/s to rad/s

    wl = 0.
    wr = 0.
    caster_angle = 0.
    caster_w = 0.

    #tilt = 0.
    #tinc = degree
    #swivel = 0.
    #angle = 0.
    #height = 0.
    #hinc = 0.005

    # message declarations
    odom_trans = TransformStamped()
    odom_trans.header.frame_id = 'odom'
    odom_trans.child_frame_id = 'base_footprint'
    joint_state = JointState()

    try:
      while rclpy.ok():
        rclpy.spin_once(self)

        # update joint_state
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        # Joint names
        joint_state.name = ['wheel_left_joint', 'wheel_right_joint', 'caster_mount_joint', 'caster_rotate_joint']
        joint_state.position = [wl, wr, caster_angle, caster_w]

        # update transform
        # (moving with preset v, w command)
        vl, vr = self.diffV_from_vw(self.v_lin_x, self.v_rot_z)
        self.updatePose_from_diffV(vl, vr)
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.translation.x = self.odom_pose.x
        odom_trans.transform.translation.y = self.odom_pose.y
        odom_trans.transform.rotation.z = self.odom_pose.theta
        odom_trans.transform.rotation = \
        euler_to_quaternion(0, 0, self.odom_pose.theta) # roll,pitch,yaw

        wl+= self.wheelAngle_from_V(vl)
        wr+= self.wheelAngle_from_V(vr)
        # send the joint state and transform
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)

        # This will adjust as needed per iteration
        loop_rate.sleep()

    except KeyboardInterrupt:
      pass

  def diffV_from_vw(self, v, w):
    vl = (v - (self.w_separation/2.0) * w)/self.w_radius
    vr = (v + (self.w_separation/2.0) * w)/self.w_radius
    return vl, vr
  
  def updatePose_from_diffV(self, vl, vr):
    #w = (vr - vl)/self.w_separation
    #R = (vr + vl)/(vr - vl) * self.w_separation /2.0

    d_s = (vl + vr)*self.dt /2.0
    d_l = vl * self.dt
    d_r = vr * self.dt

    R = (d_r + d_l) * self.w_separation / 2.0
    d_theta = (d_r - d_l) / self.w_separation
    
    self.odom_pose.theta += d_theta
    self.odom_pose.x += cos(self.odom_pose.theta) * R
    self.odom_pose.y += sin(self.odom_pose.theta) * R

  def wheelAngle_from_V(self, v):
    d = v*self.dt
    return atan(d/self.w_radius)
    


def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)





def main():
  node = StatePublisher()

if __name__ == '__main__':
  main()