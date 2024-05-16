#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler as euler_from_quaternion


class KI_Angular(Node):

  def __init__(self):
    super().__init__( 'ki_angular' )
    self.activate = True
    self.P_ref = None

    self.publish_velocity = self.create_publisher(Twist, '/cmd_vel_mux/input/navigation', 10)
    self.dist_state = self.create_subscription(Odometry, '/odom', self.state_cb, 10)
    self.dist_set_point = self.create_subscription(Float64, 'setpoint', self.setpoint_cb, 10)


  def state_cb(self, odom):
    # se obtiene el angulo de yaw
    roll, pitch, yaw = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z,
                                                odom.pose.pose.orientation.w ) )


    # Aqui va la logica del control proporcional.


  def setpoint_cb(self, msg:Float64):
    self.P_ref = msg.data
    self.get_logger().info('Setpoint received: %f radians' % self.P_ref)


def main():
    rclpy.init()
    ki_angular = KI_Angular()
    rclpy.spin(ki_angular)

if __name__ == '__main__':
  main()