#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler as euler_from_quaternion


class OdometryReader( Node ):

  def __init__( self ):
    super().__init__( 'odom_reader_node' )
    self.activate = True

    self.publish_velocity = self.create_publisher(Twist, '/cmd_vel_mux/input/navigation', 10)
    self.dist_state = self.create_subscription(Odometry, '/odom', self.state_cb, 10)
    self.dist_set_point = self.create_subscription(Float64, 'setpoint', self.setpoint_cb, 10)


  def state_cb(self, odom):
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    z = odom.pose.pose.position.z
    roll, pitch, yaw = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z,
                                                odom.pose.pose.orientation.w ) )
    

    #self.get_logger().info( 'Current pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) )


  def setpoint_cb(self, P_ref):
    pass

def main():
    rclpy.init()
    odom_reader = OdometryReader()
    rclpy.spin(odom_reader)

if __name__ == '__main__':
  main()