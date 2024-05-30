#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist
from transforms3d.euler import quat2euler as euler_from_quaternion
import os

class PosesReader( Node ):

  def __init__( self ):
    super().__init__( 'poses_reader' )
    # Se reinician los datos almacenados
    self.remove_information('odometry_KI')
    self.remove_information('act_linear_KI')
    self.remove_information('act_angular_KI')

    # Subscripcion a topicos
    self.odom = self.create_subscription(Odometry, '/odom', self.odometry_cb, 10)
    self.act_linear = self.create_subscription(Twist, '/cmd_vel_mux/input/navigation', self.act_linear_cb, 10)
    self.act_angular = self.create_subscription(Twist, '/cmd_vel_mux/input/navigation', self.act_angular_cb, 10)

  def act_linear_cb(self, speed:Twist):
    linear_array = []
    act_linear = speed.linear.x
    linear_array.append([act_linear])
    self.save_information(linear_array, 'act_linear_KI')
  
  def act_angular_cb(self, speed:Twist):
    angular_array = []
    act_angular = speed.angular.z
    angular_array.append([act_angular])
    self.save_information(angular_array, 'act_angular_KI')
    
  def odometry_cb( self, odom:Odometry):
    odometry_array = []
    # Se le debe agregar 1 porque el robot parte desde el (1,1) y odom no considera esto, piensa que parte de cero
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    yaw, _, __ = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z,
                                                odom.pose.pose.orientation.w ) )
    
    #self.get_logger().info('Odometry position ({:.2f}, {:.2f})'.format(x, y))
    # Actualización y escritura
    odometry_array.append([x, y, yaw])
    self.save_information(odometry_array, 'odometry_KI')


  def save_information(self, array:list, name:str):

    try:
      script_dir = os.path.dirname(os.path.realpath(__file__))
      poses_path = os.path.join(script_dir, '..', 'files', name +'.txt')
      with open(poses_path, 'a') as poses_file:
          for row in array:
            poses_file.write(' '.join(map(str, row)) + '\n')

    except FileNotFoundError:
      print(f"File not found: {poses_path}")
  

  def remove_information(self, name:str):
    try:
      script_dir = os.path.dirname(os.path.realpath(__file__))
      poses_path = os.path.join(script_dir, '..', 'files', name +'.txt')
      with open(poses_path, 'w') as poses_file:
          pass

    except FileNotFoundError:
      print(f"File not found: {poses_path}")


def main():
    rclpy.init()
    poses_reader = PosesReader()
    rclpy.spin(poses_reader)

if __name__ == '__main__':

  main()
