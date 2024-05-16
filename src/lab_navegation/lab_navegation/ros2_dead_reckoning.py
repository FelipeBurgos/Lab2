#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion
from math import radians



class DeadReckoningNav(Node):

  def __init__(self):
    super().__init__('dead_reckoning_nav')
    self.subscription_goal = self.create_subscription(
        PoseArray,
        'goal_list',
        self.action_move_cb,
        10)
    self.dist_set_point = self.create_publisher(Float64, 'P_ref', 10)
  
    

  def set_velocity(self, displacement_list:list):
    for displacement in displacement_list:
      msg = Float64()

      if displacement_list.index(displacement) % 2 == 0:
        msg.data = displacement[0]
        self.dist_set_point.publish(msg)
      
      else:
        msg.data = radians(displacement[1])
        self.dist_set_point.publish(msg)
  

  def move_to_point(self, goal_pose:tuple):
    # Obtención de las posiciones
    x = goal_pose[0]
    y = goal_pose[1]
    theta = goal_pose[2]

    # Descomposición de desplazamientos
    displacement_list = [(x, 0), (0, theta), (y, 0)]
    self.set_velocity(displacement_list)

    self.get_logger().info(f'Displacement list, {str(x)}, {str(y)}, {str(theta)})')
    self.get_logger().info('--------------------')


  def action_move_cb(self, msg:PoseArray):
    
    self.get_logger().info('Recibido')
    # Iteracion para traducir Quaternion a Euler
    
    for pose in msg.poses:
      quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
      )
      roll, pitch, yaw = euler_from_quaternion(quaternion) # En radianes

      # Obtencion de la pose en formato (x, y, theta)
      goal_pose = (pose.position.x, pose.position.y, yaw)
      
      # Creacion de trayectoria
      self.move_to_point(goal_pose)



def main(args=None):
  rclpy.init(args=args)

  dead_reckoning_nav = DeadReckoningNav()
  
  rclpy.spin(dead_reckoning_nav)
  dead_reckoning_nav.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
