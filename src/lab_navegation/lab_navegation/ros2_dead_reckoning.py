#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64, Bool
from tf_transformations import euler_from_quaternion



class DeadReckoningNav(Node):

  def __init__(self):
    super().__init__('dead_reckoning_nav')
    self.subscription_goal = self.create_subscription(
        PoseArray,
        'goal_list',
        self.action_move_cb,
        10)
    
    # Publicadores de desplazamientos
    self.pub_linear = self.create_publisher(Float64, 'linear_command', 10)
    self.pub_angular = self.create_publisher(Float64, 'angular_command', 10)

    # Publicadores de semaforo
    self.sub_linear_done = self.create_subscription(Bool, 'linear_done', self.linear_done_cb, 10)
    self.sub_angular_done = self.create_subscription(Bool, 'angular_done', self.angular_done_cb, 10)

    # Atributos
    self.goal_queue = []
    self.displacement_list = []
    self.current_index = 0
    self.waiting_for_completion = False
    

  def send_next_command(self):
    if self.current_index < len(self.displacement_list):
      x, theta = self.displacement_list[self.current_index]
      msg = Float64()
      if theta == 0:
        msg.data = x
        self.get_logger().info(f'Sending linear command: {msg.data}')
        self.pub_linear.publish(msg)
      
      elif x == 0:
        msg.data = theta
        self.get_logger().info(f'Sending angular command: {msg.data}')
        self.pub_angular.publish(msg)
      
      self.waiting_for_completion = True
    
    else:
      self.get_logger().info('All displacements completed.')
      self.waiting_for_completion = False
      self.process_next_goal()
  
  def linear_done_cb(self, msg:Bool):
    if msg.data:
      self.get_logger().info('Linear movement completed.')
      self.current_index += 1
      self.waiting_for_completion = False
      self.send_next_command()
  
  def angular_done_cb(self, msg:Bool):
    if msg.data:
      self.get_logger().info('Angular movement completed.')
      self.waiting_for_completion = False
      self.current_index += 1
      self.send_next_command()


  def set_velocity(self, displacement_list:list):
    self.displacement_list = displacement_list
    self.current_index = 0
    self.send_next_command()
  

  def move_to_point(self, goal_pose:tuple):
    # Obtención de las posiciones
    x = goal_pose[0]
    y = goal_pose[1]
    theta = goal_pose[2]

    # Descomposición de desplazamientos
    displacement_list = [(x, 0.0), (0.0, theta), (y, 0.0)]
    self.set_velocity(displacement_list)
    self.get_logger().info(f'Displacement list: {displacement_list}')
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

      # Se añade el objetivo a la cola
      self.goal_queue.append(goal_pose)
    
    if not self.waiting_for_completion:
      # Creacion de trayectoria
      self.process_next_goal()
  
  def process_next_goal(self):
    if self.goal_queue:
      next_goal = self.goal_queue.pop(0)
      self.move_to_point(next_goal)



def main(args=None):
  rclpy.init(args=args)

  dead_reckoning_nav = DeadReckoningNav()
  
  rclpy.spin(dead_reckoning_nav)
  dead_reckoning_nav.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
