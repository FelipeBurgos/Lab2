#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import threading
import time


class DeadReckoningNav(Node):

  def __init__(self):
    super().__init__('dead_reckoning_nav')
    self.subscription_goal = self.create_subscription(
        PoseArray,
        'goal_list',
        self.action_move_cb,
        10)
    
    self.x = 0
    self.y = 0
    self.theta = 0

    self.linear_v = 0.2 # m/s
    self.angular_v = 1.0 # rad/s
    
    self.publish_velocity = self.create_publisher(Twist, '/cmd_vel_mux/input/navigation', 1)
    

  def set_velocity(self, speed_command_list):
    
    # Iteramos sobre la lista de triplets
    for speed_command in speed_command_list:
      
      # Definimos el tiempo inicial de la tupla y creamos un tiempo actual de iteración
      time_initial = time.time()
      time_end = 0

      # Definimos el tiempo calculado en move_to_point para la tupla
      timer_period = float(speed_command[2])

      # Sentencio un while para enviar mensajes de forma iterativa hasta el tiempo solicitado
      while time_end - time_initial <= timer_period: 
        
        # Defino el mensaje y agrego las velocidades
        speed = Twist()
        speed.linear.x = float(speed_command[0])
        speed.angular.z = float(speed_command[1])
      
        # Publico en el tópico
        self.publish_velocity.publish(speed)
      
        
        # Seteo el periodo de cada mensaje, en este caso 10 Hz
        time.sleep(0.1)

        # Actualizo el tiempo actual de iteración
        time_end = time.time()


  def calculate_time(self, diferential, v):
    
    # Calcula el tiempo segun el diferencial que se ingrese y su velocidad
    time = abs(diferential)/v

    return time
  

  def move_to_point(self, goal_pose:tuple):
    # Diferenciales x e y
    dx = goal_pose[0] - self.x
    dy = goal_pose[1] - self.y
    
    # Distancia a recorrer
    distance = math.sqrt(dx**2 + dy**2)

    self.get_logger().info(f'Inicio, {str(self.x)}, {str(self.y)}, {str(self.theta)})')
    self.get_logger().info(f'Final {str(goal_pose)})')

    # Angulo de alineamiento al punto
    if math.atan2(dy, dx) < math.pi:
      point_alignment_angle = -self.theta + math.atan2(dy, dx)
    if dx < 0:
      point_alignment_angle = -self.theta + (math.atan2(dy, dx) - math.pi/2)
    if dy < 0:
      point_alignment_angle = -self.theta + (math.atan2(dy, dx) + math.pi)

    # Depreciar valores muy pequeños
    if point_alignment_angle < 1e-6:
      point_alignment_angle = 0

    self.get_logger().info(f'angulo de alineamiento: {str(point_alignment_angle)}')

    # Angulo de direccion final
    direction_angle = goal_pose[2] - point_alignment_angle
    
    self.get_logger().info(f'direccion direccion: {str(direction_angle)}')

    speed_command_list = []

    # Generacion de triplets
    factor = 1 + 0.08387853103
    # + 0.08387853103 (factor de correccion calculado)
    if point_alignment_angle != 0:
      speed_command_list.append((0, math.copysign(self.angular_v, point_alignment_angle), self.calculate_time(point_alignment_angle, self.angular_v)))
    if distance != 0:
      speed_command_list.append((self.linear_v, 0, self.calculate_time(distance, self.linear_v)))
    if direction_angle != 0:
      speed_command_list.append((0, math.copysign(self.angular_v, direction_angle), factor * self.calculate_time(direction_angle, self.angular_v)))

    # Envio de la lista de triplets al metodo que envia mensajes
    self.set_velocity(speed_command_list)
    self.get_logger().info('Sending: ' + str(speed_command_list))

        # Actualizacion de pose en marco inercial
    self.x = goal_pose[0]
    self.y = goal_pose[1]
    self.theta = goal_pose[2] if dy >=0 else 0 # Utilizamos radianes negativos

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
