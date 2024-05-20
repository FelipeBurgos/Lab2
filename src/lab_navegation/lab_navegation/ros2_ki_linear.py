#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler as euler_from_quaternion
from time import time
from math import radians



class KI_Linear(Node):

  def __init__(self, kp, ki):
    super().__init__( 'ki_linear' )
    # Constantes de control
    self.kp = kp
    self.ki = ki

    # Atributo para activar o desactivar
    self.activate = False
    self.movement = 'x'

    # Posición de referencia
    self.setpoint = None
    self.state = None

    # Intervalo de tiempo
    self.initial_time = time()
    self.final_time = time()

    # Error acumulado
    self.acumulate_error = 0
    self.error_threshold_time = 0

    # Definición de tópicos
    self.publish_velocity = self.create_publisher(Twist, '/cmd_vel_mux/input/navigation', 10)
    self.dist_state = self.create_subscription(Odometry, '/odom', self.state_cb, 10)
    self.dist_set_point = self.create_subscription(Float64, 'linear_command', self.setpoint_cb, 10)
    self.pub_linear_done = self.create_publisher(Bool, 'linear_done', 10)


  def setpoint_cb(self, msg:Float64):
    self.reset()
    self.setpoint = msg.data
    self.get_logger().info('Setpoint received: %f meters' % self.setpoint)
    self.activate = True

  def state_cb(self, odom:Odometry):
    if self.setpoint == None or self.activate == False:
      #self.get_logger().info(f'activate: {self.activate}')
      return
    
    # se obtiene el desplazamiento lineal
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    d = x if self.movement == 'x' else y

    yaw, _, __ = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z,
                                                odom.pose.pose.orientation.w ) )
    
    # Lógica de control
    error = d - self.setpoint if -radians(180) <= yaw < 0  or radians(180) - 1e-3 <= yaw <= radians(180) + 1e-3 else self.setpoint - d
    self.acumulate_error += error
    dt = 0 if (self.final_time - self.initial_time) < 1e-6 else self.final_time - self.initial_time

    #self.get_logger().info( 'Current pose - lin: (%f, %f), error (%f)' % (x, y, error))

    # Proporcional
    p_actuation = error * self.kp

    # Integrativo
    i_actuation = dt * self.acumulate_error * self.ki

    # Derivativo
    #d_actuacion = self.kd * (error - self.acumulate_error)/dt

    # Resultado actuacion
    actuation = p_actuation + i_actuation
    
    # Definicion del mensaje de la velocidad angular
    speed = Twist()
    speed.linear.x = max(min(float(actuation), 0.2), -0.2) #Saturación angular
    #self.get_logger().info(f'activate: {speed.linear.x}')
      
    # Envio de la velocidad
    self.publish_velocity.publish(speed)

    # Actualizacion del intervalo de tiempo
    self.initial_time = self.final_time
    self.final_time = time()

    # Verificar que el error se mantiene en el tiempo
    
    if abs(error) < 1e-3:
      self.error_threshold_time += dt
  
      if self.error_threshold_time >= 3:
        self.get_logger().info('Desplazamiento lineal completado.')
        self.activate = False  # Desactivar para no seguir enviando comandos
        done_msg = Bool()
        done_msg.data = True
        self.pub_linear_done.publish(done_msg)
        self.movement = 'y' if self.movement == 'x' else 'x'
        return
      

  def reset( self ):
    self.setpoint = None
    self.state = None
    self.acumulate_error = 0.0
    self.error_threshold_time = 0.0

def main():
    rclpy.init()
    ki_linear = KI_Linear(kp= 8, ki= 0.05)
    rclpy.spin(ki_linear)

if __name__ == '__main__':
  main()