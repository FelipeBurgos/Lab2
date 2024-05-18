#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler as euler_from_quaternion
from time import time


class KI_Angular(Node):

  def __init__(self, kp, ki):
    super().__init__( 'ki_angular' )
    # Constantes de control
    self.kp = kp
    self.ki = ki

    # Atributo para activar o desactivar
    self.activate = False

    # Posición de referencia
    self.setpoint = None
    self.state = None

    # Intervalo de tiempo
    self.initial_time = time()
    self.final_time = time()

    # Error acumulado
    self.acumulate_error = 0

    # Definición de tópicos
    self.publish_velocity = self.create_publisher(Twist, '/cmd_vel_mux/input/navigation', 10)
    self.dist_state = self.create_subscription(Odometry, '/odom', self.state_cb, 10)
    self.dist_set_point = self.create_subscription(Float64MultiArray, 'setpoint', self.setpoint_cb, 10)

  def setpoint_cb(self, msg:Float64MultiArray):
    #self.get_logger().info('Setpoint received: %f radians' % msg.data)
    self.reset()
    self.setpoint = msg.data[1]

    self.activate = True if msg.data[0] == 0 else False


  def state_cb(self, odom):
    if self.setpoint == None or self.activate == False:
      return
    
    # se obtiene el angulo de yaw
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    z = odom.pose.pose.position.z
    roll, pitch, yaw = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z,
                                                odom.pose.pose.orientation.w ) )

    self.get_logger().info( 'Current pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) )

    # Lógica de control
    error = self.setpoint - roll

    # Proporcional
    p_actuation = error * self.kp

    # Integrativo
    dt = 0 if (self.final_time - self.initial_time) > 1e-6 else 0
    i_actuation = dt * error * self.ki
    self.acumulate_error += error

    # Resultado actuacion
    actuation = p_actuation + i_actuation
    
    # Definicion del mensaje de la velocidad angular
    speed = Twist()
    speed.angular.z = min(float(actuation), 0.1) #Saturación angular
    #print(speed.angular.z)
      
    # Envio de la velocidad
    self.publish_velocity.publish(speed)

    # Actualizacion del intervalo de tiempo
    self.initial_time = self.final_time
    self.final_time = time()

  def reset( self ):
    self.setpoint = None
    self.state = None

def main():
    rclpy.init()
    ki_angular = KI_Angular(kp= 1, ki= 0)
    rclpy.spin(ki_angular)

if __name__ == '__main__':
  main()