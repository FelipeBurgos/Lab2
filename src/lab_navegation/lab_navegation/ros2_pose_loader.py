#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import quaternion_from_euler
from math import radians
import os
import threading

global raw_poses_list

class PoseLoader(Node):

  def __init__(self):
    super().__init__('Pose_loader')
    self.publisher_ = self.create_publisher(PoseArray, 'goal_list', 10)

    self.msg = self.pose_msg(self.file_pose_procesing())

    input_send = threading.Thread(target= self.input_send)
    input_send.daemon = True
    input_send.start()


  def input_send(self):
    while True:
      send = str(input())
      if send =='':
        self.send_msg(self.msg)
    
  def file_pose_procesing(self):
    # Lectura del archivo
    try:
      script_dir = os.path.dirname(os.path.realpath(__file__))
      poses_path = os.path.join(script_dir, 'poses.txt')
      with open(poses_path, 'r') as poses_file:
        raw_poses_list = poses_file.readlines()
      print(f"Reading file: {poses_path}")

    except FileNotFoundError:
      print(f"File not found: {poses_path}")

    # Se convierte texto en lista de tuplas con posiciones [..., (x, y, theta), ...]
    poses = [tuple(map(float, raw_pose.strip('()\n').split(','))) for raw_pose in raw_poses_list]
    
    return poses


  def pose_msg(self, positions):
    # Declaracion del mensaje PoseArray()
    pose_array_msg = PoseArray()
    
    # Lectura de las posiciones
    for x, y, yaw_degree in positions:
      pose = Pose()
      
      # Declaración del eje X e Y
      pose.position.x = x
      pose.position.y = y

      # Transformación del angulo euler en quaternion
      yaw_radian = radians(yaw_degree)
      quaternion = quaternion_from_euler(0, 0, yaw_radian)

      # Definicion de orientación en cada eje
      pose.orientation.x = quaternion[0]
      pose.orientation.y = quaternion[1]
      pose.orientation.z = quaternion[2]
      pose.orientation.w = quaternion[3]

      # Se añade lo obtenido a PoseArray()
      pose_array_msg.poses.append(pose)

    return pose_array_msg
      

  def send_msg(self, msg):
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg)




def main(args=None):
  rclpy.init(args=args)

  pose_loader = PoseLoader()

  rclpy.spin(pose_loader)
  pose_loader.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
