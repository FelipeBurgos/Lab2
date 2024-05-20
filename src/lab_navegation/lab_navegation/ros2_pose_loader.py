#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import quaternion_from_euler
from math import radians
import os
import threading

def find_poses_file():
    # Obtener el directorio $HOME del usuario actual
    home_dir = os.path.expanduser('~')
    
    # Construir la ruta absoluta del archivo poses.txt
    poses_path = os.path.join(home_dir, 'Lab2', 'src', 'lab_navegation', 'files', 'poses.txt')
    
    # Comprobar si el archivo existe en la ruta especificada
    if os.path.exists(poses_path):
        return poses_path
    else:
        raise FileNotFoundError(f"File not found: {poses_path}")

class PoseLoader(Node):

    def __init__(self):
        super().__init__('Pose_loader')
        self.publisher_ = self.create_publisher(PoseArray, 'goal_list', 10)

        self.msg = self.pose_msg(self.file_pose_procesing())

        input_send = threading.Thread(target=self.input_send)
        input_send.daemon = True
        input_send.start()

    def input_send(self):
        while True:
            send = str(input())
            if send == '':
                self.send_msg(self.msg)

    def file_pose_procesing(self):
        raw_poses_list = []
        try:
            # Utilizar la función para encontrar el archivo poses.txt
            poses_path = find_poses_file()
            with open(poses_path, 'r') as poses_file:
                raw_poses_list = poses_file.readlines()
            self.get_logger().info(f"Reading file: {poses_path}")
        except FileNotFoundError as e:
            self.get_logger().error(e)
            rclpy.shutdown()
            return []

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
            if 0 <= yaw_degree <= 180:
             yaw_radian = radians(yaw_degree)

            elif yaw_degree == 360:
             yaw_radian = radians(yaw_degree - 180)
            
            else:
             yaw_radian = -radians(yaw_degree - 180)

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
