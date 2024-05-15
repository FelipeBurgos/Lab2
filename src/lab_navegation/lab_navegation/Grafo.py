#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import os

def real_pose_info():
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        poses_path = os.path.join(script_dir, '..', 'files', 'real_pose.txt')
        real_pose_data = np.loadtxt(poses_path)
        return real_pose_data

    except FileNotFoundError:
        print(f"File not found: {poses_path}")

def odometry_info():
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        poses_path = os.path.join(script_dir, '..', 'files', 'odometry.txt')
        odometry_data = np.loadtxt(poses_path)
        return odometry_data

    except FileNotFoundError:
        print(f"File not found: {poses_path}")


# Arrays de Real Pose
x_real = real_pose_info()[:, 0]
y_real = real_pose_info()[:, 1]


# Arrays de Odometry
x_odom = odometry_info()[:, 0]
y_odom = odometry_info()[:, 1]

# Grafica

plt.figure(figsize=(8, 6))
plt.plot(x_real, y_real, 'b-', label='Real Pose')
plt.plot(x_odom, y_odom, 'r-', label='Odometry')
plt.xlabel('Eje X')
plt.ylabel('Eje Y')
plt.title('Comparación de Real Pose y Odometry sin factor de corrección')
plt.grid(True)
plt.legend()
plt.show()


# Valores experimentales
odom = odometry_info()
true_1 = np.concatenate((odom[:200, 1], odom[800:1000, 1], odom[1600:1800, 1])) # Error camino 1
true_2 = np.concatenate((odom[200:400, 0], odom[1000:1200, 0], odom[1800:2000, 0])) # Error camino 2
true_3 = np.concatenate((odom[400:600, 1], odom[1200:1400, 1], odom[2000:2200, 1])) # Error camino 3
true_4 = np.concatenate((odom[600:800, 0], odom[1400:1600, 0], odom[2200:2400, 0])) # Error camino 4

# Valores esperados
pred_0 = np.zeros((600,))
pred_1 = np.ones((600,))

def mse(true, predict):
    error_cuadratico = np.square(true - predict)
    mse = np.mean(error_cuadratico)
    return mse

mse_1 = mse(true_1, pred_0) # Verificar que Y esté en 0
mse_2 = mse(true_2, pred_1) # Verificar que X esté en 1
mse_3 = mse(true_3, pred_0) # Verificar que Y esté en 1
mse_4 = mse(true_4, pred_0) # Verificar que X esté en 0

mse_total = mse_1 + mse_2 + mse_3 + mse_4 # En distancia al cuadrado
print(mse_total)

