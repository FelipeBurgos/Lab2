#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import os
from math import radians

def odometry_lab1_info():
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        poses_path = os.path.join(script_dir, '..', 'files', 'odometry_lab1.txt')
        odometry_data = np.loadtxt(poses_path)
        return odometry_data

    except FileNotFoundError:
        print(f"File not found: {poses_path}")

def act_linear_info():
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        poses_path = os.path.join(script_dir, '..', 'files', 'act_linear_KI.txt')
        odometry_data = np.loadtxt(poses_path)
        return odometry_data

    except FileNotFoundError:
        print(f"File not found: {poses_path}")

def act_angular_info():
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        poses_path = os.path.join(script_dir, '..', 'files', 'act_angular_KI.txt')
        odometry_data = np.loadtxt(poses_path)
        return odometry_data

    except FileNotFoundError:
        print(f"File not found: {poses_path}")

def odometry_info():
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        poses_path = os.path.join(script_dir, '..', 'files', 'odometry_KI.txt')
        odometry_data = np.loadtxt(poses_path)
        return odometry_data

    except FileNotFoundError:
        print(f"File not found: {poses_path}")

def generar_ref_l(N, M, L, vueltas):
    # Generar los arrays para cada número y repetirlos según las repeticiones
    array_0 = np.zeros(N)
    array_1 = np.ones(M)
    array_2 = np.full(L, 2)
    
    # Concatenar los arrays en el orden adecuado
    resultado = np.concatenate((array_1, array_2, array_1, array_0))
    
    # Repetir el resultado según las repeticiones especificadas
    resultado_repetido = np.tile(resultado, vueltas)
    
    return resultado_repetido

def generar_ref_a(N, M, L, vueltas):
    # Generar los arrays para cada número y repetirlos según las repeticiones
    array_0 = np.zeros(N)
    array_1 = np.full(M, radians(90))
    array_2 = np.full(L, radians(180))
    
    # Concatenar los arrays en el orden adecuado
    resultado = np.concatenate((array_1, array_2, array_1*-1, array_0))
    
    # Repetir el resultado según las repeticiones especificadas
    resultado_repetido = np.tile(resultado, vueltas)
    
    return resultado_repetido

# Arrays de Odometry lab1
x_odom_lab1 = odometry_lab1_info()[:, 0]
y_odom_lab1 = odometry_lab1_info()[:, 1]

# Arrays de actuacion
linear = np.concatenate((np.zeros(25), act_linear_info()[:]))
angular = np.concatenate((np.zeros(25), act_angular_info()[:]))

# Arrays de Odometry actual
x_odom = odometry_info()[:, 0]
y_odom = odometry_info()[:, 1]
yaw_odom = odometry_info()[:, 2]

dist = (x_odom*x_odom)**0.5 + (y_odom*y_odom)**0.5


# Tiempo de ejecucion
t_odom = list(range(x_odom.shape[0]))
t_angular = list(range(angular.shape[0]))
t_linear = list(range(linear.shape[0]))

# Posiciones de referencia
inicio = np.zeros(750) #lineal P:50, angular P:320, lineal PI:40
#p_ref =  np.concatenate((inicio, generar_ref_l(N=1370, M=1440, L=1364, vueltas=3))) #P:503
#t_ref = np.arange(len(p_ref))
p_ref = np.concatenate((inicio, generar_ref_a(N=1370, M=1440, L=1364, vueltas=3)))
t_ref = np.arange(len(p_ref))


# Grafico
plt.figure(figsize=(8, 6))

# Para Ref, act y Real
plt.plot(x_odom, y_odom, 'r-', label='Lab. 2')
plt.plot(x_odom_lab1, y_odom_lab1, 'b-', label='Lab. 1')
#plt.plot(t_ref, p_ref, 'r-', label='Pref(t)')
plt.xlabel('Eje X')
plt.ylabel('Eje Y')
plt.title('Comparación Ruta Bidimensional Lab. 1 y Lab. 2 Control PI')
plt.grid(True)
plt.legend()
plt.show()
