import pygame
import sys
import numpy as np

from controller import Robot
from threading import Thread

from csvToMatrix import csvToMatrix
from GridDrawer  import GridDrawer
from node        import Node
from aStar       import aStar

# Ruta del archivo CSV que representa el mapa
routeMap = r'C:\Users\sanma\Documents\practica-webots\worlds\maze.csv'

# Transformar el archivo CSV en una matriz de 1s y 0s
matrixClass = csvToMatrix(routeMap)
matrix = matrixClass.transform()

# Dibujar el mapa en 2D
gridDrawer = None
def run_grid():
    global gridDrawer
    gridDrawer = GridDrawer(matrix)
    gridDrawer.run()

# Crear y empezar un nuevo hilo para el bucle de Pygame
thread = Thread(target=run_grid)
thread.start()

# Crear una instancia de la clase aStar
aStar = aStar()

# Crear una instancia del robot
robot = Robot()

# Obtener el intervalo de tiempo del mundo actual
timestep = int(robot.getBasicTimeStep())

# Obtener los dispositivos del robot
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
gps = robot.getDevice('gps')
imu = robot.getDevice('inertial unit')

# Habilitar los dispositivos
gps.enable(timestep)
imu.enable(timestep)

# Configurar las velocidades iniciales de los motores
motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

# Configurar las posiciones de los motores
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

# Variables globales necesarias
ID = 0
parentID = -2
gps_x_aux = 0
gps_y_aux = 0
visitedNodes = []
x_goal = 0
y_goal = 7
control = True
atascado = True
visited_x_aux = -1
visited_y_aux = -1

while robot.step(timestep) != -1:
    # Leer los sensores
    gps_vals = gps.getValues()
    imu_rads = imu.getRollPitchYaw()
    roll, pitch, yaw = imu.getRollPitchYaw()

    if ((gps_x_aux != int(gps_vals[0]) or gps_y_aux != int(gps_vals[1])) or control):
        # Crear un nodo con las coordenadas actuales
        node = Node(int(gps_vals[0]), int(gps_vals[1]), ID, parentID)

        # Agregar el nodo a la lista de nodos visitados
        if (node.parentID == -2):
            visitedNodes.append(node)
        else:
            gridDrawer.paintSquare(int(gps_vals[1]), int(gps_vals[0]))
            for i in range(len(visitedNodes)):
                if visitedNodes[i].x == node.x and visitedNodes[i].y == node.y:
                    break
            else:
                visitedNodes.append(node)
                node.dump()

        # Actualizar la matriz para indicar que este nodo ha sido visitado
        if(node.parentID == -2):
            matrix[int(gps_vals[0]), int(gps_vals[1])] = 5
        else:
            matrix[int(gps_vals[0]), int(gps_vals[1])] = 2

        # Actualizar los valores de ID para el próximo nodo a visitar
        parentID = ID
        ID += 1
        gps_x_aux = int(gps_vals[0])
        gps_y_aux = int(gps_vals[1])

    if atascado == False and (visited_x_aux != int(gps_vals[0]) or visited_y_aux != int(gps_vals[1])):
        # Sacar el último nodo visitado de la lista
        node = visitedNodes.pop()
        visited_x_aux = int(gps_vals[0])
        visited_y_aux = int(gps_vals[1])

    # Mover el robot hacia la meta utilizando el algoritmo A*
    atascado = aStar.move_robot_towards_goal(node, yaw, x_goal, y_goal, motor_right, motor_left, matrix, visitedNodes)

    if (node.x == x_goal and node.y == y_goal):
        print("CONGRATULATIONS YOU HAVE MADE IT!!!")
        motor_left.setVelocity(0.0)
        motor_right.setVelocity(0.0)
        thread.join()
        break

    control = False

print('Bye from Python!')