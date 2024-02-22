import pygame
import sys
import numpy as np

from controller import Robot
from threading import Thread

from csvToMatrix import csvToMatrix
from GridDrawer  import GridDrawer
from node        import Node
from aStar       import aStar

#routeMap = r"C:\Users\sanma\Documents\practica-webots\worlds\mapWithoutObstacles\mapWithoutObstacles.csv"
routeMap = r"C:\Users\sanma\Documents\practica-webots\worlds\mapWithObstacles\mapWithoutObstacles.csv"

# Transform the csv file into a 1s and 0s matrix
matrixClass = csvToMatrix(routeMap)
matrix = matrixClass.transform()

# Draw the map in 2D
gridDrawer = None
def run_grid():
    global gridDrawer
    gridDrawer = GridDrawer(matrix)
    gridDrawer.run()

# Crear y empezar un nuevo hilo para el bucle de Pygame
thread = Thread(target=run_grid)
thread.start()

aStar = aStar()

# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
#ds = robot.getDevice('dsname')
gps = robot.getDevice('gps')
imu = robot.getDevice('inertial unit')

#ds.enable(timestep)
gps.enable(timestep)
imu.enable(timestep)

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

# Global variables I will need
ID = 0
parentID = -2
gps_x_aux = 0
gps_y_aux = 0
visitedNodes = set()
x_goal = 8
y_goal = 16
control = True

while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #val = ds.getValue()
    gps_vals = gps.getValues()
    imu_rads = imu.getRollPitchYaw()
    roll, pitch, yaw = imu.getRollPitchYaw()

    if ((gps_x_aux != int(gps_vals[0]) or gps_y_aux != int(gps_vals[1])) or control):

        node = Node(int(gps_vals[0]), int(gps_vals[1]), ID, parentID)

        # Add the node to the list of visited nodes
        visitedNodes.add(node)
        if (node.parentID != -2):
            gridDrawer.paintSquare(int(gps_vals[1]), int(gps_vals[0]))
        node.dump()

        # Update the node in the matrix so that indicates that this node have been visited
        if(node.parentID == -2):
            matrix[int(gps_vals[0]), int(gps_vals[1])] = 5
        else: matrix[int(gps_vals[0]), int(gps_vals[1])] = 2


        # Update the ID values for the node that will be visited in the future
        parentID = ID
        ID += 1
        gps_x_aux = int(gps_vals[0])
        gps_y_aux = int(gps_vals[1])

    aStar.move_robot_towards_goal(node, yaw, x_goal, y_goal, motor_right, motor_left)

    if (node.x == x_goal and node.y == y_goal):
        print("CONGRATULATIONS YOU HAVE MADE IT!!!")
        motor_left.setVelocity(0.0)
        motor_right.setVelocity(0.0)
        thread.join()
        break

    control = False

# Enter here exit cleanup code.
print('Bye from Python!')