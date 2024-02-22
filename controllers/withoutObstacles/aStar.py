import math

from node import Node

class aStar:

    def __init__(self):
        pass

    def calculate_target_angle_with_pathfinding(self, node, x_goal, y_goal):
        """
        Calcula el ángulo objetivo en radianes teniendo en cuenta los obstáculos en la matriz.
        """
        start = Node(node.x, node.y)
        goal = Node(x_goal, y_goal)
        path = Node.a_star_search(self.grid, start, goal)

        if path is None or len(path) < 2:  # Si no hay camino o el robot ya está en el objetivo
            return None

        # Calcula el ángulo hacia el siguiente nodo en el camino
        next_node = path[1]  # El primer nodo es la posición actual, así que tomamos el segundo
        dx = next_node.x - node.x
        dy = next_node.y - node.y
        return math.atan2(dy, dx)

    def adjust_yaw_to_target(self, yaw, target_yaw):
        """
        Calcula la diferencia de ángulo más corta para ajustar el yaw actual hacia el objetivo.
        Retorna el valor que indica si girar a la izquierda o derecha y cuánto debe girar.
        """
        # Normalizar ángulos entre -pi y pi
        angle_diff = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        return angle_diff

    def calculate_target_angle(self, node, x_goal, y_goal):

        dx = x_goal - node.x
        dy = y_goal - node.y

        # Calcula el ángulo objetivo usando atan2, que tiene en cuenta el cuadrante del ángulo
        target_angle = math.atan2(dy, dx)
        return target_angle
    
    def move_robot_towards_goal(self, node, yaw, x_goal, y_goal, motor_right, motor_left):
        target_angle = self.calculate_target_angle(node, x_goal, y_goal)
        if target_angle is None:
            print("Robot is at the goal.")
            return

        angle_to_adjust = self.adjust_yaw_to_target(yaw, target_angle)

        # Determinar si es necesario girar el robot
        if abs(angle_to_adjust) > 0.1:  # Umbral de tolerancia para el giro
            # Girar robot
            if angle_to_adjust > 0:
                motor_left.setVelocity(-1.0)  # Girar a la izquierda
                motor_right.setVelocity(1.0)
            else:
                motor_left.setVelocity(1.0)   # Girar a la derecha
                motor_right.setVelocity(-1.0)
        else:
            # Mover hacia adelante una vez que esté alineado
            motor_left.setVelocity(5.0)
            motor_right.setVelocity(5.0)