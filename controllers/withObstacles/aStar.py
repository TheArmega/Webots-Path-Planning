import math

from node import Node

class aStar:
    """
    Clase que implementa el algoritmo A* para la búsqueda de caminos en una matriz con obstáculos.
    """

    def __init__(self):
        pass

    def calculate_target_angle_with_pathfinding(self, node, x_goal, y_goal):
        """
        Calcula el ángulo objetivo en radianes teniendo en cuenta los obstáculos en la matriz.
        :param node: Nodo actual del robot.
        :param x_goal: Coordenada x del objetivo.
        :param y_goal: Coordenada y del objetivo.
        :return: Ángulo objetivo en radianes.
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
        :param yaw: Yaw actual del robot.
        :param target_yaw: Yaw objetivo del robot.
        :return: Diferencia de ángulo más corta para ajustar el yaw.
        """
        # Normalizar ángulos entre -pi y pi
        angle_diff = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        return angle_diff

    def calculate_target_angle(self, node, x_goal, y_goal):
        """
        Calcula el ángulo objetivo en radianes utilizando la función atan2.
        :param node: Nodo actual del robot.
        :param x_goal: Coordenada x del objetivo.
        :param y_goal: Coordenada y del objetivo.
        :return: Ángulo objetivo en radianes.
        """
        dx = x_goal - node.x
        dy = y_goal - node.y

        # Calcula el ángulo objetivo usando atan2, que tiene en cuenta el cuadrante del ángulo
        target_angle = math.atan2(dy, dx)
        return target_angle
    
    def move_robot_towards_goal(self, node, yaw, x_goal, y_goal, motor_right, motor_left, matrix, visitedNodes):
        """
        Mueve el robot hacia el objetivo utilizando el algoritmo A* y ajustando el yaw.
        :param node: Nodo actual del robot.
        :param yaw: Yaw actual del robot.
        :param x_goal: Coordenada x del objetivo.
        :param y_goal: Coordenada y del objetivo.
        :param motor_right: Motor derecho del robot.
        :param motor_left: Motor izquierdo del robot.
        :param matrix: Matriz que representa el entorno del robot.
        :param visitedNodes: Lista de nodos visitados durante la búsqueda.
        :return: True si el robot se mueve hacia el objetivo, False si no hay camino disponible.
        """

        # Calculate Manhattan distances in all four directions
        distances = {
            'up': (abs(node.x - 1 - x_goal) + abs(node.y - y_goal)),
            'down': (abs(node.x + 1 - x_goal) + abs(node.y - y_goal)),
            'right': (abs(node.x - x_goal) + abs(node.y + 1 - y_goal)),
            'left': (abs(node.x - x_goal) + abs(node.y - 1 - y_goal))
        }
        
        # Sort distances and iterate to find a direction to move
        sorted_distances = sorted(distances, key=distances.get)
        
        control = False

        for direction in sorted_distances:
            dx, dy = 0, 0

            if direction == 'up' and (0 <= node.x - 1 < len(matrix)) and (matrix[node.x - 1][node.y] == 0 or matrix[node.x - 1][node.y] == 3):
                dx = node.x - 1
                dy = node.y
                control = True
                break
            elif direction == 'down' and (0 <= node.x + 1 < len(matrix)) and (matrix[node.x + 1][node.y] == 0 or matrix[node.x + 1][node.y] == 3):
                dx = node.x + 1
                dy = node.y
                control = True
                break
            elif direction == 'right' and (0 <= node.y + 1 < len(matrix[0])) and (matrix[node.x][node.y + 1] == 0 or matrix[node.x][node.y + 1] == 3):
                dx = node.x
                dy = node.y + 1
                control = True
                break
            elif direction == 'left' and (0 <= node.y - 1 < len(matrix[0])) and (matrix[node.x][node.y - 1] == 0 or matrix[node.x][node.y - 1] == 3):
                dx = node.x
                dy = node.y - 1
                control = True
                break
        
        if control == False:
            for i in range(0, len(visitedNodes)):
                if node.parentID == visitedNodes[i].myID:
                    nodoAux = visitedNodes[i]
                    break
            dx = nodoAux.x
            dy = nodoAux.y
        
        target_angle = self.calculate_target_angle(node, dx, dy)
        if target_angle is None:
            print("Robot is at the goal.")
            return control        

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
        
        return control
            