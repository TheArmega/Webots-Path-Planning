import math

from node import Node

class aStar:
    """
    Clase que implementa el algoritmo A* para la búsqueda de caminos en una matriz.
    """

    def __init__(self):
        pass

    def calculate_target_angle_with_pathfinding(self, node, x_goal, y_goal):
        """
        Calcula el ángulo objetivo en radianes teniendo en cuenta los obstáculos en la matriz.
        
        Args:
            node (Node): El nodo actual.
            x_goal (float): La coordenada x del objetivo.
            y_goal (float): La coordenada y del objetivo.
        
        Returns:
            float: El ángulo objetivo en radianes.
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
        
        Args:
            yaw (float): El yaw actual.
            target_yaw (float): El yaw objetivo.
        
        Returns:
            float: La diferencia de ángulo más corta para ajustar el yaw actual hacia el objetivo.
        """
        # Normalizar ángulos entre -pi y pi
        angle_diff = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        return angle_diff

    def calculate_target_angle(self, node, x_goal, y_goal):
        """
        Calcula el ángulo objetivo en radianes usando las coordenadas del objetivo.
        
        Args:
            node (Node): El nodo actual.
            x_goal (float): La coordenada x del objetivo.
            y_goal (float): La coordenada y del objetivo.
        
        Returns:
            float: El ángulo objetivo en radianes.
        """
        dx = x_goal - node.x
        dy = y_goal - node.y

        # Calcula el ángulo objetivo usando atan2, que tiene en cuenta el cuadrante del ángulo
        target_angle = math.atan2(dy, dx)
        return target_angle
    
    def move_robot_towards_goal(self, node, yaw, x_goal, y_goal, motor_right, motor_left):
        """
        Mueve el robot hacia el objetivo ajustando el ángulo y controlando los motores.
        
        Args:
            node (Node): El nodo actual.
            yaw (float): El yaw actual.
            x_goal (float): La coordenada x del objetivo.
            y_goal (float): La coordenada y del objetivo.
            motor_right (Motor): El motor derecho del robot.
            motor_left (Motor): El motor izquierdo del robot.
        """
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