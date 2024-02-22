
# Planification Algorithms with Webots

En este projecto he desarrollado dos algoritmos de planificación con el objetivo de que una plataforma robótica sea capaz de ir de un punto A a un punto B en un mapa con y sin obstáculos. A parte, se han añadadido una serie de elementos extras para complementar el desarrollo de este trabajo.

## Authors
- [Jaime Mas Santillán](https://www.github.com/TheArmega/Webots-Path-Planning)


## Index
    1. Without obstacles
    2. With obstacles
    3. Laberynth
    4. Grid display
    5. CSV to matrix

## Without obstacles
En esta primera parte de la práctica se nos pedía que una plataforma robótica fuese de un punto A a un punto B del mapa sin que se encontrase en su camino con obstáculos. Para ellos he desarrollado un algoritmo de planificación que toma el nodo actual (node) y las coordenadas de un objetivo (x_goal, y_goal). Utiliza el algoritmo A* para calcular una ruta desde el nodo actual hasta el objetivo y devuelve el ángulo hacia el siguiente nodo en esa ruta. Si no hay camino o el robot ya está en el objetivo, retorna None. El algoritmo se compone basicamente de tres funciones:

- `adjust_yaw_to_target`

```python
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
```

- `calculate_target_angle`
```python
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
```

- `move_robot_towards_goal`
```python
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
```

Con estas tres funciones somos capaces de obtener los soguientes resultados:


https://github.com/TheArmega/Webots-Path-Planning/assets/38068010/dca64454-28e2-445e-9958-d927d9636a79

## Without obstacles

## With Obstacles
Una de las mejoras que he implementado en esta práctica es la planificación de movimiento en un mapa con obstáculos, para ello he implementado el algoritmo A* el cual tiene como función de coste la distancia Manhattan (distancia más corta hacia el objetivo), para ello al código anteriormente visto le he añadido una función que es capaz de detectar si se encuentra algún obstáculo por el camino.

- `move_robot_towards_goal`
```python
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
```

Gracias a esta función he podido obtener los siguientes resultados:


https://github.com/TheArmega/Webots-Path-Planning/assets/38068010/f0599ac6-7d76-496f-bc60-d4e25d0831fc



https://github.com/TheArmega/Webots-Path-Planning/assets/38068010/b2b28149-c7dc-42b9-9804-4c15d063335e


## Laberynth

Otra de las mejoras que he implementado es mejorar el algoritmo A* para que sea capaz de resolver laberintos. Para ello lo que hace es recordar los caminos tomados y cuando no sea capaz de seguir en la dirección indicada por la función de coste volver hacia atrás hasta encontrar otro camino posible que le lleve a la salida.

```python
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

```

Los resultados son los siguientes:


https://github.com/TheArmega/Webots-Path-Planning/assets/38068010/a90df455-f2a1-4f67-9813-476f37dc2f29


## Grid display
Otro de los complementos que se le ha añadido al trabajo es la implementación mediante pygame de un display que muestra el mapa que está recorriendo el robot de una manera más gráfica, indicando los obstáculos, los posibles caminos y el punto de inicio y final.

```python
import pygame
import sys

class GridDrawer:

    def __init__(self, matrix, sizeSquare=20):
        # Inicializa la clase GridDrawer con la matriz y el tamaño de cuadrado especificados
        self.matrix     = matrix
        self.sizeSquare = sizeSquare
        self.rows       = len(matrix)
        self.cols       = len(self.matrix[0]) if self.rows > 0 else 0
        self.size       = (self.cols * self.sizeSquare, self.rows * self.sizeSquare)        
        pygame.init()
        self.screen     = pygame.display.set_mode(self.size)
        pygame.display.set_caption("Grid Display")

    def draw_grid(self):
        # Dibuja el grid en la pantalla
        for y in range(self.rows):
            for x in range(self.cols):
                if (self.matrix[y][x] == 0):
                    color = (255, 255, 255)  # Color blanco para celdas vacías
                elif (self.matrix[y][x] == 2):
                    color = (226, 143, 173)  # Color rosa para obstáculos
                elif (self.matrix[y][x] == 3):
                    color = (229, 83, 0)     # Color naranja para objetivo
                else: 
                    color = (0, 0, 0)        # Color negro para otros valores
                pygame.draw.rect(self.screen, color, (x*self.sizeSquare, y*self.sizeSquare, self.sizeSquare, self.sizeSquare))

        # Dibuja las líneas de delimitación
        for y in range(self.rows + 1):
            pygame.draw.line(self.screen, (0, 0, 0), (0, y * self.sizeSquare), (self.cols * self.sizeSquare, y * self.sizeSquare))
        for x in range(self.cols + 1):
            pygame.draw.line(self.screen, (0, 0, 0), (x * self.sizeSquare, 0), (x * self.sizeSquare, self.rows * self.sizeSquare))

    
    def paintSquare(self, x, y):
        # Pinta un cuadrado en la posición (x, y) de color verde
        color = (0, 255, 0)
        pygame.draw.rect(self.screen, color, (x*self.sizeSquare, y*self.sizeSquare, self.sizeSquare, self.sizeSquare))
        pygame.display.flip()

    def run(self):
        # Inicia el bucle principal del programa
        self.screen.fill((255, 255, 255))  # Llena la pantalla de color blanco
        self.draw_grid()                   # Dibuja el grid
        pygame.display.flip()              # Actualiza la pantalla

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

```

De tal manera que se consigue el siguiente resultado:

<img width="467" alt="Grid" src="https://github.com/TheArmega/Webots-Path-Planning/assets/38068010/d4503536-4e9c-4da3-b51e-20e4e3cdfce6">


## Node
Para que todo funcione de manera correcta se ha implementado la clase nodo, la cual nos ayudará en la tarea de planificación. Esta clase tiene sus propios métodos los cuales iran guiando al robot mediante los datos recibidos por el algoritmo A*. También nos irá sacando por pantalla los datos de los nodos visitados.

```python
from queue import PriorityQueue

class Node:
    
    def __init__(self, x, y, myID, parentID):
        """
        Inicializa un nodo con las coordenadas (x, y), su ID y el ID de su nodo padre.
        """
        self.x = x
        self.y = y
        self.myID = myID
        self.parentID = parentID

    def dump(self):
        """
        Imprime las propiedades del nodo.
        """
        print("---------- x "+str(self.x)+\
                        " | y "+str(self.y)+\
                        " | id "+str(self.myID)+\
                        " | parentId "+str(self.parentID))
        
def heuristic(a, b):
    """
    Calcula la heurística Manhattan entre dos puntos.
    """
    return abs(a.x - b.x) + abs(a.y - b.y)

def get_neighbors(matrix, node):
    """
    Devuelve los vecinos transitables de un nodo dado en la matriz matrix.
    """
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Direcciones: arriba, derecha, abajo, izquierda
    neighbors = []
    for dx, dy in directions:
        x, y = node.x + dx, node.y + dy
        if 0 <= x < len(matrix[0]) and 0 <= y < len(matrix) and matrix[y][x] == 0:
            neighbors.append(Node(x, y))
    return neighbors

def a_star_search(matrix, start, goal):
    """
    Realiza la búsqueda A* en una matriz matrix para encontrar el camino desde start hasta goal.
    """
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while not open_set.empty():
        current = open_set.get()[1]

        if current.x == goal.x and current.y == goal.y:
            path = []
            while current:
                path.append(current)
                current = current.parent
            return path[::-1]

        for neighbor in get_neighbors(matrix, current):
            tentative_g_score = g_score.get(current, float('inf')) + 1

            if tentative_g_score < g_score.get(neighbor, float('inf')):
                neighbor.parent = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if not any(neighbor == item[1] for item in open_set.queue):
                    open_set.put((f_score[neighbor], neighbor))

    return None
```

## CSV to matrix
También se ha creado una función que permite coger el CSV del mapa y pasarlo a una matriz de 1s y 0s para que sea más fácil calcular la trayectoria del robot.

```python
import csv
import numpy

class csvToMatrix:
    def __init__(self, mapRoute):
        # Initialize the csvToMatrix object with the mapRoute parameter
        self.mapRoute = mapRoute

    def transform(self):
        # Open the CSV file using the mapRoute parameter and read it as text
        reader = csv.reader(open(self.mapRoute, "rt"), delimiter=",")

        # Convert the CSV data into a list of lists
        x = list(reader)

        # Convert the list of lists into a numpy array of integers
        result = numpy.array(x).astype("int")

        # Return the resulting numpy array
        return result
```
