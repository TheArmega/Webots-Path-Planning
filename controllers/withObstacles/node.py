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