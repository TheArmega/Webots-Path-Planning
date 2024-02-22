import pygame
import sys

class GridDrawer:

    def __init__(self, matrix, sizeSquare=20):
        self.matrix     = matrix
        self.sizeSquare = sizeSquare
        self.rows       = len(matrix)
        self.cols       = len(self.matrix[0]) if self.rows > 0 else 0
        self.size       = (self.cols * self.sizeSquare, self.rows * self.sizeSquare)        
        pygame.init()
        self.screen     = pygame.display.set_mode(self.size)
        pygame.display.set_caption("Grid Display")

    def draw_grid(self):
        for y in range(self.rows):
            for x in range(self.cols):
                color = (255, 255, 255) if self.matrix[y][x] == 0 else (0, 0, 0)
                pygame.draw.rect(self.screen, color, (x*self.sizeSquare, y*self.sizeSquare, self.sizeSquare, self.sizeSquare))

        # Dibuja las líneas de delimitación
        for y in range(self.rows + 1):
            pygame.draw.line(self.screen, (0, 0, 0), (0, y * self.sizeSquare), (self.cols * self.sizeSquare, y * self.sizeSquare))
        for x in range(self.cols + 1):
            pygame.draw.line(self.screen, (0, 0, 0), (x * self.sizeSquare, 0), (x * self.sizeSquare, self.rows * self.sizeSquare))

    
    def paintSquare(self, x, y):
        color = (0, 255, 0)
        pygame.draw.rect(self.screen, color, (x*self.sizeSquare, y*self.sizeSquare, self.sizeSquare, self.sizeSquare))
        pygame.display.flip()

    def run(self):
        self.screen.fill((255, 255, 255))
        self.draw_grid()
        pygame.display.flip()

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
