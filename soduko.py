import pygame
import sys
import random

# Initialize Pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 540, 600
SCREEN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Sudoku Master - Generator Edition")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (50, 120, 255)
TEXT_COLOR = (30, 30, 30)

# Game Settings
CELL_SIZE = WIDTH // 9

class SudokuGenerator:
    def __init__(self):
        self.board = [[0 for _ in range(9)] for _ in range(9)]

    def is_valid(self, board, r, c, num):
        for i in range(9):
            if board[r][i] == num or board[i][c] == num:
                return False
        start_row, start_col = 3 * (r // 3), 3 * (c // 3)
        for i in range(3):
            for j in range(3):
                if board[start_row + i][start_col + j] == num:
                    return False
        return True

    def solve(self, board):
        for r in range(9):
            for c in range(9):
                if board[r][c] == 0:
                    nums = list(range(1, 10))
                    random.shuffle(nums)
                    for num in nums:
                        if self.is_valid(board, r, c, num):
                            board[r][c] = num
                            if self.solve(board):
                                return True
                            board[r][c] = 0
                    return False
        return True

    def generate_puzzle(self, difficulty=40):
        self.board = [[0 for _ in range(9)] for _ in range(9)]
        self.solve(self.board)
        attempts = difficulty
        while attempts > 0:
            r = random.randint(0, 8)
            c = random.randint(0, 8)
            if self.board[r][c] != 0:
                self.board[r][c] = 0
                attempts -= 1
        return self.board

class SudokuGame:
    def __init__(self):
        self.generator = SudokuGenerator()
        self.font = pygame.font.SysFont("comicsans", 40)
        self.small_font = pygame.font.SysFont("comicsans", 20)
        self.reset_game()

    def reset_game(self):
        self.board = self.generator.generate_puzzle(difficulty=40)
        # Track which cells are part of the original puzzle
        self.fixed_cells = [[(self.board[r][c] != 0) for c in range(9)] for r in range(9)]
        self.selected = None

    def draw(self):
        SCREEN.fill(WHITE)
        
        # Draw Grid
        for i in range(10):
            thickness = 4 if i % 3 == 0 else 1
            pygame.draw.line(SCREEN, BLACK, (0, i * CELL_SIZE), (WIDTH, i * CELL_SIZE), thickness)
            pygame.draw.line(SCREEN, BLACK, (i * CELL_SIZE, 0), (i * CELL_SIZE, WIDTH), thickness)

        # Draw Numbers
        for r in range(9):
            for c in range(9):
                val = self.board[r][c]
                if val != 0:
                    # If it's a fixed cell, use black. If user-entered, use a gray color.
                    color = BLACK if self.fixed_cells[r][c] else (80, 80, 80)
                    text = self.font.render(str(val), True, color)
                    text_rect = text.get_rect(center=(c * CELL_SIZE + CELL_SIZE // 2, r * CELL_SIZE + CELL_SIZE // 2))
                    SCREEN.blit(text, text_rect)

        # Draw Selection Box
        if self.selected:
            r, c = self.selected
            pygame.draw.rect(SCREEN, BLUE, (c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE), 3)

        # UI Text
        instr = self.small_font.render("Press 'R' to Regenerate | Click to select | Type to fill", True, BLACK)
        SCREEN.blit(instr, (10, 550))

    def handle_click(self, pos):
        x, y = pos
        row = y // CELL_SIZE
        col = x // CELL_SIZE
        if 0 <= row < 9 and 0 <= col < 9:
            self.selected = (row, col)

    def handle_key(self, event):
        if self.selected:
            r, c = self.selected
            if not self.fixed_cells[r][c]: # Only allow editing if not a starting number
                if event.unicode.isdigit():
                    self.board[r][c] = int(event.unicode)
                elif event.key in [pygame.K_BACKSPACE, pygame.K_DELETE]:
                    self.board[r][c] = 0

def main():
    game = SudokuGame()
    clock = pygame.time.Clock()

    while True:
        game.draw()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                game.handle_click(event.pos)
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    game.reset_game()
                else:
                    game.handle_key(event)

        pygame.display.flip()
        clock.tick(30)

if __name__ == "__main__":
    main()
