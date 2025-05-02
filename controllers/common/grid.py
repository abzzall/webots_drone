import torch
import numpy as np
import random
from datetime import datetime
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}: Using device: {device}")
SEED = 4

random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)
torch.cuda.manual_seed(SEED)
torch.cuda.manual_seed_all(SEED)
torch.backends.cudnn.deterministic = True
torch.backends.cudnn.benchmark = False


GRID_SIZE = torch.tensor( 100, device=device)
FLOOR_WIDTH = torch.tensor(835, device=device)
FLOOR_DEPTH = torch.tensor(913, device=device)
CELL_WIDTH = FLOOR_WIDTH / GRID_SIZE
CELL_DEPTH = FLOOR_DEPTH / GRID_SIZE
MAX_TARGET_DISTANCE = torch.sqrt(CELL_WIDTH**2 + CELL_DEPTH**2)
GRID_BORDERS=torch.zeros((GRID_SIZE + 2, GRID_SIZE + 2, 6), dtype=torch.float64, device=device)
MAX_PHEROMONE = torch.exp2(torch.tensor(20, dtype=torch.float64, device=device))
def init_grid_borders():
    global GRID_BORDERS
    # Create a tensor for grid borders
    half_cell_width = CELL_WIDTH / 2
    half_cell_height = CELL_DEPTH / 2

    # Create the grid indices
    i_indices = torch.arange(GRID_SIZE.item() + 2, device=device).float()
    j_indices = torch.arange(GRID_SIZE.item() + 2, device=device).float()

    # Create meshgrid for i and j indices
    i_grid, j_grid = torch.meshgrid(i_indices, j_indices, indexing='ij')

    # Calculate the X and Y values for the grid
    top_y = -FLOOR_DEPTH / 2 + i_grid*CELL_DEPTH
    right_x = -FLOOR_WIDTH / 2 + j_grid * CELL_WIDTH  # right_x = -FLOOR_WIDTH/2 + j * CELL_WIDTH
    middle_x = right_x - half_cell_width
    left_x = right_x - CELL_WIDTH
    middle_y = top_y - half_cell_height
    bottom_y = top_y - CELL_DEPTH


    # Populate the GRID_BORDERS tensor with vectorized assignment
    GRID_BORDERS[..., 0] = left_x  # left_x
    GRID_BORDERS[..., 1] = middle_x  # middle_x
    GRID_BORDERS[..., 2] = right_x  # right_x
    GRID_BORDERS[..., 3] = bottom_y  # bottom_y
    GRID_BORDERS[..., 4] = middle_y  # middle_y
    GRID_BORDERS[..., 5] = top_y  # top_y

    # Optionally, move the result to GPU if needed
    GRID_BORDERS = GRID_BORDERS.to(device)

init_grid_borders()

def find_grid_coordinates(pos):
    """
    Find the grid cell (row, column) corresponding to the given (x, y) world coordinates.
    Grid starts from 1.
    """
    column = int(torch.floor((pos[0] + FLOOR_WIDTH / 2) / CELL_WIDTH)) + 1
    row = int(torch.floor((pos[1] + FLOOR_DEPTH / 2) / CELL_DEPTH)) + 1
    if not 0 <= row <= GRID_SIZE+1 or not 0 <= column <= GRID_SIZE+1:
        print(f"Coordinates ({pos}) are out of grid bounds!")
        if row<0:
            row=0
        elif row>GRID_SIZE:
            row=GRID_SIZE
        if column<0:
            column=0
        elif column>GRID_SIZE:
            column=GRID_SIZE
    # left_x, middle_x, right_x, bottom_y, middle_y, top_y=grid_borders(row, column)
    #
    #
    # if pos[0] < left_x:
    #     column = column - 1
    # if pos[1] < bottom_y:
    #     row = row - 1
    # if pos[0] > right_x:
    #     column = column + 1
    # if pos[1] > top_y:
    #     row = row + 1


    return torch.tensor([row, column], dtype=torch.int, device=device)

def grid_borders(row, column):
    """
    Given a grid coordinate (row, column), calculate the world-coordinate borders of that grid cell.
    """
    # x_min = (column - 1) * CELL_WIDTH - FLOOR_WIDTH / 2
    # y_min = (row - 1) * CELL_DEPTH - FLOOR_DEPTH / 2
    # x_max = column * CELL_WIDTH - FLOOR_WIDTH / 2
    # y_max = row * CELL_DEPTH - FLOOR_DEPTH / 2

    # return x_min, y_min, x_max, y_max
    return GRID_BORDERS[row, column]
def angle_between_vectors(v1, v2):
    """Calculate the signed angle between two 2D vectors using PyTorch."""
    dot = torch.dot(v1, v2)  # Dot product
    det = v1[0] * v2[1] - v1[1] * v2[0]  # Determinant (cross product in 2D)
    angle = torch.atan2(det, dot)  # Signed angle in radians
    return angle
