import numpy as np

GRID_SIZE = 500  # 500x500 grid
FLOOR_WIDTH = 835
FLOOR_DEPTH = 913
CELL_WIDTH = FLOOR_WIDTH / GRID_SIZE
CELL_DEPTH = FLOOR_DEPTH / GRID_SIZE
MAX_TARGET_DISTANCE = np.sqrt(CELL_WIDTH**2 + CELL_DEPTH**2)

def find_grid_coordinates(x, y):
    """
    Find the grid cell (row, column) corresponding to the given (x, y) world coordinates.
    Grid starts from 1.
    """
    column = int(np.floor((x + FLOOR_WIDTH / 2) / CELL_WIDTH)) + 1
    row = int(np.floor((y + FLOOR_DEPTH / 2) / CELL_DEPTH)) + 1
    x_min, y_min, x_max, y_max=grid_borders(row, column)
    if x < x_min:
        column = column - 1
    if y < y_min:
        row = row - 1
    if x > x_max:
        column = column + 1
    if y > y_max:
        row = row + 1

    if not 0 <= row <= GRID_SIZE+1 or not 0 <= column <= GRID_SIZE+1:
        print(f"Coordinates ({x}, {y}) are out of grid bounds!")
    return row, column

def grid_borders(row, column):
    """
    Given a grid coordinate (row, column), calculate the world-coordinate borders of that grid cell.
    """
    x_min = (column - 1) * CELL_WIDTH - FLOOR_WIDTH / 2
    y_min = (row - 1) * CELL_DEPTH - FLOOR_DEPTH / 2
    x_max = column * CELL_WIDTH - FLOOR_WIDTH / 2
    y_max = row * CELL_DEPTH - FLOOR_DEPTH / 2

    return x_min, y_min, x_max, y_max

def angle_between_vectors(v1, v2):
    """Calculate the signed angle between two 2D vectors using NumPy."""
    dot = np.dot(v1, v2)  # Dot product
    det = np.linalg.det([v1, v2])  # Determinant (cross product in 2D)
    angle = np.arctan2(det, dot)  # Signed angle in radians
    return angle
