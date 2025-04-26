import sys
import os

import torch

# Add the libraries directory to the Python path
sys.path.append('../..')


import random

# import numpy as np
import json
from controllers.common.grid import *
# Initialize the Supervisor
# from controllers.common.db_handler import DatabaseHandler
from controllers.common.async_db_handler import SupervisorDBLogger

# Initialize Database (Session starts in constructor)
# await init_pool()
# Global parameters
PHEROMONE_MATRIX = torch.zeros((GRID_SIZE+2, GRID_SIZE+2), dtype=torch.float64, device=device)
PRIORITY_MATRIX = torch.zeros((GRID_SIZE+2, GRID_SIZE+2), dtype=torch.float64, device=device)
N=3#drone count
DELAY=1000
DRONE_POSITIONS_IN_GRID = torch.zeros((N, 2), dtype=torch.int, device=device)
DRONE_POSITIONS=torch.zeros((N, 3), dtype=torch.float64, device=device)
# To track previous drone positions
# Enable the keyboard interface
# def fill_initial_pheromone():
#     global PHEROMONE_MATRIX
#
#     indices = torch.arange(GRID_SIZE+2, dtype=torch.float64, device=device)
#     i_grid, j_grid = torch.meshgrid(indices, indices, indexing='ij')
#
#     dist_top = i_grid
#     dist_bottom = GRID_SIZE+1 - i_grid
#     dist_left = j_grid
#     dist_right = GRID_SIZE+1 - j_grid
#
#     # Use amin to get min along dimension 0 (across the stacked tensor)
#     min_dist = torch.stack([dist_top, dist_bottom, dist_left, dist_right], dim=0).amin(dim=0)
#
#     decay_base = 0.9
#     PHEROMONE_MATRIX = MAX_PHEROMONE * torch.pow(decay_base, min_dist)
# fill_initial_pheromone()

# MAX_T=1000
# PHEROMONE_HISTORY=np.zeros((MAX_T, GRID_SIZE+2, GRID_SIZE+2), dtype=float)
# PRIORITY_HISTORY=np.zeros((MAX_T, GRID_SIZE+2, GRID_SIZE+2), dtype=float)
# DRONE_HISTORY=np.zeros((MAX_T, N, 4), dtype=float)


def regular_polygon_vertices_from_inscribed(N=N, r=10):
    """
    Вычисляет координаты вершин правильного N-угольника, в который вписана окружность радиусом r.

    :param N: Количество сторон (углов)
    :param r: Радиус вписанной окружности
    :param device: Устройство ('cpu' или 'cuda')
    :return: Список кортежей (x, y, theta) - координаты вершин
    """
    if N <= 0:
        raise ValueError("N must be greater than 0")
    elif N == 1:
        return torch.tensor([(0.0, 0.0, 0.0)], device=device)
    elif N == 2:
        return torch.tensor([(0.0, r, torch.pi / 2), (0.0, -r, -torch.pi / 2)], device=device)

    # Радиус описанной окружности
    R = r / torch.cos(torch.tensor(torch.pi / N, device=device))

    # Создаем тензор индексов для вершин от 0 до N-1
    k_values = torch.arange(N, dtype=torch.float64, device=device)

    # Вычисление углов (theta) для всех вершин
    theta_values = 2 * torch.pi * k_values / N

    # Вычисление x и y для всех вершин
    x_values = R * torch.cos(theta_values)
    y_values = R * torch.sin(theta_values)

    # Собираем все в одном тензоре
    vertices = torch.stack((x_values, y_values, theta_values), dim=1)

    return vertices

def detect_drones():
    """
    Detect all Crazyflie drones and their positions.
    """
    root = supervisor.getRoot()
    children_field = root.getField("children")
    global DRONE_POSITIONS
    global DRONE_POSITIONS_IN_GRID
    for i in range(children_field.getCount()):
        node = children_field.getMFNode(i)
        if node.getTypeName() == "Crazyflie":  # Filter by node type
            # Access the 'id' field of the Crazyflie node

            id_field = node.getField("id")
            drone_id = int(id_field.getSFInt32())  # Assuming 'id' is SFInt32

            pos = torch.tensor(node.getPosition(), dtype=torch.float64, device=device)
            DRONE_POSITIONS[drone_id, :] = pos
            DRONE_POSITIONS_IN_GRID[drone_id, :] = find_grid_coordinates(pos)


# def update_pheromone_matrix():
#     """
#     Update the pheromone matrix based on drone positions and decay function.
#     Drones emit pheromones in a way that decays exponentially with distance.
#     """
#     global PHEROMONE_MATRIX
#
#     # Create grid coordinates
#     i_grid, j_grid = torch.meshgrid(
#         torch.arange(GRID_SIZE + 2, dtype=torch.float64, device=device),
#         torch.arange(GRID_SIZE + 2, dtype=torch.float64, device=device),
#         indexing='ij'
#     )
#
#     # Compute new pheromone from all drones at once
#     rows_meshgrid = torch.tensor([pos[0] for pos in DRONE_POSITIONS_IN_GRID], dtype=torch.float64, device=device).view(-1, 1, 1)
#     cols_meshgrid = torch.tensor([pos[1] for pos in DRONE_POSITIONS_IN_GRID], dtype=torch.float64, device=device).view(-1, 1, 1)
#
#     dist_i = torch.abs(rows_meshgrid - i_grid)
#     dist_j = torch.abs(cols_meshgrid - j_grid)
#
#     drone_pheromones = MAX_PHEROMONE / (1 + torch.exp2(dist_i) + torch.exp2(dist_j))  # Shape: (N_drones, H, W)
#     new_pheromone_matrix = drone_pheromones.sum(dim=0)  # Sum over drones
#
#     # Combine with old pheromone (smoothing factor α = 0.5)
#     PHEROMONE_MATRIX = 0.5 * PHEROMONE_MATRIX + 0.5 * new_pheromone_matrix
#
#
#     # Clamp to avoid overflow
#     PHEROMONE_MATRIX = torch.clamp(PHEROMONE_MATRIX, 0, MAX_PHEROMONE)
#     # Assuming DRONE_POSITIONS_IN_GRID is of shape (N, 2) with rows and columns
#     rows = DRONE_POSITIONS_IN_GRID[:, 0]
#     cols = DRONE_POSITIONS_IN_GRID[:, 1]
#     noise_strength = 0.001
#     # Use advanced indexing to set those cells in PHEROMONE_MATRIX
#     PHEROMONE_MATRIX[rows, cols] = MAX_PHEROMONE
#     noise = torch.rand_like(PHEROMONE_MATRIX) * (MAX_PHEROMONE * noise_strength)
#
#     # Apply noise only where pheromone is not maxed out
#     mask = PHEROMONE_MATRIX < MAX_PHEROMONE
#     PHEROMONE_MATRIX = PHEROMONE_MATRIX + noise * mask
#     PHEROMONE_MATRIX = torch.clamp(PHEROMONE_MATRIX, 0, MAX_PHEROMONE)
def update_pheromone_matrix(alpha=0.9):
    """
    Update pheromone matrix with scaling to one first, then applying MAX_PHEROMONE.
    Drone positions are always set directly to MAX_PHEROMONE.
    """
    global PHEROMONE_MATRIX

    N_drones = DRONE_POSITIONS_IN_GRID.shape[0]
    if N_drones == 0:
        return

    indices = torch.arange(GRID_SIZE + 2, dtype=torch.float64, device=device)
    i_grid, j_grid = torch.meshgrid(indices, indices, indexing='ij')

    drone_rows = DRONE_POSITIONS_IN_GRID[:, 0].view(-1, 1, 1)
    drone_cols = DRONE_POSITIONS_IN_GRID[:, 1].view(-1, 1, 1)

    dist_i = torch.abs(drone_rows - i_grid)
    dist_j = torch.abs(drone_cols - j_grid)
    decay = torch.pow(0.9, dist_i + dist_j)  # (N_drones, H, W)

    # Sum the contributions and normalize to [0, 1]
    new_pheromone_matrix = decay.sum(dim=0)
    max_value = new_pheromone_matrix.max()
    if max_value > 0:
        new_pheromone_matrix = new_pheromone_matrix / max_value  # Scaling to one

    # Then apply MAX_PHEROMONE after normalization
    new_pheromone_matrix = MAX_PHEROMONE * new_pheromone_matrix

    # Smooth update
    PHEROMONE_MATRIX = alpha * PHEROMONE_MATRIX + (1 - alpha) * new_pheromone_matrix

    # Set drone positions to MAX_PHEROMONE directly
    rows = DRONE_POSITIONS_IN_GRID[:, 0].long()
    cols = DRONE_POSITIONS_IN_GRID[:, 1].long()
    PHEROMONE_MATRIX[rows, cols] = MAX_PHEROMONE

    # Clamp values just in case
    PHEROMONE_MATRIX = torch.clamp(PHEROMONE_MATRIX, 0, MAX_PHEROMONE)

# def update_priority_matrix():
#     """
#     Compute priority from pheromone:
#     - Inversely proportional
#     - Higher contrast via log2 scaling
#     - Priority ∈ [0, 1]
#     - Drone positions get zero priority
#     """
#     global PRIORITY_MATRIX
#
#     epsilon = 1e-8  # To prevent log(0)
#
#     # Normalize pheromone to [0, 1]
#     norm_pheromone = PHEROMONE_MATRIX / (MAX_PHEROMONE + epsilon)
#
#     # Use log2 to increase diversity
#     log_inv = 1 - torch.log2(norm_pheromone + epsilon) / torch.log2(
#         torch.tensor(1 / epsilon, dtype=torch.float64, device=device))
#
#     # Clamp for safety
#     PRIORITY_MATRIX = torch.clamp(log_inv, 0.0, 1.0)
#
#     # Zero out drone cells
#     for (row, col) in DRONE_POSITIONS_IN_GRID:
#         PRIORITY_MATRIX[int(row), int(col)] = 0.0

def update_priority_matrix(power=3.0):
    """
    Compute sharper inverse priority from pheromone using power-law inversion.

    Args:
        power (float): Controls the sharpness of inversion (e.g., 2.0 for sharper contrast).
    """
    global PRIORITY_MATRIX

    epsilon = 1e-8  # Prevent division by zero

    # Normalize pheromone matrix
    norm_pheromone = PHEROMONE_MATRIX / (MAX_PHEROMONE + epsilon)
    norm_pheromone = torch.clamp(norm_pheromone, epsilon, 1.0)

    # Compute inverse with adjustable sharpness
    inv_pheromone = (1.0 / norm_pheromone) ** power

    # Normalize result to [0, 1]
    max_value = torch.max(inv_pheromone)
    PRIORITY_MATRIX = inv_pheromone / (max_value + epsilon)

    # Zero out drone positions
    # Instead of the loop:
    rows = DRONE_POSITIONS_IN_GRID[:, 0].long()
    cols = DRONE_POSITIONS_IN_GRID[:, 1].long()
    PRIORITY_MATRIX[rows, cols] = 0.0


def get_drone_neighbors_info(drone_id):
    row, col= DRONE_POSITIONS_IN_GRID[drone_id]
    result=dict()
    if 0<row-1<=GRID_SIZE and 0<col-1<=GRID_SIZE:
        result['left_bottom']=PRIORITY_MATRIX[row-1, col-1].item()
    if 0<row-1<=GRID_SIZE and 0<col<=GRID_SIZE:
        result['bottom']=PRIORITY_MATRIX[row-1, col].item()
    if 0<row-1<=GRID_SIZE and 0<col+1<=GRID_SIZE:
        result['right_bottom']=PRIORITY_MATRIX[row-1, col+1].item()
    if 0<row<=GRID_SIZE and 0<col-1<=GRID_SIZE:
        result['left']=PRIORITY_MATRIX[row, col-1].item()
    if 0<row+1<=GRID_SIZE and 0<col-1<=GRID_SIZE:
        result['left_top']=PRIORITY_MATRIX[row+1, col-1].item()
    if 0<row+1<=GRID_SIZE and 0<col<=GRID_SIZE:
        result['top']=PRIORITY_MATRIX[row+1, col].item()
    if 0<row+1<=GRID_SIZE and 0<col+1<=GRID_SIZE:
        result['right_top']=PRIORITY_MATRIX[row+1, col+1].item()
    if 0<row<=GRID_SIZE and 0<col+1<=GRID_SIZE:
        result['right']=PRIORITY_MATRIX[row, col+1].item()
    return result

MESSAGE_ID=0
def prepare_drone_info():
    msg=dict()
    for drone_id in range(N):
        msg[drone_id] = get_drone_neighbors_info(drone_id)

    return msg



def get_floor_size(def_name):
    """
    Access the floor node by its DEF name and retrieve its size.
    """
    floor_node = supervisor.getFromDef(def_name)
    if floor_node is None:
        print(f"Floor node with DEF name '{def_name}' not found!")
        return None

    size_field = floor_node.getField('size')
    if size_field is not None:
        floor_size = size_field.getSFVec2f()  # Retrieve as a 2D vector
        print(f"Floor size: {floor_size}")
        return floor_size
    else:
        print(f"'size' field not found for the floor node '{def_name}'.")
        return None

grid_nodes = []  # Store references to the created grid nodes
grided=False
def visualize_grid(floor_width, floor_depth):
    """
    Visualize a grid on the X-Y plane at z=0.
    """
    global grided
    if grided:
        print('already grided')
        return
    else:
        grided = True

    global grid_nodes  # Reference to store added nodes
    grid_nodes.clear()  # Clear any previous references

    root = supervisor.getRoot()
    children_field = root.getField("children")

    cell_width = floor_width / GRID_SIZE
    cell_depth = floor_depth / GRID_SIZE

    for i in range(GRID_SIZE + 1):
        # Horizontal lines
        y = -floor_depth / 2 + i * cell_depth
        line_horizontal = f"""
        Shape {{
            appearance Appearance {{
                material Material {{ diffuseColor 0.8 0.8 0.8 }}
            }}
            geometry IndexedLineSet {{
                coord Coordinate {{
                    point [
                        {-floor_width / 2} {y} 0,
                        {floor_width / 2} {y} 0
                    ]
                }}
                coordIndex [0 1 -1]
            }}
        }}
        """
        node = children_field.importMFNodeFromString(-1, line_horizontal)
        grid_nodes.append(node)  # Store reference to the node

        # Vertical lines
        x = -floor_width / 2 + i * cell_width
        line_vertical = f"""
        Shape {{
            appearance Appearance {{
                material Material {{ diffuseColor 0.8 0.8 0.8 }}
            }}
            geometry IndexedLineSet {{
                coord Coordinate {{
                    point [
                        {x} {-floor_depth / 2} 0,
                        {x} {floor_depth / 2} 0
                    ]
                }}
                coordIndex [0 1 -1]
            }}
        }}
        """
        node = children_field.importMFNodeFromString(-1, line_vertical)
        grid_nodes.append(node)  # Store reference to the node

    print("Grid visualization added!")


def delete_grid():
    """
    Deletes the previously drawn grid without affecting other objects.
    """
    global grided
    if not grided:
        print('already ungrided')
        return
    else:
        grided = False
    global grid_nodes
    root = supervisor.getRoot()
    children_field = root.getField("children")

    for node in grid_nodes:
        if node is not None and node.exists():
            node.remove()  # Remove each grid line node

    grid_nodes.clear()  # Clear the stored references
    print("Grid visualization removed!")


from controller import Supervisor


class CrazyflieSupervisor(Supervisor):
    def __init__(self):
        super().__init__()
        self.db_logger = SupervisorDBLogger(N)
        self.drones = []  # List to store drone instances
        self.is_running = False
        self.is_paused = False
        self.message_id=0

        # Initialize drones
        self.create_drones()

    def update_history(self):
        self.db_logger.insert_pheromone_matrix(PHEROMONE_MATRIX)
        self.db_logger.insert_priority_matrix(PRIORITY_MATRIX)
        # for i in range(GRID_SIZE + 1):
        #     for j in range(GRID_SIZE + 1):
        #         db.log_priority_matrix(col=i, row=j, priority=PRIORITY_MATRIX[i][j], timestep=self.t)
        #         db.log_pheromone_matrix(col=i, row=j, pheromone=PHEROMONE_MATRIX[i][j], timestep=self.t)

    def save_drone_positions(self):
        for i, (x, y, z) in enumerate(DRONE_POSITIONS):
            # db.log_drone_pos(drone_id=i, position_x=x, position_y=y, timestep=self.t)
            self.db_logger.insert_drone_position(i, x, y)
        # for drone_id, (row, col) in enumerate(DRONE_POSITIONS_IN_GRID):
            # db.log_drone_cell_position(drone_id=drone_id, row=row, col=col, timestep=self.t)
            # self.db_logger.insert_drone_position_in_cell( drone_id, row, col)

    def send_msg(self, command: str, content=None):
        msg = dict()
        # global MESSAGE_ID
        msg['MESSAGE_ID'] = self.message_id
        self.message_id += 1
        msg['COMMAND'] = command
        if content:
            msg['CONTENT'] = content
        json_message = json.dumps(msg, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
        # print('message:', json_message)
        # return json_message.encode()
        full_text=json_message.encode()
        self.emitter.send(full_text)
        # self.db_logger.insert_msg_sent(self.message_id, command, str(content), full_text.decode())
        # db.log_msg_sent(timestep=self.t, command=command, content=content, id=MESSAGE_ID, full_text=full_text)


    def create_drones(self):
        """Create Crazyflie drones at different positions."""
        drone_positions = regular_polygon_vertices_from_inscribed()
        # Get the root node of the world
        root_node = self.getRoot()
        children_field = root_node.getField("children")
        for i in range(N):  # For example, creating 3 drones
            x, y, alpha = drone_positions[i]

            drone_position = [x, y, 0] # Positions for each drone
            drone_node="Crazyflie {\n"+\
                        f"name \"drone_{i}\" \n"+\
                        f"translation {drone_position[0]} {drone_position[1]} 0\n"+ \
                       f"rotation 0 0 1 {alpha}\n" + \
                       f"id \"{i}\"\n"+\
                        "}"
            # f"rotation 0 0 1 {alpha}\n"+\
            # Import the drone into the world (not under the Supervisor)
            children_field.importMFNodeFromString(-1, drone_node)
            self.drones.append(self.getFromDef(f"Drone_{id}"))

    def start_simulation(self):
        """Start the drones' controllers and supervisor loop."""
        self.is_running = True
        # for drone in self.drones:
        #     drone_controller_field = drone.getField("controller")
        #     drone_controller_field.setSFString("drone_controller.py")  # Ensure controller is set

    def pause_simulation(self):
        """Pause the drones' controllers but keep them alive."""
        self.is_paused = True
        print(f'{self.getTime()}: Paused')
        # self.emitter.send(prepare_msg('pause'))
        self.send_msg('pause')
            # drone_controller_field = drone.getField("controller")
            # drone_controller_field.setSFString("")  # Disable controller temporarily
    def send_go_up(self):
        """Send the go up command."""
        # self.emitter.send(prepare_msg('up', 1))
        self.send_msg('up', 1)

        # Add the function behavior here

    # def process_key_press(self):
    #     """Handle key presses to start, pause, or execute functions."""
    #     # print(int(self.getBasicTimeStep()))
    #
    #     # key = self.keyboard.getKey()
    #     if key != -1:
    #         print(f'key: {key}: pressed: {chr(key)}')
    #     if key == ord('S'):  # 'S' key starts the simulation
    #         if not self.is_running:
    #             self.start_simulation()
    #     elif key == ord('P'):  # 'P' key pauses the simulation
    #         if not self.is_paused:
    #             self.pause_simulation()
    #     elif key == ord('Q'):  # 'F' key executes the function
    #         self.quit=True
    #     elif key == ord('U'):
    #         self.send_go_up()
    #     elif key == ord('G'):
    #         visualize_grid(self.floor_width, self.floor_depth)
    #     elif key == ord('H'):
    #         delete_grid()
    def run(self):

        """Main loop of the supervisor."""

        # Get floor size
        floor_def_name = 'floor'  # Replace with your floor's DEF name
        floor_size = get_floor_size(floor_def_name)

        if not floor_size:
            print("Could not retrieve floor size. Exiting...")
            return

        self.floor_width, self.floor_depth = floor_size
        self.emitter = self.getDevice("emitter")
        self.t = 0
        # pos = [random.uniform(-self.floor_width, self.floor_width), random.uniform(-self.floor_depth, self.floor_depth)]
        # self.keyboard.enable(int(self.getBasicTimeStep()))
        delay=0
        self.timestep=int(self.getBasicTimeStep())
        self.quit=False
        self.is_running = False
        while self.step(int(self.getBasicTimeStep())) != -1 and not self.quit:
            self.db_logger.next_step()
            self.t=self.t+1
            # self.process_key_press()
            delay += self.getBasicTimeStep()
            if delay>30000 and not self.is_running and not self.quit:
                self.is_running=True
                continue
            detect_drones()
            if self.is_running and not self.is_paused:
                self.save_drone_positions()
            if self.is_running and self.t % 100 == 0:
                self.update_history()
            if self.is_running and not self.is_paused and delay>DELAY:
                # Supervisor's main loop for controlling simulation if needed
                 # Add your own logic here
                # Detect drones and their positions


                # Update pheromone and priority matrices
                update_pheromone_matrix()
                update_priority_matrix()
                msg = prepare_drone_info()
                # self.emitter.send(prepare_msg('go', msg))
                self.send_msg('go', msg)
                delay = 0

        self.db_logger.close()
        # np.savez_compressed("arrays_compressed.npz", first=PHEROMONE_MATRIX, second=PRIORITY_MATRIX,
        #                     third=DRONE_HISTORY)

if __name__ == '__main__':
    # Create the supervisor and run the simulation
    supervisor = CrazyflieSupervisor()
    supervisor.run()


