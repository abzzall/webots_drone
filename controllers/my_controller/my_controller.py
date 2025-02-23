import sys
import os

# Add the libraries directory to the Python path
sys.path.append('../..')


import random

import numpy as np
import json
from controllers.common.grid import *
# Initialize the Supervisor

# Global parameters
PHEROMONE_MATRIX = np.zeros((GRID_SIZE+2, GRID_SIZE+2), dtype=float)
PRIORITY_MATRIX = np.zeros((GRID_SIZE+2, GRID_SIZE+2), dtype=float)
N=3#drone count
DRONE_POSITIONS_IN_GRID = np.zeros((N, 2), dtype=int)
DRONE_POSITIONS=np.zeros((N, 2), dtype=float)
# To track previous drone positions
# Enable the keyboard interface


MAX_T=1000
PHEROMONE_HISTORY=np.zeros((MAX_T, GRID_SIZE+2, GRID_SIZE+2), dtype=float)
PRIORITY_HISTORY=np.zeros((MAX_T, GRID_SIZE+2, GRID_SIZE+2), dtype=float)
DRONE_HISTORY=np.zeros((MAX_T, N, 4), dtype=float)


def regular_polygon_vertices_from_inscribed(N=N, r=1):
    """
    Вычисляет координаты вершин правильного N-угольника, в который вписана окружность радиусом r.

    :param N: Количество сторон (углов)
    :param r: Радиус вписанной окружности
    :return: Список кортежей (x, y) - координаты вершин
    """
    if N<=0:
        raise ValueError("N must be greater than 0")
    elif N==1:
        return [(0.0, 0.0)]
    elif N==2:
        return [(0.0, r), (0.0, -r)]

    R = r / np.cos(np.pi / N)  # Радиус описанной окружности
    vertices = []

    for k in range(N):
        theta = 2 * np.pi * k / N  # Угол в радианах
        x = R * np.cos(theta)
        y = R * np.sin(theta)
         # Угол от центра до вершины
        vertices.append((x, y, theta))
    return vertices

def detect_drones():
    """
    Detect all Crazyflie drones and their positions.
    """
    drones = []
    root = supervisor.getRoot()
    children_field = root.getField("children")
    for i in range(children_field.getCount()):
        node = children_field.getMFNode(i)
        if node.getTypeName() == "Crazyflie":  # Filter by node type
            # Access the 'id' field of the Crazyflie node
            id_field = node.getField("id")
            drone_id = int(id_field.getSFInt32())  # Assuming 'id' is SFInt32

            position = node.getPosition()
            if position[2] > 0.1:  # Ensure the drone is not "fallen" (height > 0.1)
                drones.append((drone_id, position))
    return drones


def update_pheromone_matrix(drones, floor_width, floor_depth):
    """
    Update the pheromone matrix based on the drone positions and old pheromone values.
    All drones that are not fallen affect the pheromones, regardless of movement.
    """
    global PHEROMONE_MATRIX
    new_pheromone_matrix = np.zeros_like(PHEROMONE_MATRIX)
    global DRONE_POSITIONS_IN_GRID
    for drone_id, position in drones:
        x, y, z = position
        if z < 0.1:  # Skip fallen drones (height less than 0.1)
            continue
        DRONE_POSITIONS[drone_id, :] = [x, y]
        row, col = find_grid_coordinates(x, y)
        if row is not None and col is not None:
            new_pheromone_matrix[row, col] = 1024
            DRONE_POSITIONS_IN_GRID[drone_id,:] = [row, col]

    for i in range(1, GRID_SIZE + 1):
        for j in range(1, GRID_SIZE + 1):
                for drone_id, (row, col) in enumerate(DRONE_POSITIONS_IN_GRID):
                    dist_row, dist_col = abs(row - i), abs(col - j)
                    new_pheromone_matrix[i, j]+=1024/(np.exp2( dist_row)+np.exp2( dist_col)+1)


    # Combine with old pheromone values
    PHEROMONE_MATRIX = (PHEROMONE_MATRIX  + new_pheromone_matrix)/2
    PHEROMONE_MATRIX = np.clip(PHEROMONE_MATRIX, 0, 1024)

def update_history(t:int):
    PHEROMONE_HISTORY[t, :, :] = PHEROMONE_MATRIX
    PRIORITY_HISTORY[t, :, :] = PRIORITY_MATRIX
    DRONE_HISTORY[t, :, :2] = DRONE_POSITIONS
    DRONE_HISTORY[t, :, 2:] = DRONE_POSITIONS_IN_GRID
def update_priority_matrix():
    """
    Update the priority matrix based on the pheromone matrix.
    """
    global PRIORITY_MATRIX
    epsilon = 1e-30  # Small value to avoid log issues
    PRIORITY_MATRIX = 1 - 1 / (11 - np.log2(PHEROMONE_MATRIX + epsilon))

def get_drone_neighbors_info(drone_id):
    row, col= DRONE_POSITIONS_IN_GRID[drone_id]
    result=dict()
    if 0<row-1<=GRID_SIZE and 0<col-1<=GRID_SIZE:
        result['left_bottom']=PRIORITY_MATRIX[row-1, col-1]
    if 0<row-1<=GRID_SIZE and 0<col<=GRID_SIZE:
        result['bottom']=PRIORITY_MATRIX[row-1, col]
    if 0<row-1<=GRID_SIZE and 0<col+1<=GRID_SIZE:
        result['right_bottom']=PRIORITY_MATRIX[row-1, col+1]
    if 0<row<=GRID_SIZE and 0<col-1<=GRID_SIZE:
        result['left']=PRIORITY_MATRIX[row, col-1]
    if 0<row+1<=GRID_SIZE and 0<col-1<=GRID_SIZE:
        result['left_top']=PRIORITY_MATRIX[row+1, col-1]
    if 0<row+1<=GRID_SIZE and 0<col<=GRID_SIZE:
        result['top']=PRIORITY_MATRIX[row+1, col]
    if 0<row+1<=GRID_SIZE and 0<col+1<=GRID_SIZE:
        result['right_top']=PRIORITY_MATRIX[row+1, col+1]
    if 0<row<=GRID_SIZE and 0<col+1<=GRID_SIZE:
        result['right']=PRIORITY_MATRIX[row, col+1]
    return result

MESSAGE_ID=0
def prepare_drone_info(drones):
    msg=dict()
    for drone_id, position in drones:
        msg[drone_id] = get_drone_neighbors_info(drone_id)

    return msg

def prepare_msg(command: str, content = None):
    msg=dict()
    global MESSAGE_ID
    msg['MESSAGE_ID'] = MESSAGE_ID
    MESSAGE_ID+=1
    msg['COMMAND'] = command
    if content:
        msg['CONTENT'] = content
    json_message = json.dumps(msg, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    # print('message:', json_message)
    return json_message.encode()

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


def check_if_found(pos, epsilon=0.001):
    for drone_id, [x, y] in enumerate(DRONE_POSITIONS):
        if abs(pos[0] - x) + abs(pos[1] - y) < epsilon:
            print(f'Drone: {drone_id} found object')
            return True
    return False


from controller import Supervisor


class CrazyflieSupervisor(Supervisor):
    def __init__(self):
        super().__init__()
        self.drones = []  # List to store drone instances
        self.is_running = False
        self.is_paused = False

        # Initialize drones
        self.create_drones()

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
        self.emitter.send(prepare_msg('pause'))
            # drone_controller_field = drone.getField("controller")
            # drone_controller_field.setSFString("")  # Disable controller temporarily
    def send_go_up(self):
        """Send the go up command."""
        self.emitter.send(prepare_msg('up', 1))
    def execute_function(self):
        """Execute the specific function when 'F' is pressed."""
        print("Executing function F...")
        np.savez_compressed("arrays_compressed.npz", first=PHEROMONE_MATRIX, second=PRIORITY_MATRIX,
                            third=DRONE_HISTORY)

        # Add the function behavior here

    def process_key_press(self):
        """Handle key presses to start, pause, or execute functions."""
        # print(int(self.getBasicTimeStep()))

        key = self.keyboard.getKey()
        if key != -1:
            print(f'key: {key}: pressed: {chr(key)}')
        if key == ord('S'):  # 'S' key starts the simulation
            if not self.is_running:
                self.start_simulation()
        elif key == ord('P'):  # 'P' key pauses the simulation
            if not self.is_paused:
                self.pause_simulation()
        elif key == ord('F'):  # 'F' key executes the function
            self.execute_function()
        elif key == ord('U'):
            self.send_go_up()
        elif key == ord('G'):
            visualize_grid(self.floor_width, self.floor_depth)
        elif key == ord('H'):
            delete_grid()
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
        t = 0
        pos = [random.uniform(-self.floor_width, self.floor_width), random.uniform(-self.floor_depth, self.floor_depth)]
        self.keyboard.enable(int(self.getBasicTimeStep()))
        delay=0
        while self.step(int(self.getBasicTimeStep())) != -1 and t<MAX_T and not check_if_found(pos):
            self.process_key_press()
            delay += self.getBasicTimeStep()
            if self.is_running and not self.is_paused and delay>1000:
                # Supervisor's main loop for controlling simulation if needed
                 # Add your own logic here
                # Detect drones and their positions
                drones = detect_drones()

                # Update pheromone and priority matrices
                update_pheromone_matrix(drones, self.floor_width, self.floor_depth)
                update_priority_matrix()
                msg = prepare_drone_info(drones)
                self.emitter.send(prepare_msg('go', msg))
                update_history(t)
                t = t + 1
                delay = 0


        np.savez_compressed("arrays_compressed.npz", first=PHEROMONE_MATRIX, second=PRIORITY_MATRIX,
                            third=DRONE_HISTORY)

if __name__ == '__main__':
    # Create the supervisor and run the simulation
    supervisor = CrazyflieSupervisor()
    supervisor.run()


