import psycopg2
import pandas as pd
import matplotlib.pyplot as plt

# Connect to PostgreSQL
conn = psycopg2.connect(
    dbname="webots_drone_async",
    user="postgres",
    password="1",
    host="localhost",
    port="5432"
)

# Read data
query = """
SELECT drone_id, timestep, position_x, position_y
FROM position_logs
WHERE episode_id = 17 and timestep =109
ORDER BY drone_id, timestep;
"""

df = pd.read_sql(query, conn)
conn.close()

# Plotting
plt.figure(figsize=(10, 8))
for drone_id in df['drone_id'].unique():
    # if drone_id == 2:
    #     continue
    drone_data = df[df['drone_id'] == drone_id]
    plt.plot(drone_data['position_x'], drone_data['position_y'], marker='o', label=f'Drone {drone_id}')

plt.title("Drone Trajectories for Episode 17")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()