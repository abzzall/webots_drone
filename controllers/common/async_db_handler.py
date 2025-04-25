import asyncio
import asyncpg
import torch
import os
from dotenv import load_dotenv

load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")

class BaseDBLogger:
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.pool = self.loop.run_until_complete(asyncpg.create_pool(DATABASE_URL))
        self.timestep = -1

    def next_step(self):
        self.timestep += 1

    async def _close_pool(self):
        await self.pool.close()

    def close(self):
        self.loop.run_until_complete(self._close_pool())
        self.loop.close()


class SupervisorDBLogger(BaseDBLogger):
    def __init__(self, drone_count):
        super().__init__()
        self.episode_id = self.loop.run_until_complete(self._create_episode(drone_count))

    async def _create_episode(self, drone_count):
        async with self.pool.acquire() as conn:
            return await conn.fetchval("INSERT INTO episodes (drone_count) VALUES ($1) RETURNING id;", drone_count)

    def insert_pheromone_matrix(self, cuda_tensor):
        cpu_tensor = cuda_tensor.detach().cpu().numpy().astype('float32').tobytes()
        query = "INSERT INTO pheromone_matrix_logs (episode_id, timestep, pheromone) VALUES ($1, $2, $3);"
        self.loop.run_until_complete(self.pool.execute(query, self.episode_id, self.timestep, cpu_tensor))

    def insert_priority_matrix(self, cuda_tensor):
        cpu_tensor = cuda_tensor.detach().cpu().numpy().astype('float32').tobytes()
        query = "INSERT INTO priority_matrix_logs (episode_id, timestep, priority) VALUES ($1, $2, $3);"
        self.loop.run_until_complete(self.pool.execute(query, self.episode_id, self.timestep, cpu_tensor))
    #
    # def insert_drone_position_in_cell(self, drone_id, x, y):
    #     query = "INSERT INTO drone_cell_matrix_logs (episode_id, drone_id, timestep, row, col) VALUES ($1,$2,$3,$4,$5);"
    #     self.loop.run_until_complete(self.pool.execute(query, self.episode_id, drone_id, self.timestep, x, y))

    # def insert_msg_sent(self, message_id, command, content, full_text):
    #     query = "INSERT INTO msg_sent_logs (episode_id, timestep, message_id, command, content, full_text) VALUES ($1,$2,$3,$4,$5,$6);"
    #     self.loop.run_until_complete(self.pool.execute(query, self.episode_id, self.timestep, message_id, command, content, full_text))
    #
    def insert_drone_position(self, drone_id, x, y):
        query = "INSERT INTO position_logs (episode_id, drone_id, timestep, position_x, position_y) VALUES ($1,$2,$3,$4,$5);"
        self.loop.run_until_complete(self.pool.execute(query, self.episode_id, drone_id, self.timestep, x, y))

#
# class DroneDBLogger(BaseDBLogger):
#     def __init__(self, drone_id):
#         super().__init__()
#         self.drone_id = drone_id
#         self.episode_id = self.loop.run_until_complete(self._get_latest_episode())
#
#     async def _get_latest_episode(self):
#         async with self.pool.acquire() as conn:
#             return await conn.fetchval("SELECT id FROM episodes ORDER BY id DESC LIMIT 1;")
#
#     def insert_drone_observed_state(self, roll, pitch, yaw, yaw_rate, x_global, v_x_global, y_global, v_y_global, altitude, v_x, v_y, dt):
#         query = "INSERT INTO drone_observed_state_logs (episode_id, drone_id, timestep, roll, pitch, yaw, yaw_rate, x_global, v_x_global, y_global, v_y_global, altitude, v_x, v_y, dt) VALUES ($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,$11,$12,$13,$14,$15);"
#         self.loop.run_until_complete(self.pool.execute(query, self.episode_id, self.drone_id, self.timestep, roll, pitch, yaw, yaw_rate, x_global, v_x_global, y_global, v_y_global, altitude, v_x, v_y, dt))
#
#     def insert_drone_desired_state(self, desired_vx, desired_vy, desired_yaw_rate, desired_altitude):
#         query = "INSERT INTO drone_desired_state_logs (episode_id, drone_id, timestep, desired_vx, desired_vy, desired_yaw_rate, desired_altitude) VALUES ($1,$2,$3,$4,$5,$6,$7);"
#         self.loop.run_until_complete(self.pool.execute(query, self.episode_id, self.drone_id, self.timestep, desired_vx, desired_vy, desired_yaw_rate, desired_altitude))
#
#     def insert_drone_motor_power(self, motor_speeds):
#         query = "INSERT INTO drone_motor_power_logs (episode_id, drone_id, timestep, motor1_speed, motor2_speed, motor3_speed, motor4_speed) VALUES ($1,$2,$3,$4,$5,$6,$7);"
#         self.loop.run_until_complete(self.pool.execute(query, self.episode_id, self.drone_id, self.timestep, *motor_speeds))
#
#     def insert_msg_received(self, message_id, content):
#         query = "INSERT INTO msg_received_logs (episode_id, drone_id, timestep, message_id, content) VALUES ($1,$2,$3,$4,$5);"
#         self.loop.run_until_complete(self.pool.execute(query, self.episode_id, self.drone_id, self.timestep, message_id, content))
#
#     def insert_velocity(self, velocity_x, velocity_y):
#         query = "INSERT INTO velocity_logs (episode_id, drone_id, timestep, velocity_x, velocity_y) VALUES ($1,$2,$3,$4,$5);"
#         self.loop.run_until_complete(self.pool.execute(query, self.episode_id, self.drone_id, self.timestep, velocity_x, velocity_y))
#
#     def insert_d_velocity(self, dv_x, dv_y):
#         query = "INSERT INTO d_velocity_logs (episode_id, drone_id, timestep, dv_x, dv_y) VALUES ($1,$2,$3,$4,$5);"
#         self.loop.run_until_complete(self.pool.execute(query, self.episode_id, self.drone_id, self.timestep, dv_x, dv_y))
#
#     def insert_fa(self, fa_x, fa_y):
#         query = "INSERT INTO fa_logs (episode_id, drone_id, timestep, fa_x, fa_y) VALUES ($1,$2,$3,$4,$5);"
#         self.loop.run_until_complete(self.pool.execute(query, self.episode_id, self.drone_id, self.timestep, fa_x, fa_y))
