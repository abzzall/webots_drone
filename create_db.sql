-- ────────────────────────────────────────────────────
-- episodes (tracks each simulation run)
-- ────────────────────────────────────────────────────
CREATE TABLE episodes (
    id SERIAL PRIMARY KEY,
    start_time TIMESTAMP DEFAULT NOW(),
    drone_count INTEGER,
    end_time TIMESTAMP
);

-- ────────────────────────────────────────────────────
-- drone_observed_state_logs
-- ────────────────────────────────────────────────────
CREATE TABLE drone_observed_state_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    drone_id INTEGER NOT NULL,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    roll FLOAT8 NOT NULL,
    pitch FLOAT8 NOT NULL,
    yaw FLOAT8 NOT NULL,
    yaw_rate FLOAT8 NOT NULL,
    x_global FLOAT8 NOT NULL,
    v_x_global FLOAT8 NOT NULL,
    y_global FLOAT8 NOT NULL,
    v_y_global FLOAT8 NOT NULL,
    altitude FLOAT8 NOT NULL,
    v_x FLOAT8 NOT NULL,
    v_y FLOAT8 NOT NULL,
    dt FLOAT8 NOT NULL
);

-- ────────────────────────────────────────────────────
-- drone_desired_state_logs
-- ────────────────────────────────────────────────────
CREATE TABLE drone_desired_state_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    drone_id INTEGER NOT NULL,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    desired_vx FLOAT8 NOT NULL,
    desired_vy FLOAT8 NOT NULL,
    desired_yaw_rate FLOAT8 NOT NULL,
    desired_altitude FLOAT8 NOT NULL
);

-- ────────────────────────────────────────────────────
-- drone_motor_power_logs
-- ────────────────────────────────────────────────────
CREATE TABLE drone_motor_power_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    drone_id INTEGER NOT NULL,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    motor1_speed FLOAT8 NOT NULL,
    motor2_speed FLOAT8 NOT NULL,
    motor3_speed FLOAT8 NOT NULL,
    motor4_speed FLOAT8 NOT NULL
);

-- ────────────────────────────────────────────────────
-- msg_sent_logs (message_id added)
-- ────────────────────────────────────────────────────
CREATE TABLE msg_sent_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    message_id INTEGER NOT NULL,
    command TEXT NOT NULL,
    content TEXT,
    full_text TEXT NOT NULL
);

-- ────────────────────────────────────────────────────
-- msg_received_logs (message_id added)
-- ────────────────────────────────────────────────────
CREATE TABLE msg_received_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    drone_id INTEGER NOT NULL,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    message_id INTEGER NOT NULL,
    content TEXT NOT NULL
);

-- ────────────────────────────────────────────────────
-- priority_matrix_logs
-- ────────────────────────────────────────────────────
CREATE TABLE priority_matrix_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),

    priority bytea NOT NULL
);

-- ────────────────────────────────────────────────────
-- pheromone_matrix_logs
-- ────────────────────────────────────────────────────
CREATE TABLE pheromone_matrix_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    pheromone bytea NOT NULL
);

-- ────────────────────────────────────────────────────
-- drone_cell_matrix_logs
-- ────────────────────────────────────────────────────
CREATE TABLE drone_cell_matrix_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    drone_id INTEGER NOT NULL,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    row INTEGER NOT NULL,
    col INTEGER NOT NULL
);

-- ────────────────────────────────────────────────────
-- velocity_logs
-- ────────────────────────────────────────────────────
CREATE TABLE velocity_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    drone_id INTEGER NOT NULL,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    velocity_x FLOAT8 NOT NULL,
    velocity_y FLOAT8 NOT NULL
);

-- ────────────────────────────────────────────────────
-- d_velocity_logs
-- ────────────────────────────────────────────────────
CREATE TABLE d_velocity_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    drone_id INTEGER NOT NULL,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    dv_x FLOAT8 NOT NULL,
    dv_y FLOAT8 NOT NULL
);

-- ────────────────────────────────────────────────────
-- fa_logs
-- ────────────────────────────────────────────────────
CREATE TABLE fa_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    drone_id INTEGER NOT NULL,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    fa_x FLOAT8 NOT NULL,
    fa_y FLOAT8 NOT NULL
);

-- ────────────────────────────────────────────────────
-- position_logs
-- ────────────────────────────────────────────────────
CREATE TABLE position_logs (
    id SERIAL PRIMARY KEY,
    episode_id INTEGER NOT NULL REFERENCES episodes(id) ON DELETE CASCADE,
    drone_id INTEGER NOT NULL,
    timestep INTEGER NOT NULL,
    timestamp TIMESTAMP DEFAULT NOW(),
    position_x FLOAT8 NOT NULL,
    position_y FLOAT8 NOT NULL
);