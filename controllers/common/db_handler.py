import numpy as np
from sqlalchemy.orm import Session

from controllers.common.db_models import (
    SessionLocal, Episode, DroneObservedStateLog, VelocityLog, dVelocityLog,
    FaLog, MsgReceivedLog, MsgSentLog, PriorityMatrixLog, PheromoneMatrixLog,
    DroneCellPositionLog, DronePositionLog, DroneDesiredStateLog, DroneMotorPowerLog
)


class DatabaseHandler:
    def __init__(self):
        """Initialize a database session"""
        self.db = SessionLocal()
        self.current_episode_id = None
        print("Database session started.")

    def close(self):
        """Closes the session at the end of the episode."""
        if self.db:
            self.db.close()
            print("Database session closed.")

    def create_episode(self, drone_count):
        """Creates a new episode and stores the ID."""
        new_episode = Episode(drone_count=drone_count)
        self.db.add(new_episode)
        self.db.commit()
        self.db.refresh(new_episode)
        self.current_episode_id = new_episode.id
        print(f"New episode created: ID {self.current_episode_id}")
        return self.current_episode_id

    def get_latest_episode_id(self):
        """Retrieves the last added episode ID for other robots."""
        last_episode = self.db.query(Episode).order_by(Episode.id.desc()).first()
        self.current_episode_id = last_episode.id if last_episode else None
        return self.current_episode_id

    # ─────────────────────────────────────────────
    #  ✅ Generic Function for Logging
    # ─────────────────────────────────────────────
    def add_log_entry(self, model, **kwargs):
        """
        Generic function to add log entries.
        :param model: ORM model class (e.g., VelocityLog, DroneObservedStateLog)
        :param kwargs: Dictionary of column values (excluding episode_id & timestep)
        """
        if self.current_episode_id is None:
            raise ValueError("No active episode. Call `create_episode()` first.")
        kwargs.pop("type", None)  # Removes "episode_id" if it exists
        # Convert NumPy data types to standard Python types
        for key, value in kwargs.items():
            if isinstance(value, np.floating):  # Handles np.float32, np.float64, etc.
                kwargs[key] = float(value)
            elif isinstance(value, np.integer):  # Handles np.int32, np.int64, etc.
                kwargs[key] = int(value)
        # Create a new log entry with episode_id and timestep
        log_entry = model(episode_id=self.current_episode_id, **kwargs)

        # Add to database and commit
        self.db.add(log_entry)
        self.db.commit()
        self.db.refresh(log_entry)  # Ensure ID is populated

        return log_entry.id  # Return the ID of the added entry

    # ─────────────────────────────────────────────
    #  🚀 Drone Logs using `add_log_entry`
    # ─────────────────────────────────────────────
    def log_drone_observed_state(self, **kwargs):
        """Logs the observed state of a drone."""
        return self.add_log_entry(DroneObservedStateLog, **kwargs)

    def log_drone_desired_state(self, **kwargs):
        """Logs the observed state of a drone."""
        return self.add_log_entry(DroneDesiredStateLog, **kwargs)

    def log_drone_motor_power(self, **kwargs):
        """Logs the observed state of a drone."""
        return self.add_log_entry(DroneMotorPowerLog, **kwargs)

    def log_velocity(self, **kwargs):
        """Logs the velocity of a drone."""
        return self.add_log_entry(VelocityLog, **kwargs)

    def log_d_velocity(self, **kwargs):
        """Logs the velocity change (delta velocity) of a drone."""
        return self.add_log_entry(dVelocityLog, **kwargs)

    def log_fa(self, **kwargs):
        """Logs the force applied to a drone."""
        return self.add_log_entry(FaLog, **kwargs)

    def log_msg_received(self, **kwargs):
        """Logs a message received by a drone."""
        return self.add_log_entry(MsgReceivedLog, **kwargs)

    # ─────────────────────────────────────────────
    #  🚀 Supervisor Logs using `add_log_entry`
    # ─────────────────────────────────────────────
    def log_msg_sent(self, **kwargs):
        """Logs a message sent by the supervisor."""
        return self.add_log_entry(MsgSentLog, **kwargs)

    def log_priority_matrix(self, **kwargs):
        """Logs priority matrix values."""
        return self.add_log_entry(PriorityMatrixLog, **kwargs)

    def log_pheromone_matrix(self, **kwargs):
        """Logs pheromone matrix values."""
        return self.add_log_entry(PheromoneMatrixLog, **kwargs)

    def log_drone_cell_position(self, **kwargs):
        """Logs the cell position of a drone in a matrix."""
        return self.add_log_entry(DroneCellPositionLog, **kwargs)

    def log_drone_pos(self, **kwargs):
        """Logs the state (position) of a drone."""
        return self.add_log_entry(DronePositionLog, **kwargs)
