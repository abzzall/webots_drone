from sqlalchemy import create_engine, Column, Integer, Float, String, ForeignKey, DateTime, func
from sqlalchemy.ext.declarative import declarative_base, declared_attr
from sqlalchemy.orm import sessionmaker, relationship, polymorphic_union, with_polymorphic
import os
from dotenv import load_dotenv

# Load environment variables from .env
load_dotenv()
DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    raise ValueError("DATABASE_URL is not set in .env file")

# Create a database engine
engine = create_engine(DATABASE_URL, pool_size=10, max_overflow=20)

# Create a session factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for ORM models
Base = declarative_base()

# ─────────────────────────────────────────────
#  Abstract Log Table (Parent Class for All Logs)
# ─────────────────────────────────────────────
class BaseLog(Base):
    __abstract__ = True
    # __tablename__ = "base_logs"

    id = Column(Integer, primary_key=True, autoincrement=True)
    episode_id = Column(Integer, ForeignKey("episodes.id"), nullable=False)
    timestamp = Column(DateTime, default=func.now())  # Log entry time (Auto)
    timestep = Column(Integer, nullable=False)

    @declared_attr
    def episode(cls):
        if cls.__abstract__:
            return None  # Avoid defining it in the abstract class
        return relationship("Episode", back_populates="logs")

    # @declared_attr
    # def __mapper_args__(cls):
    #     """Dynamically set polymorphic_identity to the child class's table name."""
    #     return {
    #         "polymorphic_identity": cls.__tablename__,
    #         "with_polymorphic": "*",
    #     }


# ─────────────────────────────────────────────
#  Drone Log Tables (Subclasses of BaseLog)
# ─────────────────────────────────────────────
class DroneBaseLog(BaseLog):
    __abstract__ = True

    @declared_attr
    def drone_id(cls):
        return Column(Integer, nullable=False)


class DroneObservedStateLog(DroneBaseLog):
    __tablename__ = "drone_observed_state_logs"
    roll = Column(Float, nullable=False)
    pitch = Column(Float, nullable=False)
    yaw = Column(Float, nullable=False)
    yaw_rate = Column(Float, nullable=False)
    x_global = Column(Float, nullable=False)
    v_x_global = Column(Float, nullable=False)
    y_global = Column(Float, nullable=False)
    v_y_global = Column(Float, nullable=False)
    altitude = Column(Float, nullable=False)
    v_x = Column(Float, nullable=False)
    v_y = Column(Float, nullable=False)
    dt = Column(Float, nullable=False)


class DroneDesiredStateLog(DroneBaseLog):
    __tablename__ = "drone_desired_state_logs"
    desired_vx = Column(Float, nullable=False)
    desired_vy = Column(Float, nullable=False)
    desired_yaw_rate = Column(Float, nullable=False)
    desired_altitude = Column(Float, nullable=False)


class DroneMotorPowerLog(DroneBaseLog):
    __tablename__ = "drone_motor_power_logs"
    motor1_speed = Column(Float, nullable=False)
    motor2_speed = Column(Float, nullable=False)
    motor3_speed = Column(Float, nullable=False)
    motor4_speed = Column(Float, nullable=False)


# ─────────────────────────────────────────────
#  Supervisor Logs
# ─────────────────────────────────────────────
class MsgSentLog(BaseLog):
    __tablename__ = "msg_sent_logs"
    command = Column(String, nullable=False)
    content = Column(String, nullable=True)
    full_text = Column(String, nullable=False)


# ─────────────────────────────────────────────
#  Matrix-Based Log Tables (Subclasses of BaseLog)
# ─────────────────────────────────────────────
class MatrixBaseLog(BaseLog):
    __abstract__ = True

    @declared_attr
    def row(cls):
        return Column(Integer, nullable=False)

    @declared_attr
    def col(cls):
        return Column(Integer, nullable=False)


class PriorityMatrixLog(MatrixBaseLog):
    __tablename__ = "priority_matrix_logs"
    priority = Column(Float, nullable=False)


class PheromoneMatrixLog(MatrixBaseLog):
    __tablename__ = "pheromone_matrix_logs"
    pheromone = Column(Float, nullable=False)


class DroneCellPositionLog(MatrixBaseLog):
    __tablename__ = "drone_cell_matrix_logs"
    drone_id = Column(Integer, nullable=False)

# ─────────────────────────────────────────────
#  Velocity Log Table
# ─────────────────────────────────────────────
class VelocityLog(DroneBaseLog):
    __tablename__ = "velocity_logs"
    velocity_x = Column(Float, nullable=False)
    velocity_y = Column(Float, nullable=False)

# ─────────────────────────────────────────────
#  dVelocity Log Table
# ─────────────────────────────────────────────
class dVelocityLog(DroneBaseLog):
    __tablename__ = "d_velocity_logs"
    dv_x = Column(Float, nullable=False)
    dv_y = Column(Float, nullable=False)

class FaLog(DroneBaseLog):
    __tablename__ = "fa_logs"
    fa_x = Column(Float, nullable=False)
    fa_y = Column(Float, nullable=False)



class MsgReceivedLog(DroneBaseLog):
    __tablename__ = "msg_received_logs"
    content= Column(String, nullable=False)





class DronePositionLog(DroneBaseLog):
    __tablename__ = "position_logs"
    position_x = Column(Float, nullable=False)
    position_y = Column(Float, nullable=False)
    # position_z = Column(Float, nullable=False)


# ─────────────────────────────────────────────
#  Episode Table (Tracks Each Simulation Run)
# ─────────────────────────────────────────────
class Episode(Base):
    __tablename__ = "episodes"
    id = Column(Integer, primary_key=True, autoincrement=True)

    start_time = Column(DateTime, default=func.now())  # Start timestamp (Auto)
    drone_count = Column(Integer, nullable=True)  # Number of drones in episode
    end_time = Column(DateTime, nullable=True)  # Null until episode ends
    # logs = relationship("BaseLog", back_populates="episode", lazy="dynamic")
    # logs = relationship(
    #     with_polymorphic(BaseLog, "*"),  # Allow querying all subclasses
    #     back_populates="episode",
    #     lazy="dynamic"
    # )



# ─────────────────────────────────────────────
#  Create All Tables in PostgreSQL
# ─────────────────────────────────────────────




Base.metadata.create_all(engine)
