from dataclasses import dataclass

@dataclass
class RobotConfig:
    BASE_SPEED: int = 255
    TURN_SPEED: int = 100
    CORRECTION_SPEED: int = 105
    MIN_SPEED: int = 60
    TURN_90_TIME_MS: int = 830
    TURN_90_TIME_MS_CCW: int = 1500
    LEFT_MOTOR_CORRECTION: float = 0.97
    RIGHT_MOTOR_CORRECTION: float = 1.0
    PID_KP: float = 25
    PID_KI: float = 0.05
    PID_KD: float = 10
    PID_MAX_INTEGRAL: float = 50
    PID_ALPHA: float = 0.3
    PID_CORRECTION_FACTOR: float = 1
    SMOOTH_LEFT_FACTOR: float = 0.85
    SMOOTH_RIGHT_FACTOR: float = 1.0

ROBOT_CONFIG = RobotConfig()
