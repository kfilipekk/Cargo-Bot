class RobotConfig:
    def __init__(self):
        self.BASE_SPEED = 255
        self.TURN_SPEED = 100
        self.CORRECTION_SPEED = 105
        self.MIN_SPEED = 60
        self.TURN_90_TIME_MS = 830
        self.TURN_90_TIME_MS_CCW = 1500
        self.LEFT_MOTOR_CORRECTION = 0.97
        self.RIGHT_MOTOR_CORRECTION = 1.0
        self.PID_KP = 90
        self.PID_KI = 0.05
        self.PID_KD = 35
        self.PID_MAX_INTEGRAL = 50
        self.PID_ALPHA = 0.3
        self.PID_CORRECTION_FACTOR = 1.5
        self.SMOOTH_LEFT_FACTOR = 0.85
        self.SMOOTH_RIGHT_FACTOR = 1.0

ROBOT_CONFIG = RobotConfig()
