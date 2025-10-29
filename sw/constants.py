class RobotConfig:
    def __init__(self):
        ## Motor speeds
        self.BASE_SPEED = 255
        self.TURN_SPEED = 100
        self.CORRECTION_SPEED = 105
        self.MIN_SPEED = 60

        ## Timing
        self.TURN_90_TIME_MS = 830
        self.TURN_90_TIME_MS_CCW = 1500

        ## Motor corrections
        self.LEFT_MOTOR_CORRECTION = 0.97
        self.RIGHT_MOTOR_CORRECTION = 1.0

        ## PID parameters
        self.PID_KP = 90
        self.PID_KI = 0.05
        self.PID_KD = 35
        self.PID_MAX_INTEGRAL = 50
        self.PID_ALPHA = 0.3
        self.PID_CORRECTION_FACTOR = 1.5
        ## Sensor thresholds
        self.ULTRASONIC_BOX_THRESHOLD_CM = 20
        self.QR_SCAN_ATTEMPTS = 9999

        ## Navigation
        self.INTERSECTION_CLEAR_ITERATIONS = 15

        ## Line recovery
        self.LINE_LOSS_THRESHOLD = 20  ## Iterations before triggering recovery
        self.RECOVERY_BACKUP_MS = 200
        self.RECOVERY_SEARCH_SPEED = 100
        self.RECOVERY_SEARCH_STEPS = 15

ROBOT_CONFIG = RobotConfig()

