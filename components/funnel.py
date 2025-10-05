import wpilib
from magicbot import feedback


class Funnel:
    LEFT_PWM_PORT = 0
    RIGHT_PWM_PORT = 1
    LEFT_DOWN = 0.69
    LEFT_UP = 0.25
    RIGHT_UP = 0.732
    RIGHT_DOWN = 0.285

    def __init__(self) -> None:
        """
        Initializes the funnel subsystem
        """
        self.left_servo = wpilib.Servo(Funnel.LEFT_PWM_PORT)
        self.right_servo = wpilib.Servo(Funnel.RIGHT_PWM_PORT)
        self.is_climbing = False

    def get_left_position(self) -> float:
        """
        Returns the angle of the left servo

        Returns:
            float: a measure of the left servo position in the range 0-1
        """
        return self.left_servo.get()

    def get_right_position(self) -> float:
        """
        Returns the position of the right servo

        Returns:
            float: a measure of the right servo position in the range 0-1
        """
        return self.right_servo.get()

    def climb(self) -> None:
        """
        Signals if the funnel should begin to raise so the bot can climb
        """
        self.is_climbing = True

    def execute(self) -> None:
        """
        Called every periodic cycle, runs all logic to operate the funnel
        """
        if not self.is_climbing:
            self.left_servo.set(Funnel.LEFT_DOWN)
            self.right_servo.set(Funnel.RIGHT_DOWN)
        else:
            self.left_servo.set(Funnel.LEFT_UP)
            self.right_servo.set(Funnel.RIGHT_UP)
