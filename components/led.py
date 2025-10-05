from wpilib import Timer, DriverStation
from phoenix5 import led
import utilities.constants as constants
import enum

from magicbot import feedback


class LEDStates(enum.Enum):
    """Possible states that the CANdles can be in"""

    IDLE = 1  # The bot is on red or blue alliance
    READY_TO_CLIMB = 2  # The bot is ready to climb
    NODE_SWITCH = 3  # The bot has recieved a new node from the pygame dashboard


class LEDs:
    ORANGE_FADE = led.SingleFadeAnimation(255, 165, 0, 0, 1, 66, 0)
    GREEN_FADE = led.SingleFadeAnimation(0, 255, 0, 0, 1, 66, 0)
    RED = led.SingleFadeAnimation(255, 0, 0, 0, 0, 66, 0)
    BLUE = led.SingleFadeAnimation(0, 0, 255, 0, 0, 66, 0)
    FIRE = led.FireAnimation(1, 0.4, 52, 0.4, 0.4)

    def __init__(self):
        """Initializes the CANdle on the bot"""
        self.leds = led.CANdle(constants.LED_CAN, "*")
        self.led_configuration = led.CANdleConfiguration()
        self.leds.configAllSettings(self.led_configuration)

        self.state = LEDStates.IDLE
        self.timer = Timer()

        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.leds.animate(self.BLUE)
        else:
            self.leds.animate(self.RED)

    def get_state_string(self) -> str:
        """
        Returns the current state of the led

        Returns:
            str: a string value representing the current enum state of the leds
        """
        return self.state.name

    def execute(self):
        """Called every periodic cycle in teleop or auton, runs all logic to operate the
        LEDs"""
        match self.state:
            case LEDStates.IDLE:
                if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                    self.leds.animate(self.BLUE)
                else:
                    self.leds.animate(self.RED)
            case LEDStates.READY_TO_CLIMB:
                self.leds.animate(self.ORANGE_FADE)
            case LEDStates.NODE_SWITCH:
                self.leds.animate(self.GREEN_FADE)
                if not self.timer.isRunning():
                    self.timer.restart()
                if self.timer.get() > 0.5:
                    self.timer.stop()
                    self.timer.reset()
                    self.state = LEDStates.IDLE
