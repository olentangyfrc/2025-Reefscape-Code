import math

from wpimath.controller import SimpleMotorFeedforwardMeters, ProfiledPIDController
import wpimath.trajectory

import wpimath.interpolation

from phoenix6 import hardware, configs, controls, BaseStatusSignal, signals
from wpilib import RobotBase, shuffleboard, DigitalInput, Timer

from magicbot import feedback

import utilities.constants as constants
import utilities.ozone_utility_functions as utility_functions

EXTRA_CLIMB_TIME = 0.15
CLIMB_VOLTAGE = -9


class Climber:
    def __init__(self):
        """Initializes the winched climber subsystem"""
        self.climb_motor = hardware.talon_fx.TalonFX(constants.CLIMB_MOTOR_ID, "*")

        config = configs.TalonFXConfiguration()
        config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        config.current_limits.stator_current_limit_enable = True
        config.current_limits.stator_current_limit = 90
        config.future_proof_configs = True

        self.climb_motor.configurator.apply(config)

        self.beam_break = DigitalInput(2)
        self.climbing = False

        self.climb_timer = Timer()
        self.climb_timer.stop()
        self.climb_timer.reset()

    def climb(self):
        """Tells the climber to begin the climbing process"""
        self.climbing = True
    
    @feedback(key="Climber motor current")
    def get_motor_current(self) -> float:
        """
        Returns the stator currrent being applied to the climber motor

        Returns:
            float: the current in amps being applied
        """
        return self.climb_motor.get_stator_current().value
    
    @feedback(key="Climber pos")
    def get_climber_position(self) -> float:
        """
        Returns the current position of the climber motor

        Returns:
            float: the current number of rotations the climber motor has done
        """
        return self.climb_motor.get_position().value

    @feedback(key="Climber beam break")
    def get_beam_break(self) -> bool:
        """
        Returns whether or not the beam break is currently broken

        Returns:
            bool: true if the beam break is broken else false
        """
        return not self.beam_break.get()

    def execute(self):
        """Called every periodic cycle in auton and teleop, runs all neccessary logic
            to operate the climber"""
        if not self.climbing:
            self.climb_motor.set_control(controls.VoltageOut(0))
        else:
            if not self.get_beam_break() and self.climb_timer.get() < 0.001:
                self.climb_motor.set_control(controls.VoltageOut(CLIMB_VOLTAGE))
            else:
                if not self.climb_timer.isRunning():
                    self.climb_timer.start()
                    self.climb_motor.set_control(controls.VoltageOut(CLIMB_VOLTAGE))
                elif self.climb_timer.get() < EXTRA_CLIMB_TIME:
                    self.climb_motor.set_control(controls.VoltageOut(CLIMB_VOLTAGE))
                elif self.climb_timer.get() >= EXTRA_CLIMB_TIME:
                    self.climb_motor.set_control(controls.VoltageOut(0))
                    self.climbing = False