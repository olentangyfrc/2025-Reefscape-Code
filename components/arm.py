# None: An assumption is made that the valid angle range is [0,3pi/4]. This (and several other settings)
# obviously need to be checked in the actual bot.

import math
from enum import Enum
import time

# import utilities.ozone_utility_functions as utils
from phoenix6 import configs, signals, hardware
from phoenix6.controls import VoltageOut
import wpilib
from wpilib import RobotBase

import wpimath
from wpimath.controller import ProfiledPIDController, ArmFeedforward
from wpimath.trajectory import TrapezoidProfile
from wpimath.filter import LinearFilter, Debouncer

from wpilib.shuffleboard import Shuffleboard
from magicbot import feedback, tunable

from utilities import ozone_utility_functions as utils
from utilities import constants

MAX_VELOCITY = 9999.0
MAX_ACCELERATION = 9999.0

class Arm():

    MINIMUM_ANGLE = -32
    MAXIMUM_ANGLE = 90

    INTAKE_ANGLE_TOLERANCE = math.radians(3)

    SCORE_DURATION = 0.7

    GEAR_RATIO = 24/40 # Gear ratio for motor to arm

    p = tunable(2.6) # 2.2
    i = tunable(0.0) # 0.05
    d = tunable(0.0)

    kg = tunable(0.21) # 0.29
    ks = tunable(0.0) # 0.03

    ground_intake_speed = tunable(-0.8)
    ground_intake_angle = tunable(-35)

    max_vel = tunable(MAX_VELOCITY)
    max_acc = tunable(MAX_ACCELERATION)

    manual = tunable(False)

    manual_angle = tunable(0.0)
    manual_speed = tunable(0.0)
    grab_angle = tunable(25.0)

    def __init__(self) -> None:
        """Initializes the arm subsystem"""
        # Intake Roller
        self.intake_roller_configs = configs.TalonFXConfiguration()
        self.intake_roller_configs.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        self.intake_roller_configs.current_limits.supply_current_limit = 20
        self.intake_roller_configs.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.intake_roller_motor = hardware.TalonFX(
            constants.ARM_ROLLERS_ID, "*")
        self.intake_roller_motor.configurator.apply(self.intake_roller_configs)

        # Angle Motor
        self.intake_angle_configs = configs.TalonFXConfiguration()
        self.intake_angle_configs.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        self.intake_angle_configs.current_limits.supply_current_limit = 50
        self.intake_angle_configs.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.intake_angle_motor = hardware.TalonFX(
            constants.ARM_PIVOT_ID, "*")
        self.intake_angle_motor.configurator.apply(self.intake_angle_configs)

        # Encoder
        self.angle_absolute_encoder = wpilib.DutyCycleEncoder(
            1, 1, -0.214)
        self.angle_absolute_encoder.setAssumedFrequency(975.6)
        self.angle_absolute_encoder.setInverted(True)


        self.linear_filter = LinearFilter.movingAverage(10)
        self.debouncer = Debouncer(0.2, Debouncer.DebounceType.kBoth)

        self.state = ArmStates.IDLE

        self.last_v = MAX_ACCELERATION
        self.last_a = MAX_ACCELERATION
        
        self.target_voltage = 0
        self.target_angle = 0
        self.target_speed = 0

        self.holding_algae = False

        self.voltage_ramp_timer = wpilib.Timer()
        self.voltage_ramp_timer.reset()
        self.voltage_ramp_timer.stop()
        self.last_angle = 0

        self.timer = wpilib.Timer()

        self.forced = False

        self.can_range = hardware.CANrange(33, '*')

        proximity = configs.ProximityParamsConfigs()
        fov = configs.FovParamsConfigs()
        tof = configs.ToFParamsConfigs()

        proximity.proximity_threshold = 0.30
        proximity.min_signal_strength_for_valid_measurement = 2500

        fov.fov_range_x = 13.5
        fov.fov_range_y = 13.5

        tof.update_mode = signals.UpdateModeValue.SHORT_RANGE100_HZ

        canConfigs = configs.CANrangeConfiguration().with_fov_params(fov).with_proximity_params(proximity).with_to_f_params(tof)
        self.can_range.configurator.apply(canConfigs)

        self.sim_holding = False
    
    def setup(self) -> None:
        """
        This method is called automatically by magicbot after all components and 
        tunables are created
        """
        self.constraints = TrapezoidProfile.Constraints(self.max_vel, self.max_acc)
        self.intake_angle_pid = ProfiledPIDController(
            self.p,
            self.i,
            self.d,
            self.constraints
        )
        self.intake_angle_pid.setTolerance(self.INTAKE_ANGLE_TOLERANCE)
        self.intake_angle_pid.setIZone(0.01)
        self.intake_angle_pid.reset(0, 0)
        self.intake_angle_pid.setTolerance(1)

        self.intake_angle_ff = ArmFeedforward(
            self.ks,
            self.kg,
            0.0
        )
    
    @feedback
    def get_detected(self) -> bool:
        return self.can_range.get_is_detected().value

    @feedback(key="Arm State")
    def get_state_string(self) -> str:
        """
        Returns the state that the arm is in

        Returns:
            str: a string representing the arm enum name
        """
        return self.state.name

    @feedback(key="Arm Current")
    def get_current(self) -> float:
        """
        Returns the raw current applied to the intake roller motors
        
        Returns:
            float: a measure of the stator current being applied to the motor in amps
        """
        return self.intake_roller_motor.get_stator_current().value

    @feedback(key="Arm Current Filtered")
    def get_filtered_current(self) -> float:
        """
        Returns the current applied to the intake roller motors after being passed through a moving average
        filter
        
        Returns:
            float: an average of the last 10 current measurements in amps
        """
        return self.linear_filter.calculate(self.intake_roller_motor.get_stator_current().value)
    
    @feedback(key="Arm absolute pos")
    def get_abs_encoder(self) -> float:
        """
        Returns position of absolute encoder
        
        Returns:
            float: the position of the absolute encoder in rotations
        """
        return self.angle_absolute_encoder.get()

    @feedback(key="Arm pivot pos")
    def get_pivot_position(self) -> float:
        """ 
        Returns position of arm in radians
        
        Returns:
            float: the arm position in radians clamped to the range -math.pi to math.pi
        """
        if RobotBase.isReal():
            return utils.inputModulus(self.angle_absolute_encoder.get() * math.tau, -math.pi, math.pi)
        return self.target_angle
    
    @feedback(key="Arm pivot pos deg")
    def get_pivot_position_deg(self) -> float:
        """Returns position of the arm in degrees"""
        return math.degrees(self.get_pivot_position())

    @feedback(key="Arm pivot vel")
    def get_pivot_velocity(self) -> float:
        """ Returns velocity of arm in radians per second """
        return self.intake_angle_motor.get_velocity().value * self.GEAR_RATIO * math.tau
    
    @feedback(key="At target")
    def is_at_target(self) -> bool: 
        """ Returns whether if the arm is at its target position """
        return self.intake_angle_pid.atSetpoint()
    
    @feedback(key="holding")
    def is_holding(self) -> bool:
        """Returns whether the arm is holding an algae"""
        return self.holding_algae
    
    @feedback(key="Arm target voltage")
    def get_target_voltage(self) -> float:
        """Returns the target voltage given to the intake angle motor
           
           Returns:
            float: the sum of the pid and feedforward outputs for the angle motor
        """
        return self.target_voltage

    @feedback(key="Arm target angle")
    def get_target_angle(self) -> float:
        """Returns the target angle of the arm
           
           Returns:
            float: target angle of the arm in degrees
        """
        return math.degrees(self.target_angle)
    
    @feedback(key="Arm target speed")
    def get_target_speed(self) -> float:
        """Returns the target speed of the arm
           
           Returns:
            float: target speed of the arm between -1 and 1
        """
        return self.target_speed
    
    @feedback(key="Arm goal pos")
    def get_goal_position(self) -> float:
        """
        Returns the target position of the elevator profiled pid controller at its
        goal

        Returns:
            float: the target position for the profiled pid's goal
        """
        return self.intake_angle_pid.getGoal().position
    
    @feedback(key="Arm goal vel")
    def get_goal_velocity(self) -> float:
        """
        Returns the velocity of the elevator profiled pid controller at its goal

        Returns:
            float: the target velocity for the profiled pid's goal
        """
        return self.intake_angle_pid.getGoal().velocity

    @feedback(key="Arm intermediate pos")
    def get_intermediate_position(self) -> float:
        """
        Returns the intermediate position of the elevator profiled pid controller

        Returns:
            float: the position for each setpoint of the profiled pid
        """
        return self.intake_angle_pid.getSetpoint().position
    
    @feedback(key="Arm intermediate vel")
    def get_intermediate_velocity(self) -> float:
        """
        Returns the intermediate velocity of the elevator profiled pid controller

        Returns:
            float: the velocity for each setpoint of the profiled pid
        """
        return self.intake_angle_pid.getSetpoint().velocity
    
    def sim_grab(self) -> None:
        self.sim_holding = True

    def sim_release(self) -> None:
        self.sim_holding = False

    def check_current_limit(self, limit: float) -> bool:
        """
        Returns if the current applied to the intake roller motors is greater than a given limit
        Params:
            limit (float): The limit that current must be greater than
        Returns:
            bool: whether or not the current has been greater than the limit for 0.15 seconds
        """
        return self.debouncer.calculate(self.get_filtered_current() > limit)
    
    def prepare_process(self) -> None:
        """Puts the arm out to processor position"""
        if self.holding_algae:
            self.state = ArmStates.PREPARE_PROCESS

    def remove_algae(self) -> None:
        """Removes algae from the reef""" 
        if not self.holding_algae: 
            self.state = ArmStates.REMOVE_ALGAE

    def net(self) -> None: 
        """ Scores algae into barge """
        self.state = ArmStates.NET

    def set_idle(self) -> None: 
        """ Resets arm state """
        if not self.get_detected(): 
            self.state = ArmStates.IDLE
            self.holding_algae = False
        elif RobotBase.isReal():
            self.state = ArmStates.HOLD_ALGAE
    
    def processor(self) -> None: 
        """Scores algae in the processor"""
        self.state = ArmStates.PROCESSOR

    def ground_intake(self) -> None:
        """Grab an algae off the ground"""
        self.state = ArmStates.GROUND_INTAKE_ALGAE

    def execute(self):
        """Called every periodic cycle in autonomous and teleop, runs all logic to operate the arm"""
        match self.state:
            case ArmStates.IDLE:
                self.target_speed = 0
                self.target_angle = math.radians(90)
            case ArmStates.HOLD_ALGAE:
                if self.get_detected():
                    self.target_speed = 0
                    self.target_angle = math.radians(58)
                    self.holding_algae = True
                else: 
                    self.holding_algae = False
                    self.state = ArmStates.IDLE
            case ArmStates.REMOVE_ALGAE: 
                self.target_angle = math.radians(self.grab_angle)
                self.target_speed = -1.0
                if self.get_detected():
                    if not self.timer.isRunning():
                        self.timer.restart()
                    elif self.timer.get() > 0.2:
                        self.state = ArmStates.HOLD_ALGAE
                        self.holding_algae = True
                        self.timer.reset()
                        self.timer.stop()
                else:
                    self.timer.restart()
            case ArmStates.GROUND_INTAKE_ALGAE:
                self.target_angle = math.radians(self.ground_intake_angle)
                self.target_speed = self.ground_intake_speed
                if self.get_detected():
                    if not self.timer.isRunning():
                        self.timer.restart()
                    elif self.timer.get() > 0.2:
                        self.state = ArmStates.HOLD_ALGAE
                        self.holding_algae = True
                        self.timer.reset()
                        self.timer.stop()
                else:
                    self.timer.restart()
            case ArmStates.PROCESSOR:
                if not RobotBase.isReal():
                    self.sim_release()
                    self.state = ArmStates.IDLE
                self.target_angle = math.radians(-5)
                if self.is_at_target() and math.degrees(self.get_pivot_position()) < 45:
                    self.target_speed = 0.3 #TODO: Find this and good angle
                    if not self.timer.isRunning():
                        self.timer.restart()
                    if self.timer.get() > 1 and not self.get_detected():
                        self.state = ArmStates.IDLE
                        self.holding_algae = False
                        self.timer.stop()
                        self.timer.reset()
            case ArmStates.NET: 
                if not RobotBase.isReal():
                    self.sim_release()
                self.target_angle = math.radians(58)
                self.target_speed = 0.8
                
                if not self.get_detected() or not RobotBase.isReal():
                    if not self.timer.isRunning():
                        self.timer.restart()
                    if self.timer.get() > 0.5:
                        self.state = ArmStates.IDLE
                        self.holding_algae = False
                        self.timer.stop()
                        self.timer.reset()
                        
            case ArmStates.EJECT_ALGAE:
                self.target_angle = math.radians(0)
                if self.is_at_target():
                    self.target_speed = 0.5
                    if self.timer.get() > self.SCORE_DURATION:
                        self.state = ArmStates.IDLE
                        self.holding_algae = False
                        self.timer.reset()
                        self.timer.stop()

            case ArmStates.PREPARE_REMOVE_ALGAE:
                self.target_angle = math.radians(45)
                self.target_speed = 0
            
            case ArmStates.PREPARE_PROCESS:
                self.target_angle = math.radians(-5)
                self.target_speed = 0
                if not self.get_detected():
                    self.state = ArmStates.HOLD_ALGAE

        if self.manual:
            if self.last_a != self.max_acc or self.last_v != self.max_vel:
                self.constraints = TrapezoidProfile.Constraints(self.max_vel, self.max_acc)
                self.intake_angle_pid.setConstraints(self.constraints)

            self.intake_angle_ff.setKg(self.kg)
            self.intake_angle_ff.setKs(self.ks)

            self.intake_angle_pid.setP(self.p)
            self.intake_angle_pid.setI(self.i)
            self.intake_angle_pid.setD(self.d)

            self.target_angle = math.radians(utils.clamp(self.manual_angle, self.MINIMUM_ANGLE, self.MAXIMUM_ANGLE))
            self.target_speed = utils.clamp(self.manual_speed, -1, 1)
    
        self.intake_angle_pid.setGoal(self.target_angle)
        pid_output = self.intake_angle_pid.calculate(self.get_pivot_position())

        if self.target_angle != self.last_angle:
            self.voltage_ramp_timer.restart()
            self.last_angle = self.target_angle

        pid_output = utils.clamp(pid_output, -self.voltage_ramp_timer.get()*7, self.voltage_ramp_timer.get()*7)
        ff_output = self.intake_angle_ff.calculate(self.get_pivot_position(), self.get_pivot_velocity())
        self.target_voltage = pid_output + ff_output

        self.intake_angle_motor.set_control(VoltageOut(self.target_voltage))
        self.intake_roller_motor.set(self.target_speed)

class ArmStates(Enum):
    """Possible states the arm can be in each match"""
    IDLE = 0 # The arm is stowed and doing nothing
    REMOVE_ALGAE = 1 # The arm is removing algae from the reef
    GROUND_INTAKE_ALGAE = 2 # The arm is removing algae from the ground
    PROCESSOR = 3 # The arm is scoring algae in the processor
    EJECT_ALGAE = 4 # The arm is ejecting lodged algae
    HOLD_ALGAE = 5 # The arm is holding algae
    PREPARE_REMOVE_ALGAE = 6 # The arm is preparing to remove algae
    NET = 7 # Barge
    PREPARE_PROCESS = 8