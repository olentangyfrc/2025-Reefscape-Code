from wpimath.controller import ElevatorFeedforward, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile

from phoenix6 import hardware, configs, controls, signals
from wpilib import RobotBase, DutyCycleEncoder, Timer
from enum import Enum

from magicbot import feedback, tunable

from utilities.ozone_utility_functions import clamp
from utilities import constants

from wpimath.filter import SlewRateLimiter

RATIO = 43.55/1.5

MIN_HEIGHT = 0.0
MAX_HEIGHT = 1.545

MAX_VOLTAGE = 10.0
MAX_VELOCITY = 9999.0
MAX_ACCELERATION = 9999.0

class ElevatorState(Enum):
    """
    Possible heights for the elevator to go to each match
    """

    STOW = 0.0
    END_EFFECTOR_L1 = 0.18
    L1 = 0.2 #0.49
    L2 = 0.51
    ALGAE_LOW = 0.37
    ALGAE_HIGH = ALGAE_LOW + 0.415
    PROCESSOR = 0.20
    L4 = 1.545
    L3 = 0.915
    NET_INTERMEDIATE = 0.3
    NET = 1.545
    GROUND_INTAKE = 0.15
    L1_STACK = 0.28

    @staticmethod
    def from_int(height: int) -> float:
        """
        Returns a elevator state from a given integer value

        Params:
            height (int): the level which the elevator should go to
        
        Returns:
            float: the height which the elevator needs to go to so it can reach the given level
        """
        match height:
            case 0:
                return ElevatorState.STOW.value
            case 1:
                return ElevatorState.L1.value
            case 2:
                return ElevatorState.L2.value
            case 3:
                return ElevatorState.L3.value
            case 4:
                return ElevatorState.L4.value


class Elevator:

    p = tunable(17.0) #16.0
    i = tunable(0.8)
    d = tunable(0.0) #0.07

    kg = tunable(0.33)
    ks = tunable(0.11)

    l1_height = tunable(ElevatorState.L1.value)
    l1_stack_height = tunable(ElevatorState.L1_STACK.value)
    ground_intake_height = tunable(ElevatorState.GROUND_INTAKE.value)
    grab_height = tunable(ElevatorState.ALGAE_LOW.value)
    net_intermediate = tunable(ElevatorState.NET_INTERMEDIATE.value)

    manual = tunable(False)
    zero_elevator = tunable(False)

    max_vel = tunable(MAX_VELOCITY)
    max_acc = tunable(MAX_ACCELERATION)

    target = tunable(0.0)

    def __init__(self):
        """Initializes a continous elevator subsystem"""
        self.sim_height = 0.0
        self.rate_limiter = SlewRateLimiter(1.6, -1.6, 0) #TODO find a better mechanical accel limit

        config = configs.TalonFXConfiguration()
        config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        config.current_limits.stator_current_limit_enable = True
        config.current_limits.stator_current_limit = 50
        config.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        config.future_proof_configs = True

        self.left_motor = hardware.talon_fx.TalonFX(constants.LEFT_ELEVATOR_ID, "*")
        self.left_motor.configurator.apply(config)
        config.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        self.right_motor = hardware.talon_fx.TalonFX(constants.RIGHT_ELEVATOR_ID, "*")
        self.right_motor.configurator.apply(config)

        self.left_motor.set_position(0)
        self.right_motor.set_position(0)

        self.encoder = DutyCycleEncoder(0)

        self.last_a = MAX_ACCELERATION
        self.last_v = MAX_VELOCITY

        self.target_height = 0.0

        
        self.voltage_ramp_timer = Timer()
        self.voltage_ramp_timer.reset()

        self.zeroing_timer = Timer()
        self.zeroing_timer.reset()

        self.arm_angle = 95

    def setup(self) -> None:
        """
        This method is called automatically by magicbot after all components and 
        tunables are created
        """
        self.constraints = TrapezoidProfile.Constraints(self.max_vel, self.max_acc)
        self.pid = ProfiledPIDController(self.p, self.i, self.d, self.constraints)
        self.pid.setIZone(0.001)
        self.pid.reset(0, 0)
        self.pid.setTolerance(0.01)
        self.ff = ElevatorFeedforward(self.ks, self.kg, 0)

    def set(self, height: ElevatorState) -> None:
        """
        Sets a target reef position for the elevator.

        Params:
            height (int): Reef height to go to, with 0 being stowed, and 
            each reef height represented by a 1, 2, 3, or 4.
        """
        new_height = height.value
        match height:
            case ElevatorState.L1:
                new_height = self.l1_height
            case ElevatorState.L1_STACK:
                self.l1_stack_height
            case ElevatorState.GROUND_INTAKE:
                self.ground_intake_height
            case ElevatorState.ALGAE_LOW:
                self.grab_height
            case ElevatorState.ALGAE_HIGH:
                self.grab_height + 0.415
            case ElevatorState.NET_INTERMEDIATE:
                self.net_intermediate

        if self.pid.getGoal().position != new_height:
            self.voltage_ramp_timer.restart()
        self.pid.setGoal(clamp(new_height, MIN_HEIGHT, MAX_HEIGHT))
        self.target_height = new_height
        self.manual = False

    def go_to(self, height: float) -> None:
        """
        Sets the target height in the elevator carriage in meters from stowed position
        """
        if self.pid.getGoal().position != height:
            self.voltage_ramp_timer.restart()
        
        self.pid.setGoal(clamp(height, MIN_HEIGHT, MAX_HEIGHT))
        self.target_height = height

    def set_arm_position(self, angle: float) -> None:
        """
        Sets the arm position to soft stop any crash scenarios. This will not actually 
        move the arm
        
        Params:
            angle(float): the angle to set the arm to in code
        """
        self.arm_angle = angle

    @feedback(key="Elevator rots")
    def get_rotations(self) -> float:
        """
        Gets the number of rotations the elevator motors have undergone with zero being the stowed position
        """
        return (self.left_motor.get_position().value + self.right_motor.get_position().value)/2
    
    @feedback(key="Elevator left pos")
    def get_left_position(self) -> float:
        """
        Returns the number of rotations the left elevator motor has done

        Returns:
            float: the number of left elevator motor rotations
        """
        return self.left_motor.get_position().value
    
    @feedback(key="Elevator right pos")
    def get_right_position(self) -> float:
        """
        Returns the number of rotations the left elevator motor has done

        Returns:
            float: the number of left elevator motor rotations
        """
        return self.right_motor.get_position().value
    
    @feedback(key="Elevator encoder pos")
    def get_encoder_pos(self) -> float:
        """
        Returns the position of the absolute encoder on the elevator

        Returns:
            float: the position of the encoder scaled by the full range of input
        """
        return self.encoder.get()

    @feedback(key="Elevator height")
    def get_height(self) -> float:
        """
        Gets the height of the carriage from its stowed position
        """
        if RobotBase.isReal():
            return self.get_rotations() / RATIO
        return self.sim_height
    
    @feedback(key="Elevator target height")
    def get_target_height(self) -> float:
        """
        Returns the target height of the elevator
        """
        return self.target_height
    
    @feedback(key="Elevator goal pos")
    def get_goal_position(self) -> float:
        """
        Returns the target position of the elevator profiled pid controller

        Returns:
            float: the target position for the profiled pid's goal
        """
        return self.pid.getGoal().position
    
    @feedback(key="Elevator goal vel")
    def get_goal_velocity(self) -> float:
        """
        Returns the target velocity of the elevator profiled pid controller

        Returns:
            float: the target velocity for the profiled pid's goal
        """
        return self.pid.getGoal().velocity
    
    @feedback(key="Elevator intermediate pos")
    def get_intermediate_position(self) -> float:
        """
        Returns the intermediate position of the elevator profiled pid controller

        Returns:
            float: the position for each setpoint of the profiled pid
        """
        return self.pid.getSetpoint().position
    
    @feedback(key="Elevator intermediate vel")
    def get_intermediate_velocity(self) -> float:
        """
        Returns the intermediate velocity of the elevator profiled pid controller

        Returns:
            float: the velocity for each setpoint of the profiled pid
        """
        return self.pid.getSetpoint().velocity   

    @feedback(key="Elevator at target")
    def at_target(self) -> bool:
        """
        Returns whether or not the elevator is at its target position

        Returns:
            bool: if the elevator pid is within tolerance of its goal
        """
        return self.pid.atGoal()
    
    @feedback(key="Elevator error")
    def get_error(self) -> float:
        """
        Returns the error of the elevator from its target
        
        Returns:
            float: the error calculated by subtracting the height of the elevator from the profiled pid controller's target 
        """
        return self.get_target_height() - self.get_height()
    
    def execute(self):
        """
        Called every periodic cycle in auton and teleop, 
        runs all neccessary logic to operate the elevator
        """
        if not RobotBase.isReal():
            intermediate_height = self.sim_height
            if abs(self.get_error()) > self.pid.getPositionTolerance():
                intermediate_height += self.get_error()/5
                self.sim_height = self.rate_limiter.calculate(intermediate_height)
                return                 

        if self.zero_elevator:
            self.left_motor.set_position(0)
            self.right_motor.set_position(0)
            self.zero_elevator = False
        
        if self.manual:
            if self.last_a != self.max_acc or self.last_v != self.max_vel:
                self.constraints = TrapezoidProfile.Constraints(self.max_vel, self.max_acc)
                self.pid.setConstraints(self.constraints)

            self.ff.setKs(self.ks)
            self.ff.setKg(self.kg)

            self.pid.setP(self.p)
            self.pid.setI(self.i)
            self.pid.setD(self.d)

            self.pid.setGoal(clamp(self.target, MIN_HEIGHT, MAX_HEIGHT))

        self.target_voltage = clamp(self.pid.calculate(self.get_height()), -MAX_VOLTAGE, MAX_VOLTAGE) + \
            self.ff.calculate(self.pid.getGoal().position-self.get_height())
        self.target_voltage = clamp(self.target_voltage, -self.voltage_ramp_timer.get()*60, self.voltage_ramp_timer.get()*60)
        
        self.left_motor.set_control(controls.VoltageOut(self.target_voltage))
        self.right_motor.set_control(controls.VoltageOut(self.target_voltage))

        if self.target_voltage < 0 and abs(self.left_motor.get_velocity().value) < 0.02:
            self.zeroing_timer.start()
            if self.zeroing_timer.get() > 0.5 and self.arm_angle > -30:
                self.left_motor.set_position(0)
                self.right_motor.set_position(0)
        else:
            self.zeroing_timer.reset()
            self.zeroing_timer.stop()