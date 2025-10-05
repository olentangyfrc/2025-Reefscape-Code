from phoenix6 import hardware, signals, configs, controls
from enum import Enum
from wpilib import Timer, RobotBase

from utilities import constants
from magicbot import feedback, tunable


class EndEffectorStates(Enum):
    IDLE = 0
    INTAKING = 1
    REVERSING = 2
    HOLDING = 3
    DISPENSING = 4
    EJECT = 5


class EndEffector:
    CURRENT_LIMIT = 80
    max_speed = tunable(4.0)
    min_speed = tunable(3.0)
    slow_speed = tunable(3.5)
    PIVOT_RATIO = 3
    RELEASE_SPEED = 4.5
    L1 = 1.8
    L2 = 3.9
    L4 = 4

    l1_speed = tunable(L1)
    l1_stack_speed = tunable(1.6)

    def __init__(self) -> None:
        """Initializes the end effector subsystem on the bot"""
        self.shot_timer = Timer()
        self.shot_timer.stop()
        self.shot_timer.reset()
        self.current_height = 4  # Scoring height to differentiate end effector speeds

        self.sim_holding = False
        self.sim_timer = Timer()

        manipulator_motor_config = configs.TalonFXConfiguration()
        manipulator_motor_config.motor_output.neutral_mode = (
            signals.NeutralModeValue.COAST
        )
        manipulator_motor_config.current_limits.stator_current_limit_enable = True
        manipulator_motor_config.current_limits.stator_current_limit = (
            EndEffector.CURRENT_LIMIT
        )

        self.manipulator_motor = hardware.TalonFX(constants.END_EFFECTOR_ID, "*")
        self.manipulator_motor.clear_sticky_faults()
        self.manipulator_motor.configurator.apply(manipulator_motor_config)

        self.backup_timer = Timer()

        # configs for the CANranges
        proximity_params = configs.ProximityParamsConfigs()
        fov_params = configs.FovParamsConfigs()
        tof_params = configs.ToFParamsConfigs()

        proximity_params.proximity_threshold = 0.05
        proximity_params.min_signal_strength_for_valid_measurement = 3000

        fov_params.fov_range_x = 13.5
        fov_params.fov_range_y = 13.5

        tof_params.update_mode = signals.UpdateModeValue.SHORT_RANGE100_HZ

        canrange_configs = (
            configs.CANrangeConfiguration()
            .with_fov_params(fov_params)
            .with_proximity_params(proximity_params)
            .with_to_f_params(tof_params)
        )

        self.exit_canrange = hardware.CANrange(constants.CORAL_EXIT_ID, "*")
        self.entry_canrange = hardware.CANrange(constants.CORAL_ENTRY_ID, "*")

        self.exit_canrange.clear_sticky_faults()
        self.entry_canrange.clear_sticky_faults()

        self.exit_canrange.configurator.apply(canrange_configs)
        self.entry_canrange.configurator.apply(canrange_configs)

        self.holding_coral = False
        self.state = (
            EndEffectorStates.HOLDING if self.ready() else EndEffectorStates.IDLE
        )

    def ready(self) -> bool:
        """
        Returns whether or not the end effector is ready to score

        Returns:
            bool: whether either the entry or exit CANranges are tripped
        """
        if RobotBase.isReal():
            return self.get_entry() or self.get_exit()
        return self.sim_holding

    def set_height(self, height: int | float) -> None:
        """Sets the height that the robot is about to score on so the end effector knows how fast to release"""
        self.current_height = height

    def stop(self) -> None:
        """Sets the end effector to its idle state"""
        self.state = EndEffectorStates.IDLE

    def intake(self) -> None:
        """Sets the end effector to intake from the coral station"""
        self.state = EndEffectorStates.INTAKING

    def dispense(self) -> None:
        """Sets the end effector to dispense onto the reef"""
        self.state = EndEffectorStates.DISPENSING

    def eject(self) -> None:
        """Sets the end effector to eject any coral lodged in it"""
        self.state = EndEffectorStates.EJECT

    def is_done(self) -> bool:
        """
        Returns whether or not the end effector has had enough time to set the
        elevator down after shooting

        Returns:
            bool: whether the last shot was more than 0.2 seconds ago
        """
        if RobotBase.isReal():
            return self.shot_timer.get() > 0.2
        return not self.sim_holding

    @feedback(key="End effector state")
    def get_state_string(self) -> str:
        """
        Returns the name of the current state

        Returns:
            str: the current enum state's name
        """
        return self.state.name

    @feedback(key="End effector holding")
    def is_holding_coral(self) -> bool:
        """
        Returns if whether or not the end effector is currently holding a coral

        Returns:
            bool: if the end effector is holding a coral
        """
        if RobotBase.isReal():
            return self.holding_coral
        return self.sim_holding

    def get_current(self) -> float:
        """
        Returns the current stator current being applied to the manipulator motor

        Returns:
            float: the stator current in amps applied to the maipulator motor
        """
        return self.manipulator_motor.get_stator_current().value

    @feedback(key="End effector entry")
    def get_entry(self) -> bool:
        """
        Returns if the entry CANrange on the end effector is tripped or not

        Returns:
            bool: whether the entry CANrange has detected an object
        """
        if not RobotBase.isReal():
            return self.sim_holding
        return self.entry_canrange.get_is_detected().value

    @feedback(key="End effector exit")
    def get_exit(self) -> bool:
        """
        Returns if the exit CANrange on the end effector is tripped or not

        Returns:
            bool: whether the exit CANrange has detected an object
        """
        if not RobotBase.isReal():
            return self.sim_holding
        return self.exit_canrange.get_is_detected().value

    def execute(self) -> None:
        """Called every periodic cycle in auton, runs all logic to operate the end effector"""
        if not RobotBase.isReal():
            if self.state == EndEffectorStates.INTAKING:
                if not self.sim_timer.isRunning():
                    self.sim_timer.start()
                if self.sim_timer.get() > 1.2:
                    self.sim_holding = True
                    self.sim_timer.stop()
                    self.sim_timer.reset()
            elif self.state == EndEffectorStates.DISPENSING:
                if not self.sim_timer.isRunning():
                    self.sim_timer.start()
                if self.sim_timer.get() > 0.2:
                    self.sim_holding = False
                    self.sim_timer.stop()
                    self.sim_timer.reset()
            return

        match self.state:
            case EndEffectorStates.IDLE:
                self.holding_coral = False

                self.manipulator_motor.set_control(controls.VoltageOut(0))

            case EndEffectorStates.INTAKING:
                self.holding_coral = False
                if self.get_entry():
                    if self.get_exit():
                        self.manipulator_motor.set_control(
                            controls.VoltageOut(self.min_speed)
                        )
                    else:
                        self.manipulator_motor.set_control(
                            controls.VoltageOut(self.slow_speed)
                        )
                elif not self.get_entry() and self.get_exit():
                    self.holding_coral = True
                    self.state = EndEffectorStates.REVERSING
                    self.manipulator_motor.set_control(controls.VoltageOut(0))
                elif not (self.get_entry() and self.get_exit()):
                    self.manipulator_motor.set_control(
                        controls.VoltageOut(self.max_speed)
                    )
                    self.backup_timer.restart()
            case EndEffectorStates.REVERSING:
                self.manipulator_motor.set_control(controls.VoltageOut(-1.5))
                if self.get_entry():
                    self.state = EndEffectorStates.HOLDING
                    self.manipulator_motor.set_control(controls.VoltageOut(0))
            case EndEffectorStates.HOLDING:
                self.shot_timer.stop()
                self.shot_timer.reset()
                self.manipulator_motor.set_control(controls.VoltageOut(0))
                self.holding_coral = True

            case EndEffectorStates.DISPENSING:
                self.holding_coral = False

                if not self.get_entry() and not self.get_exit():
                    self.state = EndEffectorStates.IDLE
                elif self.get_exit():
                    self.manipulator_motor.set_control(
                        controls.VoltageOut(EndEffector.RELEASE_SPEED)
                    )
                    speed = EndEffector.RELEASE_SPEED
                    match self.current_height:
                        case 1:
                            speed = self.l1_speed
                        case 1.5:
                            speed = self.l1_stack_speed
                        case 2 | 3:
                            speed = EndEffector.L2
                        case 4:
                            speed = EndEffector.L4
                    self.manipulator_motor.set_control(controls.VoltageOut(speed))

                if not self.shot_timer.isRunning():
                    self.shot_timer.restart()

            case EndEffectorStates.EJECT:
                self.holding_coral = False

                if not self.get_entry() and not self.get_exit():
                    self.state = EndEffectorStates.IDLE
                elif self.get_exit():
                    self.manipulator_motor.set_control(
                        controls.VoltageOut(self.max_speed)
                    )
