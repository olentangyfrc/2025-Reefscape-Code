from choreo.trajectory import SwerveSample
from phoenix6 import hardware
import wpimath.controller
import wpimath.estimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import wpimath.kinematics
import wpilib
from wpilib import Field2d, DriverStation, Timer
from wpilib.shuffleboard import Shuffleboard
from choreo.trajectory import SwerveTrajectory
import math
from enum import Enum

from magicbot import feedback, tunable

import utilities.constants as constants
import utilities.ozone_utility_functions as utils
import utilities.reefscape_functions as reefscape_functions
import components.chassis.swervemodule as swervemodule

MAX_LINE_UP_SPEED = 2


class Drivetrain:
    instance = None

    manual = tunable(False)

    manual_voltage = tunable(0.0)
    manual_angle = tunable(0.0)

    rotation_p = tunable(4.0)
    rotation_i = tunable(0.0)
    rotation_d = tunable(0.0)

    line_up_p = tunable(3.0)
    line_up_i = tunable(0.0)
    line_up_d = tunable(0.05)

    kA = tunable(0.30)
    kAlpha = tunable(0.26)

    rot_stdev = tunable(1)

    reset_to_net = tunable(False)

    def __init__(self) -> None:
        """
        Initializes the drivetrain and its modules
        """
        self.state = DrivetrainState.NONE

        self.target_pose = Pose2d(0, 0, Rotation2d(0))

        if not wpilib.RobotBase.isReal():
            self.x_controller = wpimath.controller.PIDController(16, 0.0, 0.5)
            self.y_controller = wpimath.controller.PIDController(16, 0.0, 0.5)
        else:
            self.x_controller = wpimath.controller.PIDController(2.8, 0.0, 0.035)
            self.y_controller = wpimath.controller.PIDController(2.8, 0.0, 0.035)

        self.fast_snap_pid = wpimath.controller.PIDController(10, 1.5, 0)
        self.fast_snap_pid.setIZone(math.tau / 360 * 5)
        self.fast_snap_pid.enableContinuousInput(0, math.tau)

        wheel_base = constants.WHEEL_BASE
        track_width = constants.TRACK_WIDTH

        self.front_left_location = Translation2d(wheel_base / 2, track_width / 2)
        self.front_right_location = Translation2d(wheel_base / 2, -track_width / 2)
        self.back_left_location = Translation2d(-wheel_base / 2, track_width / 2)
        self.back_right_location = Translation2d(-wheel_base / 2, -track_width / 2)

        self.is_vision_enabled = True

        self.front_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FL,
            turning_motor_id=constants.STEER_CAN_FL,
            turning_encoder_id=constants.TURN_ENCODER_ID_FL,
            offset=constants.FL_OFFSET,
            name="Front Left",
        )

        self.front_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FR,
            turning_motor_id=constants.STEER_CAN_FR,
            turning_encoder_id=constants.TURN_ENCODER_ID_FR,
            offset=constants.FR_OFFSET,
            name="Front Right",
        )

        self.back_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BL,
            turning_motor_id=constants.STEER_CAN_BL,
            turning_encoder_id=constants.TURN_ENCODER_ID_BL,
            offset=constants.BL_OFFSET,
            name="Back Left",
        )

        self.back_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BR,
            turning_motor_id=constants.STEER_CAN_BR,
            turning_encoder_id=constants.TURN_ENCODER_ID_BR,
            offset=constants.BR_OFFSET,
            name="Back Right",
        )

        self.gyro = hardware.Pigeon2(constants.PIGEON_CAN, "*")
        self.climbing = False
        if not wpilib.RobotBase.isReal():
            self.sim_gyro = Rotation2d()

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.front_left_location,
            self.front_right_location,
            self.back_left_location,
            self.back_right_location,
        )

        self.zero_gyro(180)

        self.pose_estimator = wpimath.estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_gyro_yaw_value(),
            (
                self.front_left.get_swerve_position(),
                self.front_right.get_swerve_position(),
                self.back_left.get_swerve_position(),
                self.back_right.get_swerve_position(),
            ),
            Pose2d(0, 0, Rotation2d(0)),
        )

        self.pose_estimator.setVisionMeasurementStdDevs((0.1, 0.1, 0.1))

        self.max_speed = constants.MAX_LINEAR_SPEED

        self.snap_angle_pid = wpimath.controller.PIDController(4, 0, 0.033)
        self.snap_angle_pid.enableContinuousInput(0, math.tau)
        self.snap_angle_pid.setTolerance(math.radians(0.5))
        self.last_angular_input = 0
        self.last_translational_input = 0

        self.px = 0.0
        self.py = 0.0

        self.vx = 0.0
        self.vy = 0.0

        self.ax = 0.0
        self.ay = 0.0

        self.timer = wpilib.Timer()

        self.field_relative = True
        self.reset_ffs = False

        self.signal = None
        self.snap_angle = None

        Drivetrain.instance = self

        self.matchTimer = Timer()

        self.drivetrain_field = Field2d()
        self.drivetrain_field.setRobotPose(Pose2d())
        self.traj_end_pose = Pose2d()

        self.intermediate_pose = Pose2d()

        Shuffleboard.getTab("Teleoperated").add(
            "Drivetrain field", self.drivetrain_field
        )
        Shuffleboard.getTab("Teleoperated").add("Filler", 0.0)

        self.tighten_tolerances = False

    def setup(self) -> None:
        """
        This method is automatically called by magicbot after components and tunables are created
        """
        self.line_up_controller = wpimath.controller.PIDController(
            self.line_up_p, self.line_up_i, self.line_up_d
        )

        self.heading_controller = wpimath.controller.PIDController(
            self.rotation_p, self.rotation_i, self.rotation_d
        )
        self.heading_controller.enableContinuousInput(0, math.tau)
        self.heading_controller.setTolerance(0.2)

    def is_manual(self) -> bool:
        """
        Returns whether or not the bot is currently being manually controlled

        Returns:
            bool: if the bot is in a manual state
        """
        return self.state in [
            DrivetrainState.ACTIVE_SNAP_TELEOP,
            DrivetrainState.LOCKED_TELEOP,
            DrivetrainState.PASSIVE_SNAP_TELEOP,
            DrivetrainState.POINT_AT_REEF,
            DrivetrainState.RAW_INPUT_TELEOP,
        ]

    def is_lining_up(self) -> bool:
        """
        Returns whether or not the bot is currently lining up or not

        Returns:
            bool: whether the bots tate is currently line_up_teleop or line_up_auton
        """
        return self.state in [
            DrivetrainState.LINE_UP_TELEOP,
            DrivetrainState.LINE_UP_AUTON,
        ]

    def disable_vision(self) -> None:
        """
        Blocks vision measurements from updating the drivetrain odometry
        """
        self.is_vision_enabled = False

    def enable_vision(self) -> None:
        """
        Re-enables vision, allowing it to update odometry with vision measurements
        """
        self.is_vision_enabled = True

    def get_signal(self):
        """
        Returns the current drive signal (See DriveSignal class)
        """
        return self.signal

    def set_signal(self, signal):
        """
        Sets a target signal for the drivetrain

        Params:
            signal (DriveSignal): The target signal
        """
        if isinstance(signal, DriveSignal):
            self.signal = signal

    def update_pose_estimator(self) -> None:
        """
        Updates both pose estimators with drivetrain movements
        """
        self.pose_estimator.updateWithTime(
            self.timer.getFPGATimestamp(),
            self.get_gyro_yaw_value(),
            self.get_module_postitions(),
        )

    def drive(
        self, speeds: wpimath.kinematics.ChassisSpeeds, field_relative: bool = True
    ) -> None:
        """
        Sets the desired state for each module based on the givven ChassisSpeeds.

        Params:
            speeds (ChassisSpeeds): Target velocities for the bot.
            field_relative (bool): Whether the bot should be field relative or not
        """

        if field_relative:
            speeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, self.get_gyro_yaw_value()
            )

        target_states = self.kinematics.desaturateWheelSpeeds(
            self.kinematics.toSwerveModuleStates(speeds), self.max_speed
        )

        self.front_left.set_desired_state(target_states[0])
        self.front_right.set_desired_state(target_states[1])
        self.back_left.set_desired_state(target_states[2])
        self.back_right.set_desired_state(target_states[3])

        if not wpilib.RobotBase.isReal():
            self.sim_gyro = Rotation2d(self.sim_gyro.radians() + speeds.omega * 0.02)

    def enable_active_snap(self, angle: Rotation2d | None) -> None:
        """
        Set a target angle for the drivetrain to go to and enters the active snap state.
        Note if drivetrain is set to manual, this will set angle to value given in network
        tables

        Params:
            angle (Rotation2d): Target angle.
        """
        self.snap_angle = angle

        self.state = DrivetrainState.ACTIVE_SNAP_TELEOP

    def add_vision_measurement(self, pose: Pose2d, time_stamp: float, xy_std) -> None:
        """
        Adds a vision measurement to the pose estimator for fine tuned lineups.

        Params:
            pose (Pose2d): Position and rotation of the given estimate.
            time_stamp (float): Time of measurement in seconds since the start of the match.
        """
        if (
            self.is_vision_enabled
            and abs(pose.translation().norm()) > 1e-6
            and self.matchTimer.get() > 0.3
        ):
            modifier = (self.rot_stdev + abs(self.get_yaw_rate())) / self.rot_stdev
            self.pose_estimator.setVisionMeasurementStdDevs(
                (xy_std * modifier, xy_std * modifier, 0.2)
            )
            self.pose_estimator.addVisionMeasurement(pose, time_stamp)

    def manual_drive(self) -> None:
        """
        Sets the drivetrain to the raw joystick drive mode, resetting all snap and lock timers.
        """
        self.state = DrivetrainState.RAW_INPUT_TELEOP
        self.last_angular_input = self.timer.getFPGATimestamp()
        self.last_translational_input = self.timer.getFPGATimestamp()

    def set_forward_signal(self, speed) -> None:
        """
        Sets a forward DriveSignal for auton

        Params:
            speed: speed in meters per second to drive forward at
        """
        self.set_signal(
            DriveSignal(wpimath.kinematics.ChassisSpeeds(-speed, 0, 0), False)
        )

    def set_right_signal(self, speed) -> None:
        """
        Sets a leftward DriveSignal for auton

        Params:
            speed: speed in meters per second to drive rightwards at
        """
        self.set_signal(DriveSignal(wpimath.kinematics.ChassisSpeeds(0, -speed, 0)))

    def apply_voltage(self, drive_voltage: float, turn_voltage: float) -> None:
        """
        Applies a steady voltage to both the steer and drive motors for testing purposes

        Params:
            drive volatge(float): Voltage to apply drive motors
            turn voltage (float): Voltage to apply steer motors
        """
        self.state = DrivetrainState.NONE
        self.back_left.set_module_voltage(drive_voltage, turn_voltage)
        self.back_right.set_module_voltage(drive_voltage, turn_voltage)
        self.front_left.set_module_voltage(drive_voltage, turn_voltage)
        self.front_right.set_module_voltage(drive_voltage, turn_voltage)

    def feed_voltage(self) -> None:
        """Feeds the drivetrain a certain amount of voltage by resetting feedforwards
        and giving a forward signal. Only used for tuning feedforwards don't use during
        comp"""
        if not self.reset_ffs:
            self.front_left.reset_feedforward()
            self.front_right.reset_feedforward()
            self.back_left.reset_feedforward()
            self.back_right.reset_feedforward()
            self.reset_ffs = True

        self.state = DrivetrainState.FEED_VOLTAGE

    def get_module_postitions(self) -> tuple:
        """
        Gets all current module positions as a tuple

        Returns:
            tuple[SwerveModulePosition]: All module positions
        """
        return (
            self.front_left.get_swerve_position(),
            self.front_right.get_swerve_position(),
            self.back_left.get_swerve_position(),
            self.back_right.get_swerve_position(),
        )

    def auton_drive(self) -> None:
        """
        Sets drivetrain state to autonomous path following
        """
        self.state = DrivetrainState.FOLLOW_PATH_AUTON

    def line_up_auton(self, pose: Pose2d) -> None:
        """
        Sets drivetrain state to line up to a Pose in auton

        Params:
            pose (Pose2d): Pose to line up to
        """
        self.target_pose = pose
        self.state = DrivetrainState.LINE_UP_AUTON

    def line_up(self, loc: Pose2d) -> None:
        """
        Sets a target lineup point for the drivetrain and begins moving there, taking control from the driver.

        Params:
            loc (Pose2d): Target position and rotation for the bot.
        """
        self.target_pose = loc
        self.state = DrivetrainState.LINE_UP_TELEOP

    def zero_gyro(self, degrees: float = 0) -> None:
        """
        Zeroes the gyro to the value given in degrees.

        Params:
            degrees (float): Value to reset to.
        """
        self.gyro.set_yaw(degrees)
        if not wpilib.RobotBase.isReal():
            self.sim_gyro = Rotation2d.fromDegrees(
                degrees + 180 * DriverStation.isTeleop()
            )

    def get_state(self):
        """
        Gets the enum value of the current drivetrain state.

        Returns:
            DrivetrainState: Current state.
        """
        return self.state

    def reset_trajectory(self) -> None:
        """
        Resets the field's displayed trajectories
        """
        self.drivetrain_field.getObject("traj").setPoses([])

    def follow_trajectory(self, sample: SwerveSample, traj: SwerveTrajectory) -> None:
        """
        Follows a Choreo trajectory using a PID-based holonomic path follower.

        Params:
            sample (SwerveSample): Class containing a target position and rotation for the drivetrain.
            traj (SwerveTrajecotry): The current trajectory the drivetrain is folllowing
        """
        self.state = DrivetrainState.FOLLOW_PATH_AUTON

        if not sample:
            print("No valid trajectory sample provided.")
            self.state = DrivetrainState.LOCKED_TELEOP
            return

        self.intermediate_pose = Pose2d(sample.x, sample.y, sample.heading)

        # To minimize the amount of trajectories we generate
        if traj.get_final_pose(False) != self.traj_end_pose:
            self.drivetrain_field.getObject("traj").setPoses(traj.get_poses())
            self.traj_end_pose = traj.get_final_pose(False)

        # Get the current pose of the robot
        pose = self.get_pose()

        # Generate the next speeds for the robot
        self.vx = sample.vx
        self.vy = sample.vy
        self.ax = sample.ax
        self.ay = sample.ay

        self.px = self.x_controller.calculate(pose.X(), sample.x)
        self.py = self.y_controller.calculate(pose.Y(), sample.y)

        speeds = wpimath.kinematics.ChassisSpeeds(
            self.ax * self.kA + self.vx + self.px,
            self.ay * self.kA + self.vy + self.py,
            sample.omega
            + self.kAlpha * sample.alpha
            + self.heading_controller.calculate(
                self.get_gyro_yaw_value().radians(), sample.heading
            ),
        )

        # Apply the generated speed
        self.set_signal(DriveSignal(speeds, True))

    def reset_odometry(self, loc: Pose2d) -> None:
        """
        Resets odometry to the Pose given

        Params:
            loc: Pose2d: Pose to reset to
        """
        self.zero_gyro(loc.rotation().degrees())
        print(self.pose_estimator.getEstimatedPosition())
        self.pose_estimator.resetPosition(
            self.get_gyro_yaw_value(), self.get_module_postitions(), loc
        )
        self.pose_estimator.resetPose(loc)
        print(self.pose_estimator.getEstimatedPosition())
        print("\n" * 10)

    def get_state_string(self) -> str:
        """
        Gets the current state of the drivetrain as a string.

        Returns:
            str: Drivetrain state.
        """
        return self.state.name

    def get_intermediate_pose(self) -> Pose2d:
        """
        Get the target pose for the current sample in auton

        Returns:
            Pose2d: Target pose (should be pretty close to bot location)
        """
        return self.intermediate_pose

    def get_pose(self) -> Pose2d:
        """
        Gets a pose that is noisier but more accurate for fine drivetrain control.

        Returns:
            Pose2d: Bot's current pose
        """
        return Pose2d(
            self.pose_estimator.getEstimatedPosition().translation(),
            self.pose_estimator.getEstimatedPosition().rotation(),
        )


    def get_target_pose(self) -> Pose2d:
        """
        Gets the bot's current target pose it is trying to line up to, if no pose is given
        it returns the bot's current pose

        Returns:
            Pose2d: Target pose given to the bot
        """
        if not self.target_pose:
            return self.get_pose()
        return self.target_pose

    def get_gyro_yaw_value(self) -> Rotation2d:
        """
        Gets the bot's current rotation from the gyro as a Rotation2d

        Returns:
            Rotation2d: The bot's rotation
        """
        if not wpilib.RobotBase.isReal():
            return self.sim_gyro
        return Rotation2d(math.radians(self.gyro.get_yaw().value))

    def get_gyro_yaw_degrees(self) -> float:
        """
        Get the bot's rotation from the gyro in degrees

        Returns
            float: the bot's rotation in degrees
        """
        return self.get_gyro_yaw_value().degrees()

    def get_yaw_rate(self) -> float:
        """
        Gets the rate at which the bot is turning from the gyro

        Returns:
            float: Yaw rate in degrees per second
        """
        return -self.gyro.get_angular_velocity_z_device().value


    def is_field_relative(self) -> bool:
        """
        Gets whether the bot is currently field relative

        Returns:
            bool: Whether the bot is field relative
        """
        return self.field_relative

    def get_velocity(self) -> Pose2d:
        """
        Returns the bot's current inputted velocity

        Returns:
            Pose2d: A pose containing the xy and rotational velocity in units /s
        """
        if self.signal:
            return Pose2d(
                self.signal.get_speed().vx,
                self.signal.get_speed().vy,
                self.signal.get_speed().omega,
            )
        return Pose2d()

    def is_aligned(self) -> bool:
        """
        Returns whether the robot is within a small tolerance of the target pose. If no target pose is given
        automatically returns false. If the robot is active snapping returns whether in tolerance of snap angle.

        Returns:
            bool: If the bot is at its target.
        """

        if self.state == DrivetrainState.ALIGNED:
            return True

        if not self.target_pose or self.target_pose == Pose2d():
            return False

        if self.tighten_tolerances:
            return utils.within_pose_tolerance(
                self.get_pose(), self.target_pose, 0.035, 0.2
            )

        if self.state == DrivetrainState.ACTIVE_SNAP_TELEOP:
            return utils.within_rotation_tolerance(
                self.snap_angle, self.get_pose().rotation(), 1
            )

        if DriverStation.isAutonomous():
            return utils.within_pose_tolerance(
                self.target_pose, self.get_pose(), 0.035, 0.75
            )
        return utils.within_pose_tolerance(
            self.target_pose, self.get_pose(), 0.025, 0.75
        )

    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the drivetrain based off current state.
        """

        self.matchTimer.start()
        self.drivetrain_field.setRobotPose(
            Pose2d(self.get_pose().X(), self.get_pose().Y(), self.get_pose().rotation())
        )

        # Updates odometry with current drivetrain mov[\]ements
        self.update_pose_estimator()

        if not DriverStation.isFMSAttached():
            # Resets to test netting in pit
            if self.reset_to_net:
                self.reset_odometry(Pose2d(7, 6, Rotation2d.fromDegrees(0)))
                self.reset_to_net = False

        match self.state:
            case DrivetrainState.NONE:
                pass  # print("Warning: drivetrain state is set to \"NONE\"")

            case DrivetrainState.LOCKED_TELEOP:
                self.front_left.set_desired_state(
                    wpimath.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(225))
                )
                self.front_right.set_desired_state(
                    wpimath.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(135))
                )
                self.back_left.set_desired_state(
                    wpimath.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(315))
                )
                self.back_right.set_desired_state(
                    wpimath.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(225))
                )
                if self.last_translational_input + 0.04 > self.timer.getFPGATimestamp():
                    self.state = DrivetrainState.PASSIVE_SNAP_TELEOP
                    self.snap_angle = self.get_gyro_yaw_value()
                if self.last_angular_input + 0.04 > self.timer.getFPGATimestamp():
                    self.state = DrivetrainState.RAW_INPUT_TELEOP

            case DrivetrainState.RAW_INPUT_TELEOP:
                if self.signal:
                    self.drive(
                        self.signal.get_speed(), self.signal.get_field_relative()
                    )
                    if self.last_angular_input + 0.1 < self.timer.getFPGATimestamp():
                        # self.state = DrivetrainState.PASSIVE_SNAP_TELEOP
                        # self.snap_angle = self.get_gyro_yaw_value()
                        if (
                            self.last_translational_input + 1.5
                            < self.timer.getFPGATimestamp()
                        ):
                            pass  # self.state = DrivetrainState.LOCKED_TELEOP

            case DrivetrainState.PASSIVE_SNAP_TELEOP:
                if self.snap_angle == None:
                    self.snap_angle = self.get_gyro_yaw_value()
                if self.last_angular_input + 0.04 > self.timer.getFPGATimestamp():
                    self.state = DrivetrainState.RAW_INPUT_TELEOP
                elif (
                    self.last_translational_input + 1.5 < self.timer.getFPGATimestamp()
                ):
                    pass  # self.state = DrivetrainState.LOCKED_TELEOP
                self.snap_angle_pid.setSetpoint(self.snap_angle.radians())
                self.signal.get_speed().omega = self.snap_angle_pid.calculate(
                    self.get_pose().rotation().radians()
                )
                if self.signal.get_speed().vx == 0 and self.signal.get_speed().vy == 0:
                    self.signal.get_speed().omega = 0
                self.drive(self.signal.get_speed(), self.signal.get_field_relative())
                if self.last_translational_input + 1.5 < self.timer.getFPGATimestamp():
                    self.state = DrivetrainState.LOCKED_TELEOP
                self.snap_angle_pid.setSetpoint(self.snap_angle.radians())
                self.signal.get_speed().omega = self.snap_angle_pid.calculate(
                    self.get_pose().rotation().radians()
                )
                self.drive(self.signal.get_speed(), self.signal.get_field_relative())

            case DrivetrainState.ACTIVE_SNAP_TELEOP:
                if self.manual:
                    self.heading_controller.setP(self.rotation_p)
                    self.heading_controller.setI(self.rotation_i)
                    self.heading_controller.setD(self.rotation_d)

                    self.snap_angle = Rotation2d.fromDegrees(self.manual_angle)

                self.snap_angle_pid.setSetpoint(self.snap_angle.radians())
                self.signal.get_speed().omega = self.snap_angle_pid.calculate(
                    self.get_pose().rotation().radians()
                )
                self.drive(self.signal.get_speed(), self.signal.get_field_relative())

                if self.is_aligned() and not self.manual:
                    self.state = DrivetrainState.RAW_INPUT_TELEOP

            case DrivetrainState.POINT_AT_REEF:
                if self.last_angular_input + 0.04 > self.timer.getFPGATimestamp():
                    self.state = DrivetrainState.RAW_INPUT_TELEOP
                self.snap_angle = reefscape_functions.angle_to_reef(
                    self.get_pose().translation()
                )
                self.fast_snap_pid.setSetpoint(self.snap_angle.radians())
                self.signal.get_speed().omega = self.fast_snap_pid.calculate(
                    self.get_gyro_yaw_value().radians()
                )
                self.drive(self.signal.get_speed(), self.signal.get_field_relative())

            case DrivetrainState.LINE_UP_TELEOP:
                if self.manual:
                    self.line_up_controller.setP(self.line_up_p)
                    self.line_up_controller.setI(self.line_up_i)
                    self.line_up_controller.setD(self.line_up_d)

                difference = (
                    self.target_pose.translation() - self.get_pose().translation()
                )
                self.line_up_controller.setSetpoint(0)
                speed = min(
                    MAX_LINE_UP_SPEED,
                    self.line_up_controller.calculate(-difference.norm()),
                )
                ratio = speed / difference.norm()
                vx = ratio * difference.X()
                vy = ratio * difference.Y()

                self.snap_angle = self.target_pose.rotation()
                self.snap_angle_pid.setSetpoint(self.snap_angle.radians())
                omega = self.snap_angle_pid.calculate(
                    # self.get_gyro_yaw_value().radians())
                    self.get_pose().rotation().radians()
                )

                self.set_signal(
                    DriveSignal(wpimath.kinematics.ChassisSpeeds(vx, vy, omega), True)
                )

                self.drive(self.signal.get_speed(), self.signal.get_field_relative())

                if self.is_aligned():
                    self.state = DrivetrainState.ALIGNED

            case DrivetrainState.LINE_UP_AUTON:
                difference = (
                    self.target_pose.translation() - self.get_pose().translation()
                )
                self.line_up_controller.setSetpoint(0)
                speed = min(
                    MAX_LINE_UP_SPEED,
                    self.line_up_controller.calculate(-difference.norm()),
                )
                ratio = speed / difference.norm()
                vx = ratio * difference.X()
                vy = ratio * difference.Y()

                self.snap_angle = self.target_pose.rotation()
                self.snap_angle_pid.setSetpoint(self.snap_angle.radians())
                omega = self.snap_angle_pid.calculate(
                    self.get_gyro_yaw_value().radians()
                )

                self.set_signal(
                    DriveSignal(wpimath.kinematics.ChassisSpeeds(vx, vy, omega), True)
                )

                self.drive(self.signal.get_speed(), self.signal.get_field_relative())

            case DrivetrainState.FOLLOW_PATH_AUTON:
                self.drive(self.signal.get_speed(), self.signal.get_field_relative())

            case DrivetrainState.ALIGNED:
                self.front_left.set_desired_state(
                    wpimath.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(225))
                )
                self.front_right.set_desired_state(
                    wpimath.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(135))
                )
                self.back_left.set_desired_state(
                    wpimath.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(315))
                )
                self.back_right.set_desired_state(
                    wpimath.kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(225))
                )

            case DrivetrainState.FEED_VOLTAGE:
                self.set_forward_signal(self.manual_voltage)

                self.drive(self.signal.get_speed(), self.signal.get_field_relative())


class DriveSignal:
    def __init__(
        self, speed: wpimath.kinematics.ChassisSpeeds, field_relative: bool = True
    ):
        """
        Creates a new DriveSignal

        Params:
            speed (ChassisSpeeds): Target speeds for the bot
            field_relative (bool): Whether the bot should drive in a field_relative manner
        """
        self._speed = speed
        self._field_relative = field_relative

    def get_speed(self):
        """
        Gets the speed attribute of the drive signal.

        Returns:
            ChassisSpeeds: Current target speeds of the bot.
        """
        if self._speed:
            return self._speed
        return wpimath.kinematics.ChassisSpeeds()

    def get_field_relative(self):
        """
        Gets whether the signal is to be treated as field relative.

        Returns:
            Whether the signal is field relative.
        """
        return self._field_relative


class DrivetrainState(Enum):
    """Possible states that the drivetrain can be in during a match"""

    NONE = 0  # The drivetrain is doing nothing
    # The drivetrain is locked, making it harder to push around (not used this game)
    LOCKED_TELEOP = 1
    RAW_INPUT_TELEOP = 2  # The drivetrain is recieving input from a controller
    # The drivetrain is passive snapping to a location in order to prevent drift (not used this game)
    PASSIVE_SNAP_TELEOP = 3
    ACTIVE_SNAP_TELEOP = 4  # The drivetrain is actively snapping to an angle
    # The drivetrain is pointing towards the reef at all times for max aura (not used this game)
    POINT_AT_REEF = 5
    LINE_UP_TELEOP = 6  # The drivetrain is lining up towards the reef in teleop
    LINE_UP_AUTON = 7  # The drivetrain is lining up toward the reef in autonomous
    FOLLOW_PATH_AUTON = 8  # The drivetrain is following a choreo path in auton
    # The drivetrain is currently alligned to its target position (idk if this is in main)
    ALIGNED = 9
    FEED_VOLTAGE = 10  # The drivetrain is being fed voltage for feedfoward tuning
