import magicbot
import wpilib
from wpilib import Timer, DriverStation
import wpimath
from wpimath.geometry import Rotation2d, Pose2d, Translation2d, Transform2d
import wpimath.kinematics
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory
import math
from wpinet import WebServer

from components.chassis.drivetrain import Drivetrain, DriveSignal, DrivetrainState
from components.funnel import Funnel
from components.vision import Vision
from components.arm import Arm, ArmStates
from components.led import LEDs, LEDStates
import utilities.constants as constants
import utilities.reefscape_functions as rf
import utilities.ozone_utility_functions as utility_functions
import wpimath.geometry
from components.dashboard import Dashboard
from utilities.constants import Reef_Position, Robot_State
from components.climber import Climber
from components.elevator import Elevator, ElevatorState
from components.end_effector import EndEffector, EndEffectorStates
from utilities import elasticlib, file_utils
from wpilib import RobotBase

from magicbot import tunable, feedback

# Transfers the elastic layout if it is modified 
file_utils.search_and_copy("ozone-elastic-layout", wpilib.getDeployDirectory())

"""
    While many teams will opt for a superstructure and IO class/file, we opted to manage all of
    our controller inputs and robot state logic in the main robot.py file. This isn't the generally 
    recommended way to do things, it's just the solution we picked early in the season and stuck with.
    99% of logging is done within each individual subsystem using feedback tags (a magicbot feature)
"""

class MyRobot(magicbot.MagicRobot):
    arm: Arm # Controls algae manipulating arm and corresponding wheels
    drivetrain: Drivetrain # Control of swerve drive and gyro
    funnel: Funnel # Controls the servos that move the funnel up and out of the way to prepare for the climb
    limelight: Vision # Manages limelights, passes an instance of drivetrain into 
                      #constructor to add vision measurements directly to pose estimator.
    dashboard: Dashboard # Manages interactions with the aux driver's dashboard/tablet over NetworkTables
    climber: Climber # Pulls climber down (Climber is spring-loaded, so extension is not necessary)
    elevator: Elevator # Only has control over elevator motors
    end_effector: EndEffector # Responsible for intaking and shooting coral
    leds: LEDs


    field_oriented = True
    enable_line_up = tunable(True)
    enable_offset = tunable(True) # This was a quck solution at champs to an issue where we were consistenly missing placements to the right.

    def createObjects(self) -> None:
        """
        Called on initialization; creates controllers for the drivers
        """
        wpilib.DataLogManager.start()
        wpilib.DataLogManager.logNetworkTables(True)
        wpilib.DataLogManager.logConsoleOutput(True)
        DriverStation.startDataLog(wpilib.DataLogManager.getLog())
        self.main_controller = wpilib.XboxController(0)
        self.aux_controller = wpilib.XboxController(1)

        WebServer.getInstance().start(5800, wpilib.getDeployDirectory())
        
        self.state = Robot_State.DRIVE
        self.is_arm_ready = False
        self.backup_timer = Timer()
        self.l1_pop_index = None
        self.l1_speed_timer = Timer()

    def disabledPeriodic(self):
        """Runs periodically when robot is disabled"""
        if DriverStation.isFMSAttached():
            elasticlib.select_tab("Preflight")

    def autonomousInit(self):
        """Runs all initialization code for autonomous"""
        elasticlib.select_tab("Autonomous")
    
    def teleopInit(self) -> None:
        """Runs all initialization code for teleop"""
        elasticlib.select_tab("Teleoperated")
        self.drivetrain.reset_trajectory()  
        self.state = Robot_State.DRIVE
        self.end_effector.stop()

        # Clear button input queue to prevent driver input from affecting the robot before a match
        self.main_controller.getAButtonPressed()
        self.main_controller.getXButtonPressed()
        self.main_controller.getYButtonPressed()
        self.main_controller.getBButtonPressed()
        self.main_controller.getStartButtonPressed()
        self.main_controller.getBackButtonPressed()

    def teleopPeriodic(self) -> None:
        """
        Runs periodic logic for the robot in teleop
        """

        self.elevator.set_arm_position(
            math.degrees(self.arm.get_pivot_position())) # Send current arm angle to elevator to prevent crash scenarios

        """Main Controls"""
        if self.main_controller.getStartButtonPressed():
            self.cancel_all()
            self.state = Robot_State.DRIVE

        if self.main_controller.getBackButtonPressed():
            self.drivetrain.zero_gyro()

        if len(self.dashboard.get_node()) > 1: # Dashboard nodes are sent with the structure [Reef letter Placement height].
                                               # For example: "E2", "A4", "J3"
            if (not self.state in [Robot_State.CLIMB, Robot_State.LINE_UP_TO_REEF]) and self.main_controller.getRightBumper() and self.end_effector.get_exit():
                node = self.dashboard.get_node().capitalize()
                if rf.can_line_up(self.drivetrain.get_pose().translation(), node[0]):
                    if self.enable_line_up:
                        self.drivetrain.line_up(
                            Reef_Position.from_string(node[0]).transformBy(Transform2d(0,0.02*self.enable_offset,0)))
                    self.elevator.go_to(ElevatorState.from_int(
                        int(node[1])))
                    self.end_effector.set_height(int(node[1]))
                    self.state = Robot_State.LINE_UP_TO_REEF
        
        if self.main_controller.getAButtonPressed():
            if (not self.arm.holding_algae or (not RobotBase.isReal() and self.arm.sim_holding)) and self.drivetrain.is_manual() and self.state != Robot_State.CLIMB:
                self.state = Robot_State.GROUND_INTAKE
                self.drivetrain.manual_drive()

        if self.main_controller.getBButtonPressed():
            self.arm.state = ArmStates.IDLE
            if self.end_effector.ready() and self.arm.get_pivot_position_deg() > 80:
                if self.main_controller.getLeftTriggerAxis() > 0.5:
                    self.state = Robot_State.L1_STACK
                else:
                    self.state = Robot_State.L1_DROP
                    self.drivetrain.line_up(constants.L1_End_Effector_Reef_Position.from_string(rf.nearest_reef_branch(self.drivetrain.get_pose().translation() + self.drivetrain.get_velocity().translation() * 0.15)))
                    self.elevator.set(ElevatorState.L1)

        if self.main_controller.getYButtonPressed():
            if self.arm.state == ArmStates.HOLD_ALGAE:
                if self.drivetrain.get_pose().Y() < 3:
                    self.state = Robot_State.PROCESSOR
                    self.drivetrain.line_up(constants.PROCESSOR_POSE)
                    self.arm.prepare_process()
                else:
                    self.state = Robot_State.NET

            elif self.arm.state in [ArmStates.PREPARE_REMOVE_ALGAE, ArmStates.REMOVE_ALGAE, ArmStates.IDLE]:
                self.state = Robot_State.DEALGIFY_REEF
                relative_translation = self.drivetrain.get_pose().translation()
                if self.drivetrain.get_pose().X() < constants.FIELD_LENGTH/2:
                    self.drivetrain.line_up(rf.get_algae_loc(rf.nearest_reef_branch(
                        self.drivetrain.get_pose().translation())))
                else:
                    # Opposite alliance algae stealing
                    blue_loc = Translation2d(constants.FIELD_LENGTH-self.drivetrain.get_pose().X(), constants.FIELD_WIDTH-self.drivetrain.get_pose().Y())
                    blue_target = rf.get_algae_loc(rf.nearest_reef_branch(blue_loc))
                    self.drivetrain.line_up(Pose2d(constants.FIELD_LENGTH-blue_target.X(), constants.FIELD_WIDTH-blue_target.Y(), -blue_target.rotation()))
                    relative_translation = Translation2d(constants.FIELD_LENGTH-relative_translation.X(), constants.FIELD_WIDTH-relative_translation.Y())
                self.arm.remove_algae()
                if rf.nearest_reef_branch(relative_translation) in "abefij":
                    self.elevator.set(ElevatorState.ALGAE_HIGH)
                else:
                    self.elevator.set(ElevatorState.ALGAE_LOW)

        if self.main_controller.getXButtonPressed():
            if not self.state in [Robot_State.LINE_UP_TO_REEF, Robot_State.L1_DROP, Robot_State.L1_STACK]:
                if self.enable_line_up:
                    # Find the branch that the bot will be closest to after a fraction of a second
                    self.drivetrain.line_up(Reef_Position.from_string(
                        rf.nearest_reef_branch(self.drivetrain.get_pose().translation()+self.drivetrain.get_velocity().translation()*0.15)).transformBy(Transform2d(0,0.02*self.enable_offset,0)))
                self.state = Robot_State.LINE_UP_TO_REEF
            elif self.state in [Robot_State.L1_DROP, Robot_State.L1_STACK]:
                self.drivetrain.state = DrivetrainState.ALIGNED
            elif self.elevator.get_height() > 0.03:
                self.end_effector.dispense()
        
        if self.main_controller.getRightTriggerAxis() > 0.5 and self.arm.get_pivot_position_deg() > 80:
            self.state = Robot_State.STATION_INTAKE
            if self.enable_line_up:
                y = min(self.drivetrain.get_pose().Y(),
                        constants.FIELD_WIDTH - self.drivetrain.get_pose().Y())
                position = Translation2d(0.702, 1.370)
                if self.drivetrain.get_pose().X() > y:
                    position = Translation2d(1.61, 0.666)
                self.drivetrain.line_up(Pose2d(position, Rotation2d(0.94)))
                if self.drivetrain.get_pose().Y() > constants.FIELD_WIDTH / 2:
                    self.drivetrain.line_up(
                        Pose2d(position.X(), constants.FIELD_WIDTH - position.Y(), Rotation2d(-0.94)))

            self.end_effector.intake()
            self.elevator.set(ElevatorState.STOW)

        if self.state == Robot_State.STATION_INTAKE and self.main_controller.getRightTriggerAxis() < 0.5 and self.arm.state != ArmStates.HOLD_CORAL and self.arm.get_pivot_position()*180/math.pi > 90:
            self.end_effector.intake()
            self.drivetrain.manual_drive()

        if self.arm.get_pivot_position_deg() > 80:
            match self.main_controller.getPOV(0):
                case -1:
                    pass
                case 0:
                    self.elevator.set(ElevatorState.L2)
                    self.end_effector.set_height(2)
                case 90:
                    if math.degrees(self.arm.get_pivot_position()) > -30:
                        self.elevator.set(ElevatorState.STOW)
                case 180:
                    self.elevator.set(ElevatorState.L4)
                    self.end_effector.set_height(4)
                case 270:
                    self.elevator.set(ElevatorState.L3)
                    self.end_effector.set_height(3)

        self.drive_with_joystick(True)

        """Aux Controls"""
        if self.aux_controller.getAButton() or (self.main_controller.getLeftTriggerAxis() > 0.9 and self.main_controller.getPOV(0) == 90):
            if self.drivetrain.get_pose().X() > 6.5 or DriverStation.getMatchTime() < 20.0 or not self.enable_line_up or self.aux_controller.getBButton():
                self.state = Robot_State.CLIMB
                self.drivetrain.climbing = True

        if self.funnel.is_climbing:
            self.state = Robot_State.CLIMB
        self.drivetrain.tighten_tolerances = False

        """State logic"""
        match self.state:
            case Robot_State.DRIVE:
                if not self.drivetrain.is_manual():
                    self.drivetrain.manual_drive()

            case Robot_State.LINE_UP_TO_REEF:
                if (self.drivetrain.state == DrivetrainState.ALIGNED or not self.enable_line_up) and \
                        self.elevator.at_target() and \
                        self.elevator.get_height() > 0.1 and \
                        self.end_effector.ready():
                    self.end_effector.dispense()
                elif self.end_effector.is_done():
                    self.elevator.set(ElevatorState.STOW)
                    self.state = Robot_State.DRIVE
                    self.drivetrain.manual_drive()

            case Robot_State.CLIMB:
                self.drivetrain.manual_drive()
                self.elevator.set(ElevatorState.STOW)
                if self.elevator.get_height() < 0.02:
                    self.funnel.climb()
                    self.leds.state = LEDStates.READY_TO_CLIMB
                if self.aux_controller.getXButton() or (self.main_controller.getAButton()):
                    self.climber.climb()

            case Robot_State.STATION_INTAKE:
                if self.arm.get_pivot_position()*180/math.pi > 80:
                    self.elevator.set(ElevatorState.STOW)
                    if self.end_effector.ready():
                        self.state = Robot_State.DRIVE
                        self.drivetrain.manual_drive()
                if self.main_controller.getRightTriggerAxis() <= 0.5:
                    self.drivetrain.manual_drive()
                
                if not RobotBase.isReal() and self.drivetrain.is_aligned():
                    self.end_effector.intake()
                    
            case Robot_State.DEALGIFY_REEF:
                if self.drivetrain.is_aligned() or self.drivetrain.state != DrivetrainState.LINE_UP_TELEOP or self.arm.is_holding(): 
                    self.drivetrain.manual_drive()
                    if rf.left_reef_zone(self.drivetrain.get_pose().translation()):
                        self.elevator.set(ElevatorState.STOW)
                        self.arm.state = ArmStates.HOLD_ALGAE
                        self.arm.holding_algae = True
                        self.arm.timer.reset()
                        self.arm.timer.stop()

                if not self.arm.holding_algae:
                    if rf.nearest_reef_branch(self.drivetrain.get_pose().translation()) in "abefij":
                        self.elevator.set(ElevatorState.ALGAE_HIGH)
                    else:
                        self.elevator.set(ElevatorState.ALGAE_LOW)
                    self.arm.remove_algae()
                else:
                    self.elevator.set(ElevatorState.STOW)

            case Robot_State.PROCESSOR:
                if self.arm.holding_algae or (not RobotBase.isReal() and self.arm.sim_holding):
                    self.elevator.set(ElevatorState.STOW)
                    if self.elevator.at_target() and (self.drivetrain.state == DrivetrainState.ALIGNED or self.drivetrain.get_pose().translation().distance(self.drivetrain.target_pose.translation()) < 0.05):
                        self.arm.processor()
                        self.drivetrain.manual_drive()
                    else:
                        self.arm.prepare_process()
                else:
                    self.arm.set_idle()
                    if math.degrees(self.arm.get_pivot_position()) > 0:
                        self.elevator.set(ElevatorState.STOW)
                        self.state = Robot_State.DRIVE
            
            case Robot_State.NET:
                if self.arm.holding_algae:
                    self.elevator.set(ElevatorState.NET)
                    if self.drivetrain.get_pose().X() < constants.FIELD_LENGTH / 2:
                        self.drivetrain.enable_active_snap(Rotation2d())
                    if not self.backup_timer.isRunning():
                        self.backup_timer.restart()
                    if self.elevator.at_target() and self.backup_timer.get() > 0.5:
                        self.arm.net()
                        self.backup_timer.stop()
                        self.backup_timer.reset()
                        self.drivetrain.manual_drive()
                else:
                    self.arm.set_idle()
                    if math.degrees(self.arm.get_pivot_position()) > 65:
                        self.drivetrain.manual_drive()
                        if not self.backup_timer.isRunning():
                            self.backup_timer.restart()
                        if self.drivetrain.get_signal().get_speed().vx < -0.5 or self.backup_timer.get() > 1: 
                            self.backup_timer.stop()
                            self.backup_timer.reset()
                            self.elevator.set(ElevatorState.STOW)
                            self.state = Robot_State.DRIVE
            
            case Robot_State.L1_DROP:
                self.drivetrain.tighten_tolerances = True
                if self.drivetrain.state == DrivetrainState.ALIGNED and self.elevator.at_target() and not self.backup_timer.isRunning():
                    self.end_effector.set_height(1)
                    self.end_effector.state = EndEffectorStates.DISPENSING
                    self.backup_timer.restart()
                if self.backup_timer.get() > 0.4:
                    self.backup_timer.reset()
                    self.backup_timer.stop()
                    self.state = Robot_State.L1_SWIPE
                    # Get new target location for drivetrain at center of the reef
                    new_target = Pose2d()
                    for i in [
                        constants.AT_Coordinates.AB, 
                        constants.AT_Coordinates.CD, 
                        constants.AT_Coordinates.EF, 
                        constants.AT_Coordinates.GH,
                        constants.AT_Coordinates.IJ, 
                        constants.AT_Coordinates.KL
                    ]:
                        if rf.nearest_reef_branch(self.drivetrain.get_pose().translation()).capitalize() in i.name:
                            new_target = i.value
                            new_target = new_target.transformBy(Transform2d(-0.44, 0, 0))
                    self.drivetrain.line_up(new_target)
                    self.end_effector.stop()

                    current = rf.nearest_reef_branch(self.drivetrain.get_pose().translation()).capitalize()
                    conjugate = constants.L1_End_Effector_Reef_Position.from_string("BADCFEHGJILK"["ABCDEFGHIJKL".index(current)])
                    self.drivetrain.line_up(conjugate)

            case Robot_State.L1_SWIPE:
                if self.drivetrain.target_pose.translation().distance(self.drivetrain.get_pose().translation()) < 0.3:
                    self.state = Robot_State.DRIVE
                    self.drivetrain.manual_drive()
                    if not RobotBase.isReal():
                        self.end_effector.dispense()
            
            case Robot_State.L1_STACK:
                if not self.end_effector.ready() or not RobotBase.isReal():
                    if not self.backup_timer.isRunning():
                        self.backup_timer.start()
                    if self.backup_timer.get() > 0.25:
                            if not self.l1_pop_index is None:
                                constants.L1_STACK_REFERENCE.pop(self.l1_pop_index)
                                self.l1_pop_index = None
                            self.drivetrain.manual_drive()
                            self.state = Robot_State.DRIVE
                            self.backup_timer.stop()
                            self.backup_timer.reset()
                            self.l1_speed_timer.stop()
                            self.l1_speed_timer.reset()
                elif not self.l1_speed_timer.isRunning():
                    self.l1_speed_timer.restart()
                # elif self.l1_speed_timer.get() < 0.:
                #     self.end_effector.set_height(4)
                else:
                    self.end_effector.set_height(1.5)

                self.l1_pop_index = constants.L1_STACK_REFERENCE.index(
                    self.drivetrain.get_pose().nearest(constants.L1_STACK_REFERENCE)
                )
                self.drivetrain.line_up(constants.L1_STACK_REFERENCE[self.l1_pop_index])
                self.elevator.set(ElevatorState.L1_STACK)
                self.end_effector.set_height(1.5)

                if self.drivetrain.is_aligned() and self.elevator.at_target() or not RobotBase.isReal():
                        self.end_effector.dispense()


            case Robot_State.GROUND_INTAKE:
                if not self.arm.holding_algae:
                    self.elevator.set(ElevatorState.GROUND_INTAKE)
                    self.arm.ground_intake()
                    self.drivetrain.manual_drive()
                elif self.arm.state == ArmStates.HOLD_ALGAE:
                    self.state = Robot_State.DRIVE
                    self.elevator.set(ElevatorState.STOW)
                    

    def drive_with_joystick(self, field_relative: bool) -> None:
        """
        Called in the periodic loop; gets joystick input from driver and applies logic to the drivetrain as necessary.

        Params:
            field_relative (bool): Whether the drivetrain should operate in a field relative or bot relative frame of reference
        """
        rot = 0
        x_speed = 0
        y_speed = 0
        modifier = 1
        if self.main_controller.getLeftBumperButton():
            modifier = 0.13
        if self.drivetrain.is_manual():
            x_speed = -utility_functions.filter_input(
                self.main_controller.getLeftY()) * constants.MAX_LINEAR_SPEED * modifier
            y_speed = -utility_functions.filter_input(
                self.main_controller.getLeftX()) * constants.MAX_LINEAR_SPEED * modifier
            rot = -utility_functions.filter_input(
                self.main_controller.getRightX()) * constants.MAX_ROTATION_SPEED * modifier

        if abs(rot) > constants.DEADBAND:
            self.drivetrain.last_angular_input = Timer.getFPGATimestamp()
        if math.sqrt(x_speed ** 2 + y_speed ** 2) > constants.DEADBAND:
            self.drivetrain.last_translational_input = Timer.getFPGATimestamp()
        speeds = wpimath.kinematics.ChassisSpeeds(x_speed, y_speed, rot)
        self.drivetrain.set_signal(DriveSignal(speeds, field_relative))

    def cancel_all(self):
        """
        Cancels all commands and gives full operation control back to the driver. Some states may
        not cancel if they can't safely do so
        """
        if self.state == Robot_State.CLIMB:
            return
        if self.state == Robot_State.NET and self.arm.is_holding():
            self.elevator.set(ElevatorState.STOW)
            self.state = Robot_State.DRIVE
        elif self.arm.state == ArmStates.HOLD_ALGAE:
            self.arm.state = ArmStates.PROCESSOR
        elif (math.degrees(self.arm.get_pivot_position()) >= -60 or self.drivetrain.is_manual()):
            if math.degrees(self.arm.get_pivot_position()) > 70:
                self.elevator.set(ElevatorState.STOW)
            self.end_effector.stop()
            self.state = Robot_State.DRIVE
            self.arm.set_idle()
        self.drivetrain.manual_drive()
        self.l1_speed_timer.stop()

    @feedback
    def get_state_string(self) -> str:
        """
        Returns the current state of the robot

        Returns:
            str: a string representing the current enum state of the bot
        """
        return self.state.name
    
    @feedback
    def get_match_time(self) -> float:
        """
        Returns an aproximation of the amount of time left in a match

        Returns:
            float: the time in seconds left in current match period, auton or teleop
        """
        return self.backup_timer.getMatchTime()
    
    @feedback
    def get_voltage(self) -> float:
        """
        Returns the current voltage of the entire robot

        Returns:
            float: voltage in volts that the robot is running at (12.4+ during idle is good)
        """
        return DriverStation.getBatteryVoltage()
    
    @feedback
    def get_main_controller_connection(self) -> bool:
        """
        Returns whether or not main controller input is recieved

        Returns: 
            bool: True if main controller is holding down the a button else false
        """
        return self.main_controller.getLeftTriggerAxis() > 0.5
    
    @feedback
    def get_aux_controller_connection(self) -> bool:
        """
        Returns whether or not aux controller input is recieved

        Returns:
            bool: True if aux controller is pressing down the b button else false
        """
        return self.aux_controller.getBButton()
    