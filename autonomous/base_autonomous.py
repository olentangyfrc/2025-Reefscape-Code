from pathlib import Path
import wpilib
import math
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import choreo
from choreo.trajectory import SwerveTrajectory, SwerveSample
from magicbot import AutonomousStateMachine, state, timed_state
from wpilib import RobotBase
from wpimath.kinematics import ChassisSpeeds

from components.chassis.drivetrain import Drivetrain, DriveSignal, DrivetrainState
import utilities.ozone_utility_functions as utils
import utilities.reefscape_functions as rf
import utilities.constants as constants
from components.end_effector import EndEffector, EndEffectorStates
from components.elevator import Elevator, ElevatorState
from components.arm import Arm
from components.dashboard import Dashboard

GIVE_UP_TIME = 1.0
ELEVATOR_UP_TIME = 0.3
PUSH_DURATION = 0.9


class BaseAutonomous(AutonomousStateMachine):
    MODE_NAME = "DEFAULT_AUTONOMOUS"
    DEFAULT = True
    _AutonomousStateMachine__engaged = True

    drivetrain: Drivetrain
    elevator: Elevator
    end_effector: EndEffector
    dashboard: Dashboard
    arm: Arm

    def __init__(self) -> None:
        base_dir = "/home/lvuser/py/deploy/choreo/"
        if not RobotBase.isReal():
            base_dir = Path("deploy/choreo").resolve().as_posix() + "/"

        try:
            self.base_trajectories: dict[str, SwerveTrajectory] = {
                path: choreo.load_swerve_trajectory(base_dir + path)
                for path in [
                    "B-RightStation",
                    "C-RightStation",
                    "D-RightStation",
                    "E-RightStation",
                    "F-RightStation",
                    "RightStation-B",
                    "RightStation-C",
                    "RightStation-D",
                    "Start-E",
                ]
            }
        except ValueError:
            print("Alert! Auton initialization error\n" * 10)
        except FileNotFoundError:
            print("Alert! Missing trajectory files\n" * 10)

        self.timer = wpilib.Timer()
        self.release_timer = wpilib.Timer()
        self.intake_timer = wpilib.Timer()
        self.push_timer = wpilib.Timer()
        self.trajectories: list[SwerveTrajectory] = []
        self.current_path_index = 0
        self.push_phase = ""

        super().__init__()

    def on_enable(self) -> None:
        dashboard_auton = self.dashboard.get_custom_auton()
        if isinstance(dashboard_auton, str):
            dashboard_auton = dashboard_auton.split(",")

        if not dashboard_auton or dashboard_auton == [""]:
            print("No auto selected")
            self.done()
            return

        if not RobotBase.isReal():
            dashboard_auton = ["NET"]

        self.trajectories = []
        self.actions = []
        self.flipped_list = []
        self.starting_state = "follow_trajectory"

        # Handle PUSH-AUTON case first
        if dashboard_auton[0] == "PUSH-AUTON":
            self.starting_state = "push_movement_back"
            self.flipped_list = [False] * 6
            self.actions = [
                constants.Auton_Actions.REEF_SCORE,
                constants.Auton_Actions.INTAKE_CORAL,
            ] * 3
            self.trajectories = [
                self.base_trajectories["Start-E"],
                self.base_trajectories["E-RightStation"],
                self.base_trajectories["RightStation-D"],
                self.base_trajectories["D-RightStation"],
                self.base_trajectories["RightStation-C"],
                self.base_trajectories["C-RightStation"],
            ]
            self.drivetrain.reset_odometry(Pose2d(7.2, 2.42, math.pi))
        elif dashboard_auton[0] == "PUSH-AUTON-LEFT":
            self.starting_state = "push_movement_back"
            self.flipped_list = [True] * 6
            self.actions = [
                constants.Auton_Actions.REEF_SCORE,
                constants.Auton_Actions.INTAKE_CORAL,
            ] * 3
            self.trajectories = [
                self.base_trajectories["Start-E"],
                self.base_trajectories["E-RightStation"],
                self.base_trajectories["RightStation-D"],
                self.base_trajectories["D-RightStation"],
                self.base_trajectories["RightStation-C"],
                self.base_trajectories["C-RightStation"],
            ]
            self.drivetrain.reset_odometry(Pose2d(7.2, 2.42, math.pi))

        # Handle other cases
        elif dashboard_auton[0] == "THREE-PIECE-RIGHT":
            self.flipped_list = [False] * 6
            self.actions = [
                constants.Auton_Actions.REEF_SCORE,
                constants.Auton_Actions.INTAKE_CORAL,
            ] * 3
            self.trajectories = [
                self.base_trajectories["Start-E"],
                self.base_trajectories["E-RightStation"],
                self.base_trajectories["RightStation-D"],
                self.base_trajectories["D-RightStation"],
                self.base_trajectories["RightStation-C"],
                self.base_trajectories["C-RightStation"],
            ]
            self.drivetrain.reset_odometry(Pose2d(7.2, 2.42, math.pi))

        elif dashboard_auton[0] == "THREE-PIECE-LEFT":
            self.flipped_list = [True] * 6
            self.actions = [
                constants.Auton_Actions.REEF_SCORE,
                constants.Auton_Actions.INTAKE_CORAL,
            ] * 3
            self.trajectories = [
                self.base_trajectories["Start-E"],
                self.base_trajectories["E-RightStation"],
                self.base_trajectories["RightStation-D"],
                self.base_trajectories["D-RightStation"],
                self.base_trajectories["RightStation-C"],
                self.base_trajectories["C-RightStation"],
            ]
            self.drivetrain.reset_odometry(
                Pose2d(7.2, constants.FIELD_WIDTH - 2.42, math.pi)
            )

        elif dashboard_auton[0] == "SIMPLE-RIGHT":
            self.drivetrain.reset_odometry(
                Pose2d(7.2, constants.FIELD_WIDTH / 2, math.pi)
            )
            self.drivetrain.line_up(constants.Reef_Position.G.value)
            self.starting_state = "pid_to_G"

        elif dashboard_auton[0] == "SIMPLE-LEFT":
            self.drivetrain.reset_odometry(
                Pose2d(7.2, constants.FIELD_WIDTH / 2, math.pi)
            )
            self.drivetrain.line_up(constants.Reef_Position.H.value)
            self.starting_state = "pid_drive"

        elif dashboard_auton[0] == "NET":
            self.drivetrain.reset_odometry(
                Pose2d(7.2, constants.FIELD_WIDTH / 2, math.pi)
            )
            self.starting_state = "pid_to_G"

        # Handle custom auton sequences
        elif dashboard_auton[0].upper() in "ABCDEFGHIJKL":
            is_flipped = dashboard_auton[0].upper() in "HIJKLA"
            self.flipped_list = [is_flipped] * (len(dashboard_auton) * 2)

            start_pose = Pose2d(
                7.2, constants.FIELD_WIDTH - 2.42 if is_flipped else 2.42, math.pi
            )
            self.drivetrain.reset_odometry(start_pose)

            first_letter = self.flip_letter(dashboard_auton[0].upper(), is_flipped)
            self.trajectories.extend(
                [
                    self.base_trajectories[f"Start-{first_letter}"],
                    self.base_trajectories[f"{first_letter}-RightStation"],
                ]
            )
            self.actions.extend(
                [
                    constants.Auton_Actions.REEF_SCORE,
                    constants.Auton_Actions.INTAKE_CORAL,
                ]
            )

            for branch in dashboard_auton[1:]:
                branch_letter = self.flip_letter(branch.upper(), is_flipped)
                self.trajectories.extend(
                    [
                        self.base_trajectories[f"RightStation-{branch_letter}"],
                        self.base_trajectories[f"{branch_letter}-RightStation"],
                    ]
                )
                self.actions.extend(
                    [
                        constants.Auton_Actions.REEF_SCORE,
                        constants.Auton_Actions.INTAKE_CORAL,
                    ]
                )

        else:
            print(f"Unknown auton mode: {dashboard_auton[0]}")
            self.done()

    @state(first=True)
    def startup(self) -> None:
        self.next_state_now(self.starting_state)

    @timed_state(duration=0.9, next_state="push_movement_forward")
    def push_movement_back(self, initial_call):
        if initial_call:
            self.drivetrain.manual_drive()
        self.drivetrain.set_forward_signal(-10.0)

    @timed_state(duration=0.9, next_state="follow_trajectory")
    def push_movement_forward(self, initial_call):
        self.drivetrain.set_forward_signal(10.0)

    @state
    def pid_drive(self, initial_call) -> None:
        if initial_call:
            self.timer.restart()
            self.tripped = False

        if (
            self.drivetrain.get_pose()
            .translation()
            .distance(self.drivetrain.target_pose.translation())
            < 0.04
            and not self.tripped
        ):
            self.elevator.set(ElevatorState.L4)

        if (
            self.elevator.get_height() > 1.5
            and self.timer.get() > 3.0
            and not self.tripped
        ):
            self.end_effector.dispense()
            self.timer.restart()
            self.tripped = True

        if self.elevator.get_height() > 1.5 and self.timer.get() > 2.0 and self.tripped:
            self.elevator.set(ElevatorState.STOW)

    @state
    def intake_coral(self, initial_call) -> None:
        if initial_call:
            print("Intaking coral...")
            self.cancel_all()
            self.elevator.set(ElevatorState.STOW)
            self.end_effector.intake()
            self.intake_timer.restart()
            self.drivetrain.state = DrivetrainState.RAW_INPUT_TELEOP

        if self.end_effector.ready() or (
            self.intake_timer.get() > 0.15 and RobotBase.isReal()
        ):
            self.current_path_index += 1
            self.timer.restart()
            self.next_state_now("follow_trajectory")

    @state
    def score_on_reef(self, initial_call) -> None:
        if initial_call:
            print("Scoring on reef...")
            self.release_timer.reset()
            self.timer.restart()
            if not self.end_effector.ready():
                self.elevator.set(ElevatorState.STOW)
                self.current_path_index += 1
                self.release_timer.stop()
                self.release_timer.reset()
                self.timer.restart()
                self.next_state_now("follow_trajectory")

        trajectory = self.trajectories[self.current_path_index]
        if trajectory:
            sample = trajectory.sample_at(10, False)
            if sample:
                augmented = self.augment_sample(
                    sample, 10, self.flipped_list[self.current_path_index]
                )
                self.drivetrain.follow_trajectory(augmented, trajectory)
                self.drivetrain.target_pose = Pose2d(
                    augmented.x, augmented.y, augmented.heading
                )
                self.drivetrain.auton_drive()

        if self.end_effector.get_exit() or self.current_path_index == 0:
            self.elevator.set(ElevatorState.L4)

        if self.elevator.at_target() and self.elevator.get_height() > 0.1:
            self.end_effector.dispense()
            print("Dispensing game piece")
            if not self.end_effector.get_exit():
                if not self.release_timer.get() > 0:
                    self.release_timer.start()
                if self.release_timer.get() > 0.2:
                    self.elevator.set(ElevatorState.STOW)
                    self.current_path_index += 1
                    self.release_timer.stop()
                    self.release_timer.reset()
                    self.timer.restart()
                    self.next_state_now("follow_trajectory")

    @state
    def follow_trajectory(self, initial_call) -> None:
        if self.current_path_index >= len(self.trajectories):
            self.done()
            return

        if initial_call:
            self.timer.restart()
            if self.trajectories[self.current_path_index] and self.trajectories[
                self.current_path_index
            ].sample_at(10, False):
                fin: SwerveSample = self.augment_sample(
                    self.trajectories[self.current_path_index].sample_at(10, False),
                    10,
                    self.flipped_list[self.current_path_index],
                )
                self.drivetrain.target_pose = Pose2d(fin.x, fin.y, fin.heading)

        self.drivetrain.auton_drive()

        time = self.timer.get()
        trajectory = self.trajectories[self.current_path_index]
        action = self.actions[self.current_path_index]
        flipped = self.flipped_list[self.current_path_index]

        target_pose = Pose2d()
        if trajectory:
            sample = trajectory.sample_at(time, False)
            if sample:
                augmented = self.augment_sample(sample, time, flipped)
                self.drivetrain.follow_trajectory(augmented, trajectory)
                target_pose = Pose2d(augmented.x, augmented.y, augmented.heading)

        if (
            trajectory.get_total_time() - time < ELEVATOR_UP_TIME
            and action == constants.Auton_Actions.REEF_SCORE
            and (self.end_effector.get_exit() or self.current_path_index == 0)
        ):
            self.elevator.set(ElevatorState.L4)
        elif (
            self.end_effector.get_exit()
            and action == constants.Auton_Actions.REEF_SCORE
        ):
            self.elevator.set(ElevatorState.L2)

        if action == constants.Auton_Actions.INTAKE_CORAL:
            if RobotBase.isReal():
                self.end_effector.intake()
            if time > trajectory.get_total_time() and self.end_effector.ready():
                self.timer.reset()
                self.current_path_index += 1
                self.next_state_now("follow_trajectory")

        if self.is_aligned(target_pose) and (time > trajectory.get_total_time()):
            match action:
                case None:
                    self.current_path_index += 1
                    self.timer.reset()
                    self.next_state_now("follow_trajectory")
                case constants.Auton_Actions.REEF_SCORE:
                    self.next_state_now("score_on_reef")
                case constants.Auton_Actions.PROCCESOR_SCORE:
                    self.next_state_now("score_in_processor")
                case constants.Auton_Actions.Do_None:
                    self.next_state("Do_None")
        elif (
            time > trajectory.get_total_time() + GIVE_UP_TIME
            and action == constants.Auton_Actions.REEF_SCORE
        ):
            self.next_state_now("score_on_reef")

    @state
    def pid_to_G(self, initial_call) -> None:
        if initial_call:
            self.drivetrain.line_up(constants.Reef_Position.G.value)
            self.elevator.set(ElevatorState.L4)
            self.timer.restart()
            self.tripped = False

        if (
            self.drivetrain.get_pose()
            .translation()
            .distance(self.drivetrain.target_pose.translation())
            < 0.04
            and not self.tripped
        ):
            self.elevator.set(ElevatorState.L4)

        if (
            self.elevator.get_height() > 1.5
            and self.drivetrain.is_aligned()
            and not self.tripped
        ):
            self.end_effector.dispense()
            self.timer.restart()
            self.tripped = True

        if self.elevator.get_height() > 1.5 and self.timer.get() > 0.5 and self.tripped:
            self.next_state("G_to_Galgae")

    @state
    def G_to_Galgae(self, initial_call) -> None:
        if initial_call:
            self.drivetrain.line_up(rf.get_algae_loc("G"))
            self.timer.restart()
            self.elevator.set(ElevatorState.STOW)

            self.elevator.set(ElevatorState.ALGAE_LOW)

        if self.elevator.at_target() and self.drivetrain.is_aligned():
            self.arm.remove_algae()

        if self.arm.holding_algae:
            self.next_state("Galgae_to_net")

    @state
    def Galgae_to_net(self, initial_call) -> None:
        if initial_call:
            self.drivetrain.line_up(Pose2d(Translation2d(7.65, 5), Rotation2d(math.pi)))
            self.elevator.set(ElevatorState.STOW)

        if self.drivetrain.is_aligned() and self.timer.get() > 0.25:
            if not self.timer.isRunning():
                self.timer.restart()
            if self.timer.get() > 0.25:
                self.next_state("netting")

    @state
    def netting(self, initial_call) -> None:
        if initial_call:
            self.drivetrain.line_up(Pose2d(7.65, 5, 0))
            self.elevator.set(ElevatorState.STOW)

        if self.drivetrain.is_aligned():
            self.elevator.set(ElevatorState.NET)
            if self.elevator.at_target():
                self.arm.net()

        if not self.arm.holding_algae:
            self.arm.set_idle()
            if self.arm.get_pivot_position_deg() > 75:
                self.next_state("net_to_Ialgae")

    @state
    def net_to_Ialgae(self, initial_call) -> None:
        if initial_call:
            self.drivetrain.line_up(rf.get_algae_loc("I"))
            self.timer.restart()
            self.elevator.set(ElevatorState.ALGAE_HIGH)

        if self.drivetrain.is_aligned() and not self.arm.holding_algae:
            self.elevator.set(ElevatorState.ALGAE_HIGH)
            self.arm.remove_algae()

        if self.arm.holding_algae:
            self.elevator.set(ElevatorState.NET_INTERMEDIATE)
            self.next_state("final_pid")

    @state
    def final_pid(self, initial_call) -> None:
        if initial_call:
            self.drivetrain.line_up(Pose2d(6.6, 5.5, math.pi))
            self.elevator.set(ElevatorState.STOW)

    def flip_letter(self, letter: str, should_flip: bool) -> str:
        if not should_flip:
            return letter
        return "BCDEFGHIJKLA"[11 - "BCDEFGHIJKLA".index(letter)]

    def done(self) -> None:
        self.drivetrain.set_signal(DriveSignal(ChassisSpeeds(0, 0, 0), True))
        super().done()

    def cancel_all(self):
        self.timer.restart()
        self.drivetrain.set_signal(DriveSignal(ChassisSpeeds(0, 0, 0), True))
        self.elevator.set(ElevatorState.STOW)
        if self.end_effector.holding_coral:
            self.end_effector.state = EndEffectorStates.HOLDING
        else:
            self.end_effector.stop()

    def augment_sample(
        self, sample: SwerveSample, time: float, flipped: bool
    ) -> SwerveSample:
        augmented_sample = SwerveSample(
            time,
            sample.x,
            sample.y,
            sample.heading,
            sample.vx,
            sample.vy,
            sample.omega,
            sample.ax,
            sample.ay,
            sample.alpha,
            sample.fx,
            sample.fy,
        )
        if flipped:
            augmented_sample.y = constants.FIELD_WIDTH - sample.y
            augmented_sample.heading = -sample.heading
            augmented_sample.vy = -sample.vy
            augmented_sample.ay = -sample.ay
            augmented_sample.omega = -sample.omega
            augmented_sample.alpha = -augmented_sample.alpha
        return augmented_sample

    def is_aligned(self, sample_pose) -> bool:
        return utils.within_pose_tolerance(
            self.drivetrain.get_pose(), sample_pose, 0.05, 1
        )
