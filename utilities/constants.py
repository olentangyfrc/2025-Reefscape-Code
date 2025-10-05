import math
from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from wpilib import RobotBase

from enum import Enum

# Field
FIELD_WIDTH = 8.0518
FIELD_LENGTH = 17.548225
REEF_LENGTH = 0.46

LATERAL_REEF_OFFSET = 0.1651
LINE_UP_DISTANCE = 0.51
L1_LINE_UP_DISTANCE = 0.67

NET_LENGTH = FIELD_WIDTH - 4.334
ALGAE_LINEUP_X = 7.4
NUM_PTS = 3
L1_STACK_SHIFT = 0.44  # Hypotenuse of shift from reef

TRANSFORM_LEFT = Transform2d(-LINE_UP_DISTANCE, LATERAL_REEF_OFFSET, 0)
TRANSFORM_RIGHT = Transform2d(-LINE_UP_DISTANCE, -LATERAL_REEF_OFFSET, 0)

L1_ARM_TRANSFORM_LEFT = Transform2d(-L1_LINE_UP_DISTANCE, LATERAL_REEF_OFFSET + 0.03, 0)
L1_ARM_TRANSFORM_RIGHT = Transform2d(
    -L1_LINE_UP_DISTANCE, -LATERAL_REEF_OFFSET - 0.03, 0
)

L1_END_EFFECTOR_TRANSFORM_LEFT = Transform2d(-0.44, 0.38, 0)
L1_END_EFFECTOR_TRANSFORM_RIGHT = Transform2d(-0.44, -0.38, 0)

PROCESSOR_POSE = Pose2d(6, 0.81, Rotation2d.fromDegrees(-90))


class Robot_State(Enum):
    """
    Enum representing the different states a robot can be in during operation.

    Each state corresponds to a specific phase or task the robot is performing
    each match.
    """

    DRIVE = 1  # The robot is driving
    MANUAL_OVERRIDE = 2  # The driver pressed the cancel commands button
    STATION_INTAKE = 3  # The robot is intaking from the coral station
    LINE_UP_TO_REEF = 4  # The robot is lining up to the reef
    DEALGIFY_REEF = 5  # The robot if dealgifying the reef
    PROCESSOR = 6  # The robot is scoring in the processor
    GROUND_INTAKE = 7  # The robot is intaking algae form the ground
    L1_PLACING = 8  # The robot is placing L1 coral
    CLIMB = 9  # The robot is deep climbing
    PREPARE_DEALGIFY_REEF = 10  # The robot is preparing to dealgify the reef
    L1_DROP = 11  # The robot is placing a coral on L1 using the end effector
    L1_SWIPE = 12  # Second part of the L1 sequence, swipes towards the center of the reef to lay a coral in the trough
    NET = 13  # Net...what did you expect
    L1_STACK = 14


class AT_Coordinates(Enum):
    """The april tag poses for each reef pairing (ex. AB)"""

    AB = Pose2d(3.6576, 4.0208, math.radians(0))
    CD = Pose2d(4.0739, 3.3012, math.radians(60))
    EF = Pose2d(4.9047, 3.3012, math.radians(120))
    GH = Pose2d(5.3210, 4.0208, math.radians(180))
    IJ = Pose2d(4.9047, 4.7455, math.radians(-120))
    KL = Pose2d(4.0739, 4.7455, math.radians(-60))


class Reef_Position(Enum):
    """The poses used to line up at each reef for scoring L2-L4"""

    A = AT_Coordinates.AB.value.transformBy(TRANSFORM_LEFT)
    B = AT_Coordinates.AB.value.transformBy(TRANSFORM_RIGHT)
    C = AT_Coordinates.CD.value.transformBy(TRANSFORM_LEFT)
    D = AT_Coordinates.CD.value.transformBy(TRANSFORM_RIGHT)
    E = AT_Coordinates.EF.value.transformBy(TRANSFORM_LEFT)
    F = AT_Coordinates.EF.value.transformBy(TRANSFORM_RIGHT)
    G = AT_Coordinates.GH.value.transformBy(TRANSFORM_LEFT)
    H = AT_Coordinates.GH.value.transformBy(TRANSFORM_RIGHT)
    I = AT_Coordinates.IJ.value.transformBy(TRANSFORM_LEFT)
    J = AT_Coordinates.IJ.value.transformBy(TRANSFORM_RIGHT)
    K = AT_Coordinates.KL.value.transformBy(TRANSFORM_LEFT)
    L = AT_Coordinates.KL.value.transformBy(TRANSFORM_RIGHT)

    Start1 = Pose2d(7.18, 0.68, Rotation2d.fromDegrees(180))
    Start2 = Pose2d(7.10, 2.200, Rotation2d.fromDegrees(180))

    @staticmethod
    def from_string(reef_position: str) -> Pose2d:
        """
        Returns a reef position given a certain letter ex. returns the lineup position
        of reef a if inputted "a"

        Params:
            reef_position(str): the reef letter to line up to, must be between A and L

        Returns:
            Pose2d: the pose to line up to in order to score L2-L4
        """
        match reef_position:
            case "a" | "A":
                return Reef_Position.A.value
            case "b" | "B":
                return Reef_Position.B.value
            case "c" | "C":
                return Reef_Position.C.value
            case "d" | "D":
                return Reef_Position.D.value
            case "e" | "E":
                return Reef_Position.E.value
            case "f" | "F":
                return Reef_Position.F.value
            case "g" | "G":
                return Reef_Position.G.value
            case "h" | "H":
                return Reef_Position.H.value
            case "i" | "I":
                return Reef_Position.I.value
            case "j" | "J":
                return Reef_Position.J.value
            case "k" | "K":
                return Reef_Position.K.value
            case "l" | "L":
                return Reef_Position.L.value


class L1_End_Effector_Reef_Position(Enum):
    A = AT_Coordinates.AB.value.transformBy(L1_END_EFFECTOR_TRANSFORM_LEFT)
    B = AT_Coordinates.AB.value.transformBy(L1_END_EFFECTOR_TRANSFORM_RIGHT)
    C = AT_Coordinates.CD.value.transformBy(L1_END_EFFECTOR_TRANSFORM_LEFT)
    D = AT_Coordinates.CD.value.transformBy(L1_END_EFFECTOR_TRANSFORM_RIGHT)
    E = AT_Coordinates.EF.value.transformBy(L1_END_EFFECTOR_TRANSFORM_LEFT)
    F = AT_Coordinates.EF.value.transformBy(L1_END_EFFECTOR_TRANSFORM_RIGHT)
    G = AT_Coordinates.GH.value.transformBy(L1_END_EFFECTOR_TRANSFORM_LEFT)
    H = AT_Coordinates.GH.value.transformBy(L1_END_EFFECTOR_TRANSFORM_RIGHT)
    I = AT_Coordinates.IJ.value.transformBy(L1_END_EFFECTOR_TRANSFORM_LEFT)
    J = AT_Coordinates.IJ.value.transformBy(L1_END_EFFECTOR_TRANSFORM_RIGHT)
    K = AT_Coordinates.KL.value.transformBy(L1_END_EFFECTOR_TRANSFORM_LEFT)
    L = AT_Coordinates.KL.value.transformBy(L1_END_EFFECTOR_TRANSFORM_RIGHT)

    @staticmethod
    def from_string(reef_position: str) -> Pose2d:
        match reef_position:
            case "a" | "A":
                return L1_End_Effector_Reef_Position.A.value
            case "b" | "B":
                return L1_End_Effector_Reef_Position.B.value
            case "c" | "C":
                return L1_End_Effector_Reef_Position.C.value
            case "d" | "D":
                return L1_End_Effector_Reef_Position.D.value
            case "e" | "E":
                return L1_End_Effector_Reef_Position.E.value
            case "f" | "F":
                return L1_End_Effector_Reef_Position.F.value
            case "g" | "G":
                return L1_End_Effector_Reef_Position.G.value
            case "h" | "H":
                return L1_End_Effector_Reef_Position.H.value
            case "i" | "I":
                return L1_End_Effector_Reef_Position.I.value
            case "j" | "J":
                return L1_End_Effector_Reef_Position.J.value
            case "k" | "K":
                return L1_End_Effector_Reef_Position.K.value
            case "l" | "L":
                return L1_End_Effector_Reef_Position.L.value


class Auton_Actions(Enum):
    """
    Possible actions that can be done on each route in an autonomous routine

    Routes only support having one auton action per each
    """

    INTAKE_CORAL = 0  # Robot is intaking coral from the coral station
    PROCCESOR_SCORE = 1  # Robot is scoring algae in the processor
    REEF_SCORE = 2  # Robot is scoring L4 coral on the reef
    Do_None = 3  # No action done


def generate_L1_stack_poses(center: AT_Coordinates, pts: int) -> list[Pose2d]:
    """
    Generates L1 stacking poses from the april tag on each reef.

    Params:
        center (AT_Coordinate): the AT_Coordinates enum value for each reef
        pts (int): the number of points (including center) generated

    Returns:
        list[Pose2d]: a list containing poses on the edge of the reef face scaled by angle and an
        external shift
    """

    r_list = []
    center_pose = center.value
    step = REEF_LENGTH / (pts - 1)

    cos_val = math.cos(-center_pose.rotation().radians())
    sin_val = math.sin(-center_pose.rotation().radians())

    x_shift = cos_val * L1_STACK_SHIFT
    y_shift = sin_val * L1_STACK_SHIFT

    for i in range(pts):
        offset = i - (pts - 1) / 2
        dx = offset * (step) * sin_val - x_shift
        dy = offset * (step) * cos_val + y_shift
        r_list.append(
            Pose2d(center_pose.X() + dx, center_pose.Y() + dy, center_pose.rotation())
        )

    return r_list


L1_STACK_REFERENCE = []
L1_STACK_REFERENCE.extend(generate_L1_stack_poses(AT_Coordinates.AB, NUM_PTS))
L1_STACK_REFERENCE.extend(generate_L1_stack_poses(AT_Coordinates.CD, NUM_PTS))
L1_STACK_REFERENCE.extend(generate_L1_stack_poses(AT_Coordinates.EF, NUM_PTS))
L1_STACK_REFERENCE.extend(generate_L1_stack_poses(AT_Coordinates.GH, NUM_PTS))
L1_STACK_REFERENCE.extend(generate_L1_stack_poses(AT_Coordinates.IJ, NUM_PTS))
L1_STACK_REFERENCE.extend(generate_L1_stack_poses(AT_Coordinates.KL, NUM_PTS))
print(len(L1_STACK_REFERENCE))


# Wheel Offsets
FL_OFFSET = -144.714
FR_OFFSET = 127.590
BL_OFFSET = 122.341
BR_OFFSET = -146.108

# Controller
DEADBAND = 0.15**2
XBOX_USB_PORT = 0
XBOX_USB_PORT_AUX = 1

if RobotBase.isReal():
    MAX_LINEAR_SPEED = 10  # meters per sec
else:
    MAX_LINEAR_SPEED = 4
MAX_LINEAR_ACCELERATION = 4
# Meters per second squared

MAX_ROTATION_SPEED = 7  # radians per sec
MAX_ROTATION_ACCELERATION = 1 / 2  # Radians per second squared

MAX_SINGLE_SWERVE_ROTATION_SPEED = 9999  # degrees per sec
MAX_SINGLE_SWERVE_ROTATION_ACCELERATION = 9999  # degrees per second squared

# Robot Measurements
WHEEL_RADIUS = 0.08592 / 2  # meters
# Center of wheel to center of wheel across the side of the bot in meters
WHEEL_BASE = 0.584
# Center of wheel to center of wheel across the front of the bot in meters
TRACK_WIDTH = 0.584
TURN_ENCODER_TICKS = 2048  # Ticks per revolution of the angle encoder.
GEAR_RATIO = 1 / 5.63

# Swerve PID constants
DRIVE_P = 2
DRIVE_I = 0
DRIVE_D = 0

TURNING_P = 4.0
TURNING_I = 0.0
TURNING_D = 0.0

# Pigeon
PIGEON_CAN = 5

# Front Left
DRIVE_CAN_FL = 10
STEER_CAN_FL = 15
TURN_ENCODER_ID_FL = 35

# Front Right
DRIVE_CAN_FR = 11
STEER_CAN_FR = 16
TURN_ENCODER_ID_FR = 36

# Back Left
DRIVE_CAN_BL = 12
STEER_CAN_BL = 17
TURN_ENCODER_ID_BL = 37

# Back Right
DRIVE_CAN_BR = 13
STEER_CAN_BR = 18
TURN_ENCODER_ID_BR = 38

# Elevator
LEFT_ELEVATOR_ID = 20
RIGHT_ELEVATOR_ID = 21

# Climber
CLIMB_MOTOR_ID = 22

# Arm
ARM_ROLLERS_ID = 24
ARM_PIVOT_ID = 25

# End Effector

END_EFFECTOR_ID = 26
CORAL_ENTRY_ID = 30
CORAL_EXIT_ID = 31

# LED
LED_CAN = 60  # TODO change this to 7 once LED is updated
