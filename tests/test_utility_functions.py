import math
import pytest
import utilities.ozone_utility_functions as utility_functions
import utilities.constants as constants
from wpimath.geometry import Pose2d, Rotation2d, Translation2d


@pytest.mark.parametrize(
    "tick_count, expected",
    [
        (0, 0),
        (constants.TURN_ENCODER_TICKS, -360),
        (-constants.TURN_ENCODER_TICKS, 360),
        (constants.TURN_ENCODER_TICKS / 2, -180),
        (-constants.TURN_ENCODER_TICKS / 2, 180),
    ],
)
def test_convert_ticks_to_degrees(tick_count: int, expected: float) -> None:
    assert utility_functions.convert_ticks_to_degrees(tick_count) == pytest.approx(
        expected
    )


@pytest.mark.parametrize(
    "tick_count, expected",
    [
        (0, 0),
        (constants.TURN_ENCODER_TICKS, -2 * math.pi),
        (-constants.TURN_ENCODER_TICKS, 2 * math.pi),
        (constants.TURN_ENCODER_TICKS / 2, -math.pi),
        (-constants.TURN_ENCODER_TICKS / 2, math.pi),
    ],
)
def test_convert_ticks_to_radians(tick_count: int, expected: float) -> None:
    assert utility_functions.convert_ticks_to_radians(tick_count) == pytest.approx(
        expected
    )


@pytest.mark.parametrize(
    "controller_input",
    [
        2,
        1,
        1.5 * constants.DEADBAND,
        1.1 * constants.DEADBAND,
        constants.DEADBAND,
        0.8 * constants.DEADBAND,
        0.1 * constants.DEADBAND,
        0 - 0.1 * constants.DEADBAND,
        -0.8 * constants.DEADBAND,
        -1 * constants.DEADBAND,
        -1.1 * constants.DEADBAND,
        -1.5 * constants.DEADBAND,
        -1,
        -2,
    ],
)
def test_filter_input(controller_input: float) -> None:
    if abs(controller_input) > 1:
        utility_functions.filter_input(controller_input)
    elif abs(math.pow(controller_input, 2)) > constants.DEADBAND:
        assert utility_functions.filter_input(controller_input) == math.copysign(
            math.pow(controller_input, 2), controller_input
        )
    elif abs(math.pow(controller_input, 2)) <= constants.DEADBAND:
        assert utility_functions.filter_input(controller_input) == 0


@pytest.mark.parametrize(
    "p1, p2, trans_tol, rot_tol, expected",
    [
        (Pose2d(0, 0, Rotation2d(0)), Pose2d(0, 0, Rotation2d(0)), 1, 10, True),
        (
            Pose2d(0, 0, Rotation2d(math.radians(5))),
            Pose2d(0, 0, Rotation2d(math.radians(0))),
            1,
            10,
            True,
        ),
        (
            Pose2d(0, 0, Rotation2d(math.radians(15))),
            Pose2d(0, 0, Rotation2d(math.radians(0))),
            1,
            10,
            False,
        ),
        (Pose2d(0, 0, Rotation2d(0)), Pose2d(1, 1, Rotation2d(0)), 2, 10, True),
        (Pose2d(0, 0, Rotation2d(0)), Pose2d(1, 1, Rotation2d(0)), 1, 10, False),
    ],
)
def test_within_pose_tolerance(
    p1: Pose2d, p2: Pose2d, trans_tol: float, rot_tol: float, expected: bool
) -> None:
    assert (
        utility_functions.within_pose_tolerance(p1, p2, trans_tol, rot_tol) == expected
    )


@pytest.mark.parametrize(
    "p1,p2,rotation_tol, expected",
    [
        (Rotation2d(math.pi), Rotation2d(math.pi), 10, True),
        (Rotation2d(math.pi), Rotation2d(1.05 * math.pi), 10, True),
    ],
)
def test_within_rotation_tolerance(
    p1: Rotation2d, p2: Rotation2d, rotation_tol: float, expected: bool
) -> None:
    assert utility_functions.within_rotation_tolerance(p1, p2, rotation_tol) == expected


@pytest.mark.parametrize(
    "t1,t2,tolerance, expected",
    [
        (Translation2d(2, 2), Translation2d(2, 2), 0.1, True),
        (Translation2d(2, 2), Translation2d(2, 2.05), 0.1, True),
        (Translation2d(2, 2), Translation2d(2, 1), 0.5, False),
        (Translation2d(10, 3), Translation2d(3, 10), 0.6, False),
    ],
)
def test_within_translation_tolerance(
    t1: Translation2d, t2: Translation2d, tolerance: float, expected: bool
) -> None:
    assert utility_functions.within_translation_tolerance(t1, t2, tolerance) == expected


# def test_RPM_to_RadianPerSecond(RPM):
#     assert utility_functions.RPM_to_RadianPerSecond(RPM) ==


@pytest.mark.parametrize(
    "value, min, max",
    [(5, 2, 10), (2, 5, 14), (10, 3, 7), (4, 4, 6), (6, 4, 8), (0, 0, 0)],
)
def test_clamp(value: float, min: float, max: float) -> None:
    if value < min:
        assert utility_functions.clamp(value, min, max) == min
    elif value > max:
        assert utility_functions.clamp(value, min, max) == max
    elif min <= value <= max:
        assert utility_functions.clamp(value, min, max) == value


@pytest.mark.parametrize(
    "RPM, expected",
    [
        (0, 0),
        (100, (100 * 2 * math.pi * constants.WHEEL_RADIUS) / 60),
        (-100, (-100 * 2 * math.pi * constants.WHEEL_RADIUS) / 60),
        (200, (200 * 2 * math.pi * constants.WHEEL_RADIUS) / 60),
        (-200, (-200 * 2 * math.pi * constants.WHEEL_RADIUS) / 60),
        (300, (300 * 2 * math.pi * constants.WHEEL_RADIUS) / 60),
        (-300, (-300 * 2 * math.pi * constants.WHEEL_RADIUS) / 60),
        (400, (400 * 2 * math.pi * constants.WHEEL_RADIUS) / 60),
        (-400, (-400 * 2 * math.pi * constants.WHEEL_RADIUS) / 60),
        (500, (500 * 2 * math.pi * constants.WHEEL_RADIUS) / 60),
        (-500, (-500 * 2 * math.pi * constants.WHEEL_RADIUS) / 60),
    ],
)
def test_rpm_to_mps(RPM, expected):
    result = utility_functions.convert_rpm_to_mps(RPM)
    assert result == pytest.approx(expected)

    if RPM > 0:
        assert result > 0
    elif RPM < 0:
        assert result < 0
    elif RPM == 0:
        assert result == 0


@pytest.mark.parametrize(
    "input_value, min_value, max_value, expected",
    [
        # Result should always be within min and max
        (5, 0, 10, 5),
        (12, 0, 10, 2),
        (25, 0, 10, 5),
        (-15, 0, 10, 5),
        # If input is already in range, it should remain unchanged
        (3, 0, 10, 3),
        (9.9, 0, 10, 9.9),
        (10, 0, 10, 0),
        (-10, 0, 10, 0),
        (-20, 0, 10, 0),
        (15, -5, 5, -5),
    ],
)
def test_input_modulus(input_value, min_value, max_value, expected):
    assert utility_functions.inputModulus(input_value, min_value, max_value) == expected


FIELD_LENGTH = constants.FIELD_LENGTH
FIELD_WIDTH = constants.FIELD_WIDTH


@pytest.mark.parametrize(
    "red_position, expected_blue_position",
    [
        (Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2), Translation2d(0, 0)),
        (Translation2d(0, 0), Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2)),
        (
            Translation2d(FIELD_LENGTH, FIELD_WIDTH),
            Translation2d(-FIELD_LENGTH / 2, -FIELD_WIDTH / 2),
        ),
        (
            Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH),
            Translation2d(0, -FIELD_WIDTH / 2),
        ),
        (
            Translation2d(FIELD_LENGTH / 4, FIELD_WIDTH / 4),
            Translation2d(
                -FIELD_LENGTH / 4 + FIELD_LENGTH / 2, -FIELD_WIDTH / 4 + FIELD_WIDTH / 2
            ),
        ),
        (
            Translation2d(FIELD_LENGTH * 0.75, FIELD_WIDTH * 0.75),
            Translation2d(
                -FIELD_LENGTH * 0.75 + FIELD_LENGTH / 2,
                -FIELD_WIDTH * 0.75 + FIELD_WIDTH / 2,
            ),
        ),
    ],
)
def test_red_to_blue(red_position, expected_blue_position):
    blue_position = utility_functions.red_to_blue(red_position)
    assert blue_position.X() == expected_blue_position.X(), (
        f"Expected X to be {expected_blue_position.X()}, but got {blue_position.X()}"
    )
    assert blue_position.Y() == expected_blue_position.Y(), (
        f"Expected Y to be {expected_blue_position.Y()}, but got {blue_position.Y()}"
    )
