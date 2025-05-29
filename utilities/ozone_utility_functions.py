import math

import wpimath
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpilib import DriverStation, RobotBase

import utilities.constants as constants


def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Clamps a value within a specified range.

    Args:
        value (float): The input value.
        min_val (float): The minimum allowed value.
        max_val (float): The maximum allowed value.

    Returns:
        float: The clamped value.
    """
    return min(max_val, max(min_val, value))


def convert_rpm_to_mps(RPM: float) -> float:
    """
    Converts wheel speed from RPM to meters per second.

    Args:
        RPM (float): Wheel speed in revolutions per minute.

    Returns:
        float: Speed in meters per second.
    """
    return (RPM/60)*(2*math.pi*constants.WHEEL_RADIUS)


def convert_ticks_to_degrees(tick_count: int) -> float:
    """
    Converts encoder tick counts to degrees of rotation.

    This function takes the tick count from an encoder and converts it to degrees.
    The conversion accounts for the total number of ticks per full rotation as defined
    in the constants. The result is negated to ensure the correct rotation direction.

    Args:
        tick_count (int): The number of ticks recorded by the encoder.

    Returns:
        float: The corresponding rotation in degrees.
    """
    return -math.degrees(tick_count * (2 * math.pi) / constants.TURN_ENCODER_TICKS)


def convert_ticks_to_radians(tick_count: int) -> float:
    """
    Converts encoder tick count to radians.

    The function calculates the corresponding angle in radians based on the 
    given tick count. The negative sign is applied to correct the rotation direction.

    Args:
        tick_count (int): The number of encoder ticks.

    Returns:
        float: The equivalent angle in radians.
    """
    return -(tick_count * (2 * math.pi) / constants.TURN_ENCODER_TICKS)


def filter_input(controller_input: float, apply_deadband: bool = True) -> float:
    """
    Filters the controller input by applying a squared scaling and an optional deadband.

    This function squares the input while preserving its sign to provide finer control 
    at lower values. If `apply_deadband` is True, it applies a deadband to ignore small 
    inputs that may result from controller drift.

    Args:
        controller_input (float): The raw input from the controller, ranging from -1 to 1.
        apply_deadband (bool, optional): Whether to apply a deadband to the input. Defaults to True.

    Returns:
        float: The filtered controller input.
    """
    controller_input_corrected = math.copysign(
        math.pow(controller_input, 2), controller_input)

    if apply_deadband:
        return wpimath.applyDeadband(controller_input_corrected, constants.DEADBAND)
    else:
        return controller_input_corrected


def inputModulus(input: float, min: float, max: float) -> float:
    """
    Wraps an input value into a specified range using modulus arithmetic.

    Args:
        input (float): The value to be wrapped.
        min (float): The minimum of the range.
        max (float): The maximum of the range.

    Returns:
        float: The wrapped value within the range [min, max].
    """
    modulus = max - min
    return ((input - min) % modulus) + min


def red_to_blue(position: Translation2d) -> Translation2d:
    """
    Converts a field position from the red alliance perspective to the blue alliance perspective.

    Args:
        position (Translation2d): The position relative to the red alliance.

    Returns:
        Translation2d: The equivalent position relative to the blue alliance.
    """
    return Translation2d(-position.X()+constants.FIELD_LENGTH/2, -position.Y()+constants.FIELD_WIDTH/2)

def swap_pose(pose: Pose2d) -> Pose2d:
    """
    Swaps the alliance color of a pose.

    Args:
        pose (Pose2d): The pose to be swapped.

    Returns:
        Pose2d: The swapped pose.
    """
    return Pose2d(red_to_blue(pose.translation()), -pose.rotation())


def RPM_to_RadiansPerSecond(RPM: float) -> float:
    """
    Converts a speed from RPM (revolutions per minute) to radians per second.

    Args:
        RPM (float): Speed in revolutions per minute.

    Returns:
        float: Speed in radians per second.
    """
    return (RPM / 60) * (2 * math.pi)


def within_pose_tolerance(p1: Pose2d, p2: Pose2d, translation_tol: float, rotation_tol: float) -> bool:
    """
    Checks whether two poses are within a given translation and rotation tolerance.

    This function determines if the translation and rotation differences between 
    two poses are within the specified tolerances.

    Args:
        p1 (Pose2d): The first pose.
        p2 (Pose2d): The second pose.
        translation_tol (float): The allowed tolerance for translation (distance).
        rotation_tol (float): The allowed tolerance for rotation in degrees (angle).

    Returns:
        bool: True if both translation and rotation differences are within tolerance, False otherwise.
    """
    t = within_translation_tolerance(
        p1.translation(), p2.translation(), translation_tol)
    r = within_rotation_tolerance(p1.rotation(), p2.rotation(), rotation_tol)
    return t and r


def within_translation_tolerance(t1: Translation2d, t2: Translation2d, tolerance: float):
    """
    Checks whether two translations are within a given tolerance.

    Params:
        t1 (Translation2d): The first translation.
        t2 (Translation2d): The second translation.
        tolerance (float): The allowed tolerance for the translation in meters

    Returns
        bool: True if the translations are within the desired tolerance of one another
    """
    return t1.distance(t2) <= tolerance


def within_rotation_tolerance(r1: Rotation2d, r2: Rotation2d, tolerance: float) -> bool:
    """
    Checks whether two rotations are within a given tolerance.

    Params:
        r1 (Rotation2d): The first rotation.
        r2 (Rotation2d): The second rotation.
        tolerance (float): The allowed tolerance in degrees allowed between the two rotations.

    Returns:
        bool: Whether the rotations are within the given tolerance.
    """
    error = abs((r1 - r2).radians())
    return error <= math.radians(tolerance)
