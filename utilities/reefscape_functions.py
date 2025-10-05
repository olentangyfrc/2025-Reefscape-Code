import math

from wpimath.geometry import Rotation2d, Translation2d, Pose2d, Transform2d

import utilities.constants as constants


def angle_to_reef(position: Translation2d) -> Rotation2d:
    """
    Computes the angle from the given position to the reef location on the field.

    Args:
        position (Translation2d): The robot's current position.

    Returns:
        Rotation2d: The angle to the reef in field coordinates.
    """
    reef_pos = Translation2d(4.489, 4.0259)
    diff = reef_pos - position

    return Rotation2d(math.atan2(diff.Y(), diff.X()))


def nearest_reef_branch(position: Translation2d) -> str:
    """
    Determines the target reef point based on the robot's field position.

    Args:
        position (Translation2d): The robot's position on the field.

    Returns:
        str: The designated reef point identifier ("a", "i", "k", etc.) or None if undefined.
    """
    center = Translation2d(4.489, 4.0259)
    blue_position = position

    diff = blue_position - center
    theta = (math.atan2(diff.Y(), diff.X()) * 180 / math.pi) % 360

    if theta > 330:
        return "g"
    elif theta < 30:
        return "h"
    elif theta < 60:
        return "i"
    elif theta < 90:
        return "j"
    elif theta < 120:
        return "k"
    elif theta < 150:
        return "l"
    elif theta < 180:
        return "a"
    elif theta < 210:
        return "b"
    elif theta < 240:
        return "c"
    elif theta < 270:
        return "d"
    elif theta < 300:
        return "e"
    elif theta <= 330:
        return "f"
    return None


def can_line_up(
    bot_loc: Translation2d, reef_position: str, tolerance: float = 0.43
) -> bool:
    """
    Gets whether the bot can line up to the given reef position without hitting the reef

    Params:
        bot_loc (Translation2d): Bot's current translation
        reef_position (str): Set reef_position to line up to
        tolerance (float): Distance in meters the bot needs to be from the reef

    Returns:
        bool: Whether the bot can safely line up
    """
    tag_pose = Pose2d()
    should_pose_be_above_tag = False
    match reef_position:
        case "a" | "A" | "b" | "B":
            tag_pose = constants.AT_Coordinates.AB.value
            should_pose_be_above_tag = False
        case "c" | "C" | "d" | "D":
            tag_pose = constants.AT_Coordinates.CD.value
            should_pose_be_above_tag = False
        case "e" | "E" | "f" | "F":
            tag_pose = constants.AT_Coordinates.EF.value
            should_pose_be_above_tag = True
        case "g" | "G" | "h" | "H":
            tag_pose = constants.AT_Coordinates.GH.value
            should_pose_be_above_tag = True
        case "i" | "I" | "j" | "J":
            tag_pose = constants.AT_Coordinates.IJ.value
            should_pose_be_above_tag = True
        case "k" | "K" | "l" | "L":
            tag_pose = constants.AT_Coordinates.KL.value
            should_pose_be_above_tag = False

    spacial_pose = tag_pose.transformBy(Transform2d(-tolerance, 0, 0))
    slope = -math.tan(tag_pose.rotation().radians())
    # y-y1 = m(x-x1)
    is_pose_above_tag = (
        bot_loc.X() > slope * (bot_loc.Y() - spacial_pose.Y()) + spacial_pose.X()
    )
    return is_pose_above_tag == should_pose_be_above_tag


def get_algae_loc(bot: str) -> Pose2d:
    tag_pose = Pose2d()
    match bot:
        case "a" | "A" | "b" | "B":
            tag_pose = constants.AT_Coordinates.AB.value
        case "c" | "C" | "d" | "D":
            tag_pose = constants.AT_Coordinates.CD.value
        case "e" | "E" | "f" | "F":
            tag_pose = constants.AT_Coordinates.EF.value
        case "g" | "G" | "h" | "H":
            tag_pose = constants.AT_Coordinates.GH.value
        case "i" | "I" | "j" | "J":
            tag_pose = constants.AT_Coordinates.IJ.value
        case "k" | "K" | "l" | "L":
            tag_pose = constants.AT_Coordinates.KL.value

    return tag_pose.transformBy(Transform2d(-0.74, 0, 0))


def left_reef_zone(bot: Translation2d, tolerance: float = 0.45) -> bool:
    """
    Gets whether the bot is within a certain distance of the center of the reef

    Params:
        bot (Translation2d): Bot's position
        tolerance (float): Distance the bot can be within the reef zone

    Returns:
        bool: Whether the bot has left this zone
    """
    return bot.distance(Translation2d(4.489, 4.0259)) >= 1.35 + tolerance
