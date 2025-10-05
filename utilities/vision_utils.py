from dataclasses import dataclass

from ntcore import NetworkTableInstance, Value
from wpimath.geometry import Pose2d, Rotation2d
from wpilib import DriverStation

import utilities.constants as constants


@dataclass
class RawFiducial:
    id: int = 0
    txyc: float = 0
    tync: float = 0
    ta: float = 0
    dist_to_camera: float = 0
    dist_to_robot: float = 0
    ambiguity: float = 0


@dataclass
class PoseEstimate:
    pose: Pose2d
    timestamp_seconds: float
    latency: float
    tag_count: int
    tag_span: float
    avg_tag_dist: float
    avg_tag_area: float
    raw_fiducials: list[RawFiducial]


@dataclass
class VisionMeasurement:
    pose: Pose2d
    timestamp: float
    xy_std: float
    rot_std: float


FIELD_TOLERANCE = 0.3
Z_TOLERANCE = 1


def get_botpose_estimate(botpose: list[float], time: float) -> PoseEstimate:
    """
    Gets the pose estimate from a NetworkTables entry.

    Params:
        botpose (list[float]): The given botpose NetworkTables entry
        time (float): The timestamp attached to the given entry

    Returns:
        PoseEstimate: A PoseEstimate object from the given NetworkTables entry
    """
    pose = Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]))
    latency = botpose[6]
    tag_count = int(botpose[7])
    tag_span = botpose[8]
    tag_dist = botpose[9]
    tag_area = botpose[10]
    timestamp = time / 1e6 - latency / 1000

    raw_fiducials = []

    if len(botpose) == 11 + 7 * tag_count:
        for i in range(tag_count):
            index = 11 + 7 * i
            id = int(botpose[index])
            txnc = botpose[index + 1]
            tync = botpose[index + 2]
            ta = botpose[index + 3]
            dist_to_camera = botpose[index + 4]
            dist_to_robot = botpose[index + 5]
            ambiguity = botpose[index + 6]
            raw_fiducials.append(
                RawFiducial(
                    id, txnc, tync, ta, dist_to_camera, dist_to_robot, ambiguity
                )
            )
    return PoseEstimate(
        pose, timestamp, latency, tag_count, tag_span, tag_dist, tag_area, raw_fiducials
    )


def print_PoseEstimate(pose_estimate: PoseEstimate) -> None:
    """
    Prints the given pose estimate for debugging/identification purposes

    Params:
        pose_estimate (PoseEstimate): A PoseEstimate object to be printed
    """
    if pose_estimate is None:
        print("No PoseEstimate available.")
        return

    print("Pose Estimate Information:")
    print(f"Timestamp (Seconds): {pose_estimate.timestamp_seconds}")
    print(f"Latency: {pose_estimate.latency}")
    print(f"Tag Count: {pose_estimate.tag_count}")
    print(f"Tag Span: {pose_estimate.tag_span} meters")
    print(f"Average Tag Distance: {pose_estimate.avg_tag_dist} meters")
    print(f"Average Tag Area: {pose_estimate.avg_tag_area} of image")
    print()

    if pose_estimate.raw_fiducials is None or len(pose_estimate.raw_fiducials) == 0:
        print("No RawFiducials data available.")
        return

    print("Raw Fiducials Details:")
    for i in range(len(pose_estimate.raw_fiducials)):
        fiducial = pose_estimate.raw_fiducials[i]
        print(f" Fiducial #{i + 1}:")
        print(f" ID: {fiducial.id}")
        print(f" TXNC: {fiducial.txyc}")
        print(f" TYNC: {fiducial.tync}")
        print(f" TA: {fiducial.ta}")
        print(f" Distance to Camera: {fiducial.dist_to_camera} meters")
        print(f" Distance to Robot: {fiducial.dist_to_robot} meters")
        print(f" Ambiguity: {fiducial.ambiguity}")
        print()


def get_std_deviations(estimate: PoseEstimate) -> tuple[float, float]:
    """
    Gets the estimated standard deviations of the given PoseEstimate

    Params:
        estimate (PoseEstimate): The PoseEstimate to find the standard deviations of

    Returns:
        tuple[float, float]: The standard deviations in meters in the xy direction and radians for rotational
    """
    if estimate.tag_count != 0:
        xy = 5 / ((estimate.tag_count * estimate.avg_tag_area) ** 1.5 * 184.76 + 148.41)
        return (xy, 0.2)
    return (10, 10)


def get_queue(limelight_name: str, table_type: str) -> list[Value]:
    """
    Gets the queue of recent measurements from a limelight

    Params:
        limelight_name (str): The limelight to get the measurements from
        table_type  (str): The type of measurement to take from the limelight

    Returns:
        list[Value]: A list of recent measurements and their timestamps
    """
    return (
        NetworkTableInstance.getDefault()
        .getTable(limelight_name)
        .getEntry(table_type)
        .readQueue()
    )


def is_on_field(pose: Pose2d) -> bool:
    """
    Gets whether the given pose is on the field for filtering purposes.

    Params:
        pose (Pose2d): The pose to be measured

    Returns:
        bool: Whether the given pose is on the field
    """
    return (
        pose.X() + FIELD_TOLERANCE > 0
        and pose.X() - FIELD_TOLERANCE < constants.FIELD_LENGTH
        and pose.Y() + FIELD_TOLERANCE > 0
        and pose.Y() - FIELD_TOLERANCE < constants.FIELD_WIDTH
    )


def is_non_zero(pose: Pose2d) -> bool:
    """
    Gets whether the given pose is a real measurement by checking if it is not at the origin. Used for filtering purposes.

    Params:
        pose (Pose2d): The pose to be measured

    Returns:
        bool: Whether the given pose is away from the origin
    """
    return pose.translation().norm() > 1e-7


def get_recent_vision_measurements(
    limelight_name: str, table_type: str
) -> list[VisionMeasurement]:
    """
    Gets a list of all vision measurements since the last time this method was called

    Params:
        limelight_name(str): Name of the limelight to get measurements from
        table_type (str): Which NetworkTable to grab information from (i.e. "botpose-orb-wpiblue")

    Returns:
        list[VisionMeasurement]: A list containing all recent measurements as a VisionMeasurement object"""
    measurements = get_queue(limelight_name, table_type)
    if len(measurements) > 10:
        return []
    readings = []
    for measurement in measurements:
        if measurement.isValid() and len(measurement.value()) >= 10:
            time = measurement.time()
            pose_array = measurement.value()
            z = pose_array[2]
            pose_estimate = get_botpose_estimate(pose_array, time)

            pose = pose_estimate.pose
            timestamp = pose_estimate.timestamp_seconds
            (xy_std, rot_std) = get_std_deviations(pose_estimate)

            if is_on_field(pose) and abs(z) < Z_TOLERANCE and is_non_zero(pose):
                readings.append(VisionMeasurement(pose, timestamp, xy_std, rot_std))

    return readings


def set_robot_orientation(
    limelight_name: str, yaw_degrees: float, yaw_rate: float
) -> None:
    """
    Sends the current drivetrain yaw to the limelight to help calculate for MegaTag2 estimation

    Params:
        limelight_name (str): Name of the limelight to send yaw to
        yaw_degrees (float): Current rotation of the bot in degrees
        yaw_rate (float): Rate at which the bot is turning in dps
    """
    if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
        NetworkTableInstance.getDefault().getTable(limelight_name).getEntry(
            "robot_orientation_set"
        ).setDoubleArray([yaw_degrees, yaw_rate, 0, 0, 0, 0])
    else:
        NetworkTableInstance.getDefault().getTable(limelight_name).getEntry(
            "robot_orientation_set"
        ).setDoubleArray([(yaw_degrees + 180) % 360, yaw_rate, 0, 0, 0, 0])
