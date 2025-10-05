import math

from ntcore import NetworkTableInstance, NetworkTable
from wpilib import Timer, Field2d, DriverStation
from components.chassis import drivetrain
from wpimath.geometry import Pose2d

from wpilib.shuffleboard import Shuffleboard

from magicbot import feedback, tunable

from utilities.vision_utils import (
    get_recent_vision_measurements,
    set_robot_orientation,
    VisionMeasurement,
)


CAMERA_NAMES = ["limelight-left", "limelight-right"]


class Vision:
    instance = None

    left_enable = tunable(True)
    right_enable = tunable(True)
    send_yaw_rate = tunable(True)

    def __init__(self) -> None:
        """
        Initializes the vision subsystem by creating a networktables instance with all given limelights.
        """
        Vision.instance = self

        self.cameraName = "limelight"
        instance = NetworkTableInstance.getDefault()

        self.tables: list[NetworkTable] = [
            instance.getTable(name) for name in CAMERA_NAMES
        ]

        self.measurement_tracker: dict[str, list[VisionMeasurement]] = {
            name: [] for name in CAMERA_NAMES
        }

        self.vision_field = Field2d()
        self.vision_field.setRobotPose(Pose2d())
        Shuffleboard.getTab("Teleoperated").add("Vision Field", self.vision_field)

        # Filler object so sendables get sent through
        Shuffleboard.getTab("Teleoperated").add("Filler2", 0.0)

    def execute(self) -> None:
        """
        Run every auton and teleop periodic cycle. Feeds vision measurements to the drivetrain to update odometry
        """

        for camera_name in CAMERA_NAMES:
            if not self.left_enable and camera_name == "limelight-left":
                continue
            if not self.right_enable and camera_name == "limelight-right":
                continue

            nt_id = "botpose_orb_wpiblue"
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                nt_id = "botpose_orb_wpired"

            set_robot_orientation(
                camera_name,
                drivetrain.Drivetrain.instance.get_gyro_yaw_value().degrees(),
                drivetrain.Drivetrain.instance.get_yaw_rate() * self.send_yaw_rate,
            )
            recent_measurements = get_recent_vision_measurements(camera_name, nt_id)
            for measurement in recent_measurements:
                self.measurement_tracker[camera_name].append(measurement)
                drivetrain.Drivetrain.instance.add_vision_measurement(
                    measurement.pose, measurement.timestamp, measurement.xy_std
                )
                self.vision_field.setRobotPose(
                    Pose2d(
                        measurement.pose.X(),
                        measurement.pose.Y(),
                        measurement.pose.rotation(),
                    )
                )

            for i in self.measurement_tracker[camera_name]:
                if i.timestamp + 1.0 < Timer.getFPGATimestamp():
                    self.measurement_tracker[camera_name].pop(
                        self.measurement_tracker[camera_name].index(i)
                    )

    def recent_pose(self, limelight_name: str) -> Pose2d:
        """
        Gets the most recent vision measurement for the given limelight name

        Args:
            limelight_name (str): Name of the limelight to measure from

        Returns:
            Pose2d: The most recent measured pose from the given limelight
        """
        if (
            limelight_name in self.measurement_tracker.keys()
            and len(self.measurement_tracker[limelight_name]) != 0
        ):
            return self.measurement_tracker[limelight_name][-1].pose
        return Pose2d()

    def get_recent_fps(self, limelight_name: str) -> int:
        """
        Gets the number of valid, processed vision measurements from the specific limelight in the last second

        Args:
            limelight_name (str): Name of the limelight to measure from

        Returns:
            int: The number of frames measured from in the last second
        """
        if (
            limelight_name in self.measurement_tracker.keys()
            and len(self.measurement_tracker[limelight_name]) != 0
        ):
            return len(self.measurement_tracker[limelight_name])
        return 0

    @feedback
    def get_left_pose(self) -> Pose2d:
        return self.recent_pose("limelight-left")

    @feedback
    def get_right_pose(self) -> Pose2d:
        return self.recent_pose("limelight-right")

    @feedback
    def get_avg_pose(self) -> Pose2d:
        """
        Gets the average pose reading for all cameras on the bot

        Returns:
            Pose2d: Pose2d object representing the average camera pose reading
        """

        num_cameras = len(CAMERA_NAMES)

        x = 0.0
        y = 0.0
        sin_sum = 0.0
        cos_sum = 0.0

        recent_poses = [self.recent_pose(name) for name in CAMERA_NAMES]

        for pose in recent_poses:
            x += pose.X()
            y += pose.Y()

            angle = pose.rotation().radians()
            sin_sum += math.sin(angle)
            cos_sum += math.cos(angle)

        avg_x = x / num_cameras
        avg_y = y / num_cameras

        avg_rot = math.atan2(sin_sum / num_cameras, cos_sum / num_cameras)

        return Pose2d(avg_x, avg_y, avg_rot)

    def get_avg_pose_x(self) -> float:
        """
        Returns the x component of the average pose determined by vision

        Returns:
            float: the average x in meters that the bot's vision returns
        """
        return self.get_avg_pose().X()

    def get_avg_pose_y(self) -> float:
        """
        Returns the y component of the average pose determined by vision

        Returns:
            float: the average y in meters that the bot's vision returns
        """
        return self.get_avg_pose().Y()

    @feedback
    def get_avg_pose_rot(self) -> float:
        """
        Returns the y component of the average pose determined by vision

        Returns:
            float: the average rotations in degrees that the bot's vision returns
        """
        return self.get_avg_pose().rotation().degrees()

    def get_avg_fps(self) -> float:
        """
        Returns the average frames per second of all cameras on the bot

        Returns:
            float: the average number of frames measured from in the last second
        """
        return round(sum(map(self.get_recent_fps, CAMERA_NAMES)) / len(CAMERA_NAMES), 2)
