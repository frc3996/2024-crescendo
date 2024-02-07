import math
from typing import Any, Tuple

import wpilib
from ntcore import NetworkTableInstance
from wpimath.filter import MedianFilter
from wpimath.geometry import Pose3d, Rotation3d, Translation3d

from common.tools import map_value
from components import field
from components.swervedrive import SwerveDrive


class LimeLightVision:
    drivetrain: SwerveDrive

    def __init__(self, name="limelight"):
        self.nt = NetworkTableInstance.getDefault().getTable(name)

        # Register to the limelight topics
        self.tclass = self.nt.getStringTopic("tclass").subscribe("None")
        self.ta = self.nt.getFloatTopic("ta").subscribe(0)
        self.tx = self.nt.getFloatTopic("tx").subscribe(-999)
        self.tv = self.nt.getIntegerTopic("tv").subscribe(0)
        self.pipe = self.nt.getIntegerTopic("getpipe").subscribe(-1)
        self.cl = self.nt.getFloatTopic("cl").subscribe(0)
        self.tl = self.nt.getFloatTopic("tl").subscribe(0)
        self.tid = self.nt.getIntegerTopic("tid").subscribe(0)

        self.botpose = self.nt.getFloatArrayTopic("botpose").subscribe(
            [-99, -99, -99, 0, 0, 0, -1]
        )
        self.botpose_wpiblue = self.nt.getFloatArrayTopic("botpose_wpiblue").subscribe(
            [-99, -99, -99, 0, 0, 0, -1]
        )
        self.botpose_wpired = self.nt.getFloatArrayTopic("botpose_wpired").subscribe(
            [-99, -99, -99, 0, 0, 0, -1]
        )

        # create the timer that we can use to the the FPGA timestamp
        self.timer = wpilib.Timer()

        self.fieldLayout = field.FieldLayout()

        # And a bunch of filters
        self.poseXFilter = MedianFilter(8)
        self.poseYFilter = MedianFilter(8)
        self.poseZFilter = MedianFilter(8)
        self.poseYawFilter = MedianFilter(8)
        self.__last_update = 0

    def get_pose(self) -> Tuple[Pose3d, Any] | None:
        pose3d = self.botpose_to_pose3d(self.botpose.get())
        if pose3d is None:
            return None
        return (*pose3d,)

    def get_alliance_pose(self) -> Tuple[Pose3d, Any] | None:
        if self.fieldLayout.alliance == field.RED_ALLIANCE:
            pose3d = self.botpose_to_pose3d(self.botpose_wpired.get())
        elif self.fieldLayout.alliance == field.BLUE_ALLIANCE:
            pose3d = self.botpose_to_pose3d(self.botpose_wpiblue.get())
        else:
            raise RuntimeError("No alliance set")
        if pose3d is None:
            return None
        return (*pose3d,)

    def botpose_to_pose3d(self, poseArray) -> Tuple[Pose3d, Any] | None:
        """Takes limelight array data and creates a Pose3d object for
           robot position and a timestamp reprepresenting the time
           the position was observed.

        Args:
            poseArray (_type_): An array from the limelight network tables.

        Returns:
            Tuple[Pose3d, Any]: Returns vision Pose3d and timestamp.
        """
        pX, pY, pZ, pRoll, pPitch, pYaw, msLatency = poseArray
        if msLatency == -1:
            return None
        else:
            return Pose3d(
                Translation3d(pX, pY, pZ), Rotation3d.fromDegrees(pRoll, pPitch, pYaw)
            ), self.timer.getFPGATimestamp() - (msLatency / 1000)

    def execute(self) -> None:
        # Add vision pose measurements
        vision_pose = self.get_alliance_pose()
        if (
            vision_pose is None
            or vision_pose[0].x == 0
            or vision_pose[0].y == 0
            or vision_pose[1] - self.__last_update > 500
        ):
            # If it's been more than 500ms since the last update, nuke the filter
            # TODO: This is unused for now, instead try to rely on std deviation
            self.poseXFilter.reset()
            self.poseYFilter.reset()
            self.poseZFilter.reset()
            self.poseYawFilter.reset()
        if vision_pose and vision_pose[0].x > 0 and vision_pose[0].y > 0:
            """
            To promote stability of the pose estimate and make it robust to bad vision
            data, we recommend only adding vision measurements that are already within
            one meter or so of the current pose estimate.

            Note that the vision measurement standard deviations passed into this
            method will continue to apply to future measurements until a subsequent
            call to SetVisionMeasurementStdDevs() or this method.
            """

            # Increase standard deviation with distance form tag
            transform = self.fieldLayout.getTagRelativePosition(self.tid.get())
            if transform is None:
                return
            # This means if we're 2m from the target, the std dev is 0.3, but
            # over 8m we're at 2.0m
            stddevupdate = map_value(
                math.sqrt(transform.x**2 + transform.y**2), 0, 10, 0.2, 3.0
            )
            stddevupdate_rot = map_value(
                math.sqrt(transform.x**2 + transform.y**2),
                0,
                math.tau,
                math.tau / 16,
                math.tau / 2,
            )
            self.drivetrain.odometry.addVisionMeasurement(
                vision_pose[0].toPose2d(),
                vision_pose[1],
                (stddevupdate, stddevupdate, stddevupdate_rot),
            )

    # def set_pipeline(self, value: int):
    #     self.nt.putNumber("pipeline", value)

    # def get_latency(self):
    #     return (self.cl.get() + self.tl.get()) / 1000
    #
    # def get_pipeline(self):
    #     return self.pipe.get()

    # def get_target(self):
    #     if self.Tv.get():
    #         self.tClass = self.Tclass.get()
    #         self.ta = self.Ta.get()
    #         self.tx = self.Tx.get()
    #         self.tv = self.Tv.get()
    #         self.drive_vRotate = self.calculate_rotation(self.tx)
    #         self.drive_vX = self.calculate_x(self.ta)
    #         self.drive_vY = 0
    #     else:
    #         self.tClass = self.ta = self.tx = self.tv = None
    #         self.drive_vRotate = self.drive_vX = self.drive_vY = 0
    #         # self.txFilter.reset()
    #         # self.taFilter.reset()
    #
    # def calculate_x(self, targetArea):
    #     """Calculate X robot-oriented speed from the size of the target.  Return is inverted
    #     since we need the robot to drive backwards toward the target to pick it up.
    #
    #     Args:
    #         targetArea (Float):  The target area determined by limelight.
    #
    #     Returns:
    #         Float: Velocity in the X direction (robot oriented)
    #     """
    #     return min(-0.20, -(targetArea * -0.0125 + 1.3125))
    #     # calcX = -(-0.0002*(targetArea**2) + 0.0093*targetArea+1)
    #     # return max(-1, calcX)
    #
    # def calculate_rotation(self, targetX):
    #     """Calculate the rotational speed from the X value of the target in the camera frame.
    #     Return is inverted to make left rotation positive from a negative X value, meaning the
    #     target is to the left of center of the camera's view.
    #
    #     Args:
    #         targetX (Float): The X value of the target in the camera frame, 0 is straight ahead,
    #         to the left is negative, to the right is positive.
    #
    #     Returns:
    #         Float: Rotational velocity with CCW (left, robot oriented) positive.
    #     """
    #     return -(targetX / 25)
    #
    # def get_velocity(self):
    #     """Get calculated velocities from vision target data
    #
    #     Returns:
    #         Tuple(vX, vY, vT): X, Y, and rotation velocities as a tuple.
    #     """
    #     return (self.drive_vX, self.drive_vY, self.drive_vRotate)
    #
    # def has_target(self):
    #     return self.tv
    #
    # def is_target_ready(self, acceptable_offset=2):
    #     if self.has_target():
    #         return False
    #
    #     if abs(self.Tx.get()) < acceptable_offset:
    #         return True
    #     return False
