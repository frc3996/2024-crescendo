from typing import Any, Tuple

import wpilib
from ntcore import NetworkTableInstance
from wpimath.filter import MedianFilter
from wpimath.geometry import Pose3d, Rotation3d, Translation3d

from .field import BLUE_ALLIANCE, RED_ALLIANCE, FieldLayout


class LimeLightVision:
    def __init__(self, name="limelight"):
        self.nt = NetworkTableInstance.getDefault().getTable(name)

        self.Tclass = self.nt.getStringTopic("tclass").subscribe("None")
        self.Ta = self.nt.getFloatTopic("ta").subscribe(0)
        self.Tx = self.nt.getFloatTopic("tx").subscribe(-999)
        self.Tv = self.nt.getIntegerTopic("tv").subscribe(0)
        self.Pipe = self.nt.getIntegerTopic("getpipe").subscribe(-1)
        self.Cl = self.nt.getFloatTopic("cl").subscribe(0)
        self.Tl = self.nt.getFloatTopic("tl").subscribe(0)

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

        self.fieldLayout = FieldLayout()

    def get_latency(self):
        return (self.Cl.get() + self.Tl.get()) / 1000

    def get_pipeline(self):
        return self.Pipe.get()

    def get_pose(self) -> Tuple[Pose3d, Any]:
        return (*self.botpose_to_pose3d(self.botpose.get()),)

    def get_alliance_pose(self) -> Tuple[Pose3d, Any]:
        if self.fieldLayout.alliance == RED_ALLIANCE:
            return (*self.botpose_to_pose3d(self.botpose_wpired.get()),)
        elif self.fieldLayout.alliance == BLUE_ALLIANCE:
            return (*self.botpose_to_pose3d(self.botpose_wpiblue.get()),)
        else:
            raise RuntimeError("No alliance set")

    def get_tid(self) -> float:
        return self.nt.getNumber("tid", -1.0)

    def get_target(self):
        if self.Tv.get():
            self.tClass = self.Tclass.get()
            self.ta = self.Ta.get()
            self.tx = self.Tx.get()
            self.tv = self.Tv.get()
            self.drive_vRotate = self.calculate_rotation(self.tx)
            self.drive_vX = self.calculate_x(self.ta)
            self.drive_vY = 0
        else:
            self.tClass = self.ta = self.tx = self.tv = None
            self.drive_vRotate = self.drive_vX = self.drive_vY = 0
            # self.txFilter.reset()
            # self.taFilter.reset()

    def calculate_x(self, targetArea):
        """Calculate X robot-oriented speed from the size of the target.  Return is inverted
        since we need the robot to drive backwards toward the target to pick it up.

        Args:
            targetArea (Float):  The target area determined by limelight.

        Returns:
            Float: Velocity in the X direction (robot oriented)
        """
        return min(-0.20, -(targetArea * -0.0125 + 1.3125))
        # calcX = -(-0.0002*(targetArea**2) + 0.0093*targetArea+1)
        # return max(-1, calcX)

    def calculate_rotation(self, targetX):
        """Calculate the rotational speed from the X value of the target in the camera frame.
        Return is inverted to make left rotation positive from a negative X value, meaning the
        target is to the left of center of the camera's view.

        Args:
            targetX (Float): The X value of the target in the camera frame, 0 is straight ahead,
            to the left is negative, to the right is positive.

        Returns:
            Float: Rotational velocity with CCW (left, robot oriented) positive.
        """
        return -(targetX / 25)

    def get_velocity(self):
        """Get calculated velocities from vision target data

        Returns:
            Tuple(vX, vY, vT): X, Y, and rotation velocities as a tuple.
        """
        return (self.drive_vX, self.drive_vY, self.drive_vRotate)

    def has_target(self):
        return self.tv

    def is_target_ready(self, acceptable_offset=2):
        if self.has_target():
            return False

        if abs(self.Tx.get()) < acceptable_offset:
            return True
        return False

    def execute(self) -> None:
        self.get_target()

    def set_pipeline(self, value: int):
        self.nt.putNumber("pipeline", value)

    def botpose_to_pose3d(self, poseArray) -> Tuple[Pose3d, Any]:
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
            return None, None
        else:
            return Pose3d(
                Translation3d(pX, pY, pZ), Rotation3d.fromDegrees(pRoll, pPitch, pYaw)
            ), self.timer.getFPGATimestamp() - (msLatency / 1000)
