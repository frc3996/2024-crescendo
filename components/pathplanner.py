import pathplannerlib.telemetry
from wpimath import controller, estimator, filter, geometry, kinematics

from components.chassis import ChassisComponent


class PathPlanner:
    drivetrain: ChassisComponent

    def get_odometry_pose(self) -> geometry.Pose2d:
        """
        For PathPlannerLib
        Robot pose supplier
        """
        return self.drivetrain.get_pose()

    def execute(self):
        pathplannerlib.telemetry.PPLibTelemetry.setCurrentPose(
            self.drivetrain.get_pose()
        )
