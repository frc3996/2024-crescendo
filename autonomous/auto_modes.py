import ntcore
import os
from components import swervedrive
from magicbot.state_machine import timed_state
import wpimath.controller
import wpimath.trajectory
from pathplannerlib.path import PathPlannerPath
import constants
import wpilib

from .base_auto import BaseAuto


class Path1(BaseAuto):
    MODE_NAME = "Path1"
    DEFAULT = True

    # Injection
    drivetrain: swervedrive.SwerveDrive
    nt: ntcore.NetworkTable

    @timed_state(duration=1, next_state="avance", first=True)
    def init(self):
        self.path = PathPlannerPath.fromPathFile(os.path.join(os.path.dirname(__file__), '..', "deploy", "pathplanner", "paths", "test_path2"))
        self.trajectory = self.path.getTrajectory(wpimath.kinematics.ChassisSpeeds(0,0,0), wpimath.geometry.Rotation2d())
        self.controller = wpimath.controller.HolonomicDriveController(
                wpimath.controller.PIDController(1, 0, 0),
                wpimath.controller.PIDController(1, 0, 0),
                wpimath.controller.ProfiledPIDControllerRadians(1, 0, 0, wpimath.trajectory.TrapezoidProfileRadians.Constraints(constants.MAX_ANGULAR_VEL, constants.MAX_ANGULAR_ACCEL)),
        )
        self.controller.setEnabled(True)

        reset_pose = self.trajectory.sample(0).getTargetHolonomicPose()
        new_pose = wpimath.geometry.Pose2d(reset_pose.X(), reset_pose.Y(), wpimath.geometry.Rotation2d.fromDegrees(self.drivetrain.get_angle()))
        self.drivetrain.resetPose(new_pose)

        self.auto_timer = wpilib.Timer.getFPGATimestamp()
        self.next_state("execute_path")
        # self.skip_recule = self.nt.getNumber("auto_mode/skip_recule", False)

    @timed_state(duration=10, next_state="shoot")
    def execute_path(self):
        now = wpilib.Timer.getFPGATimestamp() - self.auto_timer
        if now >= self.trajectory.getTotalTimeSeconds():
            self.next_state("shoot")

        fake_pose = self.trajectory.sample(now)
        goal = self.trajectory.sample(now+0.02)
        adjustedSpeeds = self.controller.calculate(fake_pose.getTargetHolonomicPose(), goal.getTargetHolonomicPose(), goal.velocityMps, goal.heading)
        speed = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(adjustedSpeeds.vx, adjustedSpeeds.vy, adjustedSpeeds.omega, goal.heading)

        self.drivetrain.set_absolute_automove_value(speed.vx, speed.vy)
        self.drivetrain.set_angle(goal.getTargetHolonomicPose().rotation().degrees())


    @timed_state(duration=1, next_state="recule")
    def shoot(self):
        # TODO
        pass
        # self.drivetrain.set_absolute_automove_value(0.1, 0)

    @timed_state(duration=1, next_state="finish")
    def recule(self):
        # TODO
        pass
        # if self.skip_recule is True:
        #     # Il est possible de forcer le changement immédiat vers un autre état avec:
        #     self.next_state("finish")

        # self.drivetrain.set_absolute_automove_value(-0.1, 0)
