import os
from components import swervedrive
from pathplannerlib.path import PathPlannerPath
from wpimath import controller, kinematics, geometry, controller, trajectory
import constants
import wpilib
import json

class AutoHelper:
    def __init__(self, drivetrain, auto_name):
        self.drivetrain: swervedrive.SwerveDrive = drivetrain
        if not auto_name.endswith(".auto"):
            auto_name += ".auto"

        data = ""
        file = os.path.join(
            os.path.dirname(__file__),
            "..",
            "deploy",
            "pathplanner",
            "autos",
            auto_name,
        )

        with open(file, 'r') as f:
            data = f.read()
        self.path = json.loads(data)


class PathHelper:
    def __init__(self, drivetrain, path_name):
        self.drivetrain: swervedrive.SwerveDrive = drivetrain
        self.timer = wpilib.Timer()
        self.timer.start()

        self.path = PathPlannerPath.fromPathFile(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "deploy",
                "pathplanner",
                "paths",
                path_name,
            )
        )

    def init_path(self, force_robot_starting_position=False):
        self.trajectory = self.path.getTrajectory(
            kinematics.ChassisSpeeds(0, 0, 0), geometry.Rotation2d()
        )
        self.controller = controller.HolonomicDriveController(
            controller.PIDController(1, 0, 0),
            controller.PIDController(1, 0, 0),
            controller.ProfiledPIDControllerRadians(
                1,
                0,
                0,
                trajectory.TrapezoidProfileRadians.Constraints(
                    constants.MAX_ANGULAR_VEL, constants.MAX_ANGULAR_ACCEL
                ),
            ),
        )
        self.controller.setEnabled(True)
        self.timer.reset()

        if force_robot_starting_position is False:
            return
        reset_pose = self.trajectory.sample(0).getTargetHolonomicPose()
        new_pose = geometry.Pose2d(
            reset_pose.X(),
            reset_pose.Y(),
            geometry.Rotation2d.fromDegrees(self.drivetrain.get_angle()),
        )
        self.drivetrain.resetPose(new_pose)

    def auto_move(self):
        # If odometry is bad, compute now + 0.02 as goal and now as current position
        # fake_pose = self.trajectory.sample(self.timer.get())
        # goal = self.trajectory.sample(self.timer.get() + 0.02)
        # adjustedSpeeds = self.controller.calculate(fake_pose.getTargetHolonomicPose(), goal.getTargetHolonomicPose(), goal.velocityMps, goal.heading)

        goal = self.trajectory.sample(self.timer.get())
        target_rotation = goal.targetHolonomicRotation.degrees()
        goal.targetHolonomicRotation = geometry.Rotation2d(0)
        current = self.drivetrain.getEstimatedPose()
        current = geometry.Pose2d(current.X(), current.Y(), geometry.Rotation2d(0))
        adjustedSpeeds = self.controller.calculate(
            current, goal.getTargetHolonomicPose(), 0, geometry.Rotation2d(0)
        )
        # speed = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(adjustedSpeeds.vx, adjustedSpeeds.vy, adjustedSpeeds.omega, goal.heading)
        self.drivetrain.set_absolute_automove_value(
            adjustedSpeeds.vx, adjustedSpeeds.vy
        )
        self.drivetrain.set_angle(target_rotation)

    def path_reached_end(self):
        return self.timer.get() >= self.trajectory.getTotalTimeSeconds()
