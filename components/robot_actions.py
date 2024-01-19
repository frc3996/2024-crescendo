
import wpilib
from components import swervedrive, lobra, intake
from wpimath.controller import PIDController
from common import limelight, arduino_light
import ntcore
import os
import math
from pathplannerlib.path import PathPlannerPath
import pathplannerlib.telemetry
# from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
# from pathplannerlib.auto import NamedCommands
# from pathplannerlib.auto import PathPlannerAuto
import wpimath.kinematics
import wpimath.geometry
from components.swervedrive import rotate_vector
import wpimath.controller
import wpimath.trajectory

def get_linear_damp_ratio(current_value, minimum, maximum):
    """Return a ratio from 1 to 0 based on a range"""
    m = maximum - minimum
    b = minimum
    res = (current_value - b) / m
    res = min(1, res)
    res = max(0, res)
    return res


class RobotActions:
    drivetrain: swervedrive.SwerveDrive
    intake: intake.Intake
    lobras: lobra.LoBras
    limelight_intake: limelight.Limelight
    limelight_shoot: limelight.Limelight
    arduino_light: arduino_light.I2CArduinoLight
    status_light: wpilib.Solenoid
    nt: ntcore.NetworkTable

    def setup(self):
        self.nt.putNumber("actions/shoot_limelight_adjust_pid/Kp", 0.012)
        self.nt.putNumber("actions/shoot_limelight_adjust_pid/Ki", 0)
        self.nt.putNumber("actions/shoot_limelight_adjust_pid/Kd", 0)
        self.shoot_limelight_adjust_pid = PIDController(0, 0, 0)

        self.nt.putNumber("actions/intake_limelight_adjust_pid/Kp", 0.7)
        self.nt.putNumber("actions/intake_limelight_adjust_pid/Ki", 0)
        self.nt.putNumber("actions/intake_limelight_adjust_pid/Kd", 0)
        self.intake_limelight_adjust_pid = PIDController(0, 0, 0)


        self.path = PathPlannerPath.fromPathFile(os.path.join(os.path.dirname(__file__), '..', "deploy", "pathplanner", "paths", "test_path"))
        self.trajectory = self.path.getTrajectory(wpimath.kinematics.ChassisSpeeds(0,0,0), wpimath.geometry.Rotation2d())

        self.controller = wpimath.controller.HolonomicDriveController(
                wpimath.controller.PIDController(1, 0, 0),
                wpimath.controller.PIDController(1, 0, 0),
                wpimath.controller.ProfiledPIDControllerRadians(1, 0, 0, wpimath.trajectory.TrapezoidProfileRadians.Constraints(4, 2)),
        )

        # Register Named Commands

        # AutoBuilder.configureHolonomic(
        #     self.drivetrain.getPose, # Robot pose supplier
        #     self.drivetrain.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
        #     self.drivetrain.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        #     self.drivetrain.driveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        #     HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
        #         PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
        #         PIDConstants(5.0, 0.0, 0.0), # Rotation PID constants
        #         4.5, # Max module speed, in m/s
        #         0.4, # Drive base radius in meters. Distance from robot center to furthest module.
        #         ReplanningConfig() # Default path replanning config. See the API for the options here
        #     ),
        #     True,
        #     self # Reference to this subsystem to set requirements
        # )

    def reset_auto(self):
        self.auto_timer = wpilib.Timer.getFPGATimestamp()
        self.drivetrain.resetPose(self.trajectory.sample(0).getTargetHolonomicPose())

    def auto_test(self):
        now = wpilib.Timer.getFPGATimestamp() - self.auto_timer
        test_goal = self.trajectory.sample(now).getTargetHolonomicPose()
        goal = self.trajectory.sample(now+0.02)
        angle = wpimath.geometry.Rotation2d.fromDegrees(0)
        adjustedSpeeds = self.controller.calculate(test_goal, goal.getTargetHolonomicPose(), 0, angle)
        self.drivetrain.set_absolute_automove_value(adjustedSpeeds.vx*10, adjustedSpeeds.vy*10)
        print(adjustedSpeeds.vx*10, adjustedSpeeds.vy*10)

        self.drivetrain.set_angle(goal.getTargetHolonomicPose().rotation().degrees()+180)
        # vectors = rotate_vector([goal.getTargetHolonomicPose().X(), goal.getTargetHolonomicPose().Y()], goal.heading.degrees())
        # x = vectors[0] / abs(math.sqrt(vectors[0]**2 + vectors[1]**2))
        # y = vectors[1] / abs(math.sqrt(vectors[0]**2 + vectors[1]**2))
        # self.drivetrain.set_absolute_automove_value(x * 0.2, y* 0.2)
        # print(round(now), x, y, goal.getTargetHolonomicPose().X(), goal.getTargetHolonomicPose().Y(), goal.heading.degrees())


    def autoshoot_amp(self):
        arm_angle = 0  # TODO calibrate all
        head_angle = 0  # TODO calibrate all
        if self.intake.shot_done is True:
            self.retract()
            return

        self.drivetrain.set_angle(0) # TODO GET PROPER ANGLE
        self.lobras.set_angle(arm_angle, head_angle)
        fwd = 0
        error = 0
        if self.limelight_intake.is_target_in_view():
            offset = self.limelight_intake.get_tx()
            if self.limelight_intake.get_ta() >= 1.5:  # TODO ADJUST SIZE TO ARRIVAL
                self.intake.shoot(fire=True)
            else:
                # Move forward
                res = get_linear_damp_ratio(abs(offset), 50, 0)
                fwd = 0.5 * res
            self.shoot_limelight_adjust_pid.setP(self.nt.getNumber("actions/shoot_limelight_adjust_pid/Kp", 0))
            self.shoot_limelight_adjust_pid.setI(self.nt.getNumber("actions/shoot_limelight_adjust_pid/Ki", 0))
            self.shoot_limelight_adjust_pid.setD(self.nt.getNumber("actions/shoot_limelight_adjust_pid/Kd", 0))
            error = self.shoot_limelight_adjust_pid.calculate(offset, 0)
        else:
            fwd = 0.5
            error = 0

        self.drivetrain.set_relative_automove_value(fwd, error)

    def autoshoot_speaker(self):
        pass

    def retract(self):
        arm_angle = 0  # TODO calibrate all
        head_angle = 0  # TODO calibrate all
        self.lobras.set_angle(arm_angle, head_angle)
        self.shoot_limelight_adjust_pid.reset()
        self.intake_limelight_adjust_pid.reset()
        pass

    def TestLoBra(self, angle):
        self.lobras.set_arm_angle(angle)

    def autointake_with_limelight(self):
        arm_angle = 20  # TODO calibrate all
        head_angle = 30  # TODO calibrate all

        if self.intake.is_object_intaken() is True:
            self.arduino_light.set_RGB(0, 0, 255)
            # self.status_light.set(1)
            self.retract()
            return

        self.intake.intake()
        self.lobras.set_angle(arm_angle, head_angle)
        if self.limelight_intake.is_target_in_view():
            offset = self.limelight_intake.get_tx()
            res = get_linear_damp_ratio(abs(offset), 50, 0)
            fwd = 0.5 * res
            self.intake_limelight_adjust_pid.setP(self.nt.getNumber("actions/intake_limelight_adjust_pid/Kp", 0))
            self.intake_limelight_adjust_pid.setI(self.nt.getNumber("actions/intake_limelight_adjust_pid/Ki", 0))
            self.intake_limelight_adjust_pid.setD(self.nt.getNumber("actions/intake_limelight_adjust_pid/Kd", 0))
            error = self.intake_limelight_adjust_pid.calculate(offset, 0)
        else:
            fwd = 0.5
            error = 0

        self.drivetrain.relative_rotate(-error)
        self.drivetrain.set_relative_automove_value(fwd, 0)

    def execute(self):
        pathplannerlib.telemetry.PPLibTelemetry.setCurrentPose(self.drivetrain.getPose())
        pass
