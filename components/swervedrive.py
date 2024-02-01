"""
X:      The component of speed in the x direction relative to the field.
        Positive x is away from your alliance wall.
Y:      The component of speed in the y direction relative to the field.
        Positive y is to your left when standing behind the alliance wall.
Angle:  The angle of the robot as measured by a gyroscope. The robot's angle
        is considered to be zero when it is facing directly away from
        your alliance station wall. Remember that this should be CCW positive.
"""

import math
import magicbot
import ntcore
from dataclasses import dataclass
from wpimath import geometry, filter, kinematics, controller, trajectory, estimator
from navx import AHRS
from common import tools
from components import swervemodule
import pathplannerlib.telemetry
import constants


constants.MAX_WHEEL_SPEED = 4  # meter per second
constants.MAX_MODULE_SPEED = 4.5


@dataclass
class SwerveDriveConfig:
    base_width: float
    base_length: float
    is_simulation: bool


class SwerveDrive:
    # Modules injectés depuis le code de ../robot.py
    cfg: SwerveDriveConfig
    frontLeftModule: swervemodule.SwerveModule
    frontRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule
    navx: AHRS
    nt: ntcore.NetworkTable

    controller_forward = magicbot.will_reset_to(0)
    controller_strafe = magicbot.will_reset_to(0)
    controller_angle_stick_x = magicbot.will_reset_to(0)
    controller_angle_stick_y = magicbot.will_reset_to(0)
    request_wheel_lock = magicbot.will_reset_to(False)
    automove_forward = magicbot.will_reset_to(0)
    automove_strafe = magicbot.will_reset_to(0)
    automove_strength = magicbot.will_reset_to(0)

    debug = magicbot.tunable(False)

    angle_kp = magicbot.tunable(0.004)
    angle_ki = magicbot.tunable(0)
    angle_kd = magicbot.tunable(0)
    angle_max_acc = magicbot.tunable(constants.MAX_ANGULAR_VEL)
    angle_max_vel = magicbot.tunable(constants.MAX_ANGULAR_ACCEL)

    def setup(self):
        """
        Appelé après l'injection
        """
        self.chassis_speed = kinematics.ChassisSpeeds()
        self.sim_angle = 0
        self.lower_input_thresh = 0.1
        self.NavxPitchZero = 0
        self.NavxRollZero = 0
        self.zero_done = False

        self.lost_navx = False
        self.target_angle = 0

        self.fwd_limiter = filter.SlewRateLimiter(16)
        self.strafe_limiter = filter.SlewRateLimiter(16)

        self.angle_pid = controller.ProfiledPIDController(
            self.angle_kp,
            self.angle_ki,
            self.angle_kd,
            trajectory.TrapezoidProfile.Constraints(
                constants.MAX_ANGULAR_VEL, constants.MAX_ANGULAR_ACCEL
            ),
        )
        self.angle_pid.enableContinuousInput(-180, 180)
        self.angle_pid.setTolerance(-1, 1)


        self.frontLeftLocation = geometry.Translation2d(
            self.cfg.base_width / 2, self.cfg.base_length / 2
        )
        self.frontRightLocation = geometry.Translation2d(
            self.cfg.base_width / 2, -self.cfg.base_length / 2
        )
        self.backLeftLocation = geometry.Translation2d(
            -self.cfg.base_width / 2, self.cfg.base_length / 2
        )
        self.backRightLocation = geometry.Translation2d(
            -self.cfg.base_width / 2, -self.cfg.base_length / 2
        )
        self.kinematics = kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.navx_zero_angle()

        # self.odometry = kinematics.SwerveDrive4Odometry(
        self.odometry = estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.getRotation2d(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.rearLeftModule.getPosition(),
                self.rearRightModule.getPosition(),
            ),
            geometry.Pose2d(),
        )
        self.odometry.setVisionMeasurementStdDevs((0.5, 0.5, math.pi / 2))

    def init_navx(self):
        if self.zero_done is True:
            return
        self.zero_done = True
        self.navx_zero_all()

    def navx_zero_angle(self):
        self.navx.reset()
        self.target_angle = self.get_angle()
        self.angle_pid.reset(trajectory.TrapezoidProfile.State(0, 0))

    def navx_zero_all(self):
        self.navx_zero_angle()
        self.NavxPitchZero = self.navx.getPitch()
        self.NavxRollZero = self.navx.getRoll()

    def flush(self):
        """
        Cette méthode devrait être appelé pour remettre à zéro le drivetrain.
        Remotes les module swerve à zéro en même temps.
        """
        self.frontLeftModule.flush()
        self.frontRightModule.flush()
        self.rearLeftModule.flush()
        self.rearRightModule.flush()

    def init(self):
        self.init_navx()
        self.flush()
        self.refresh_nt_config()
        self.frontLeftModule.refresh_nt_config()
        self.frontRightModule.refresh_nt_config()
        self.rearLeftModule.refresh_nt_config()
        self.rearRightModule.refresh_nt_config()

    def refresh_nt_config(self):
        self.angle_pid.setPID(
            self.angle_kp,
            self.angle_ki,
            self.angle_kd,
        )
        constraint = trajectory.TrapezoidProfile.Constraints(
            self.angle_max_vel,
            self.angle_max_acc,
        )
        self.angle_pid.setConstraints(constraint)

    def getRotation2d(self):
        if self.cfg.is_simulation:
            return geometry.Rotation2d(self.sim_angle)
        else:
            return self.navx.getRotation2d()

    def get_angle(self):
        """
        Retourne l'angle absolue basée sur le zéro
        De -180 à 180 degrée
        """
        return self.getRotation2d().degrees()

    def set_angle(self, angle):
        """Écrit l'angle absolue à atteindre de 0 à 360"""
        self.target_angle = angle

    def set_absolute_automove_value(self, forward, strafe, strength=0.2):
        self.automove_forward = forward
        self.automove_strafe = strafe
        self.automove_strength = strength

    def relative_rotate(self, rotation):
        self.target_angle = ((self.get_angle() + 180 + rotation + 360) % 360) - 180

    def set_relative_automove_value(self, forward, strafe, strength=0.2):
        vector = tools.rotate_vector([-forward, strafe], self.get_angle())
        self.automove_forward = vector[0]
        self.automove_strafe = vector[1]
        self.automove_strength = strength

    def set_controller_values(self, forward, strafe, angle_stick_x, angle_stick_y):
        self.controller_forward = -forward
        self.controller_strafe = -strafe
        self.controller_angle_stick_x = angle_stick_x
        self.controller_angle_stick_y = angle_stick_y
        self.compute_look_at_stick_angle()

    def __compute_move(self):
        """Fait bouger le robot avec un contrôleur"""
        forward = tools.square_input(self.controller_forward)
        strafe = tools.square_input(self.controller_strafe)

        if abs(forward) < self.lower_input_thresh:
            forward = 0
        if abs(strafe) < self.lower_input_thresh:
            strafe = 0

        forward *= constants.MAX_WHEEL_SPEED
        strafe *= constants.MAX_WHEEL_SPEED

        if self.automove_strength != 0:
            new_fwd = self.automove_forward + (forward * (1 - self.automove_strength))
            new_strafe = self.automove_strafe + (strafe * (1 - self.automove_strength))
            forward = new_fwd
            strafe = new_strafe

        return forward, strafe

    def compute_look_at_stick_angle(self):
        angle_stick_x = self.controller_angle_stick_x
        angle_stick_y = self.controller_angle_stick_y

        # Élimite la zone morte du joystick (petits déplacements)
        if math.sqrt(angle_stick_x**2 + angle_stick_y**2) > 0.5:
            angle = (math.degrees(math.atan2(angle_stick_y, angle_stick_x)) + 360) % 360
            angle += 270
            angle %= 360
            angle -= 180
            angle = -angle
            self.set_angle(angle)

    def snap_angle_nearest_180(self):
        """Force le robot à regarder vers l'avant ou l'arrière"""
        angle_deg = self.get_angle() + 180
        remainder = angle_deg % 180
        if remainder < 90:
            rounded_angle_deg = angle_deg // 180 * 180
        else:
            rounded_angle_deg = (angle_deg // 180 + 1) * 180
        self.set_angle((rounded_angle_deg % 360) - 180)

    def snap_angle_nearest_90(self):
        """Force le robot à regarder vers le côté le plus prêt"""
        angle_deg = self.get_angle() + 180
        remainder = angle_deg % 90
        if remainder < 45:
            rounded_angle_deg = angle_deg // 90 * 90
        else:
            rounded_angle_deg = (angle_deg // 90 + 1) * 90
        self.set_angle((rounded_angle_deg % 360) - 180)

    def __compute_navx_angle_error(self) -> float:
        # Lost navx, avoid turning
        if self.navx.isConnected() is False:
            self.lost_navx = True
            return 0

        # We previously lost navx but got it back
        if self.lost_navx is True:
            self.angle_pid.reset(trajectory.TrapezoidProfile.State(0, 0))
            self.target_angle = self.get_angle()
            self.lost_navx = False

        # Compute error if navx is okay
        angle_error = self.angle_pid.calculate(
            self.get_angle(), self.target_angle
        )
        angle_error = max(min(angle_error, 2), -2)
        if abs(angle_error) <= 0.002:
            angle_error = 0

        return angle_error


    def __calculate_vectors(self):
        """
        Réalise le calcul des angles et vitesses pour chaque swerve module
        en fonction des commandes reçues
        """

        fwdSpeed, strafeSpeed = self.__compute_move()
        rot = self.__compute_navx_angle_error()

        fwdSpeed = self.fwd_limiter.calculate(fwdSpeed)
        strafeSpeed = self.strafe_limiter.calculate(strafeSpeed)

        self.chassis_speed = kinematics.ChassisSpeeds.discretize(
            kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                fwdSpeed, strafeSpeed, rot, self.getRotation2d()
            ),
            0.02,
        )
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            self.chassis_speed
        )
        self.sim_angle = self.sim_angle + (rot * 0.02 * 20)

        # Non field centric, kept as reference
        # swerveModuleStates = self.kinematics.toSwerveModuleStates(
        #     kinematics.ChassisSpeeds.discretize(
        #         kinematics.ChassisSpeeds(fwdSpeed, strafeSpeed, rot),
        #         0.02,
        #     )
        # )

        kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, constants.MAX_WHEEL_SPEED
        )


        # Ne fais rien si les vecteurs sont trop petits
        if fwdSpeed == 0 and strafeSpeed == 0 and self.request_wheel_lock:
            # TODO Ajouter le requested_wheel_lock
            pass

        self.frontLeftModule.setTargetState(swerveModuleStates[0])
        self.frontRightModule.setTargetState(swerveModuleStates[1])
        self.rearLeftModule.setTargetState(swerveModuleStates[2])
        self.rearRightModule.setTargetState(swerveModuleStates[3])

    def update_nt(self):
        """
        Affiche des informations de débogage
        """
        if self.debug:
            self.nt.putNumber("navx/pitch", self.navx.getPitch())
            self.nt.putNumber("navx/yaw", self.navx.getYaw())
            self.nt.putNumber("navx/roll", self.navx.getRoll())
            self.nt.putNumber("navx/angle", self.navx.getAngle())
            self.nt.putNumber(
                "debug/navx_angle_error", self.target_angle - self.get_angle()
            )

    def __updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.getRotation2d(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.rearLeftModule.getPosition(),
                self.rearRightModule.getPosition(),
            ),
        )
        # TODO To add vision merging in pose
        # visionPose, visionTime = self.limelight.getBotPoseEstimateForAlliance()
        # if visionPose:
        #     if (
        #         abs(visionPose.x - self.estimatorPose.x) < 0.5
        #         and abs(visionPose.y - self.estimatorPose.y) < 0.5
        #     ):
        #         stddevupdate = remap(visionPose.x,2.0, 8.0, 0.3, 2.0)
        #         # self.logger.info(f'Adding vision measuerment with StdDev of {stddevupdate} and distance of {visionPose.x} ')
        #         self.estimator.addVisionMeasurement(visionPose.toPose2d(), visionTime, (stddevupdate, stddevupdate, math.pi/2))

        current_pose = self.getEstimatedPose()
        pathplannerlib.telemetry.PPLibTelemetry.setCurrentPose(current_pose)

    def getModuleStates(self):
        """
        For PathPlannerLib
        Return Swerve module states
        """
        states = [
            self.frontLeftModule.getState(),
            self.frontRightModule.getState(),
            self.rearLeftModule.getState(),
            self.rearRightModule.getState(),
        ]
        return states

    def getEstimatedPose(self) -> geometry.Pose2d:
        """
        For PathPlannerLib
        Robot pose supplier
        """
        return self.odometry.getEstimatedPosition()

    def resetPose(self, pose: geometry.Pose2d):
        """
        For PathPlannerLib
        Method to reset odometry (will be called if your auto has a starting pose)
        """
        self.frontLeftModule.resetPose()
        self.frontRightModule.resetPose()
        self.rearLeftModule.resetPose()
        self.rearRightModule.resetPose()

        self.odometry.resetPosition(
            self.getRotation2d(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.rearLeftModule.getPosition(),
                self.rearRightModule.getPosition(),
            ),
            pose,
        )

    def getRobotRelativeSpeeds(self):
        """
        ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        """
        return self.kinematics.toChassisSpeeds(self.getModuleStates())

    def getFieldRelativeSpeeds(self):
        """
        ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        """
        speed = kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
            self.kinematics.toChassisSpeeds(self.getModuleStates()),
            self.getRotation2d(),
        )
        return speed

    def driveFieldRelative(self, fieldRelativeSpeeds: kinematics.ChassisSpeeds):
        """
        Method that will drive the robot given FIELD RELATIVE ChassisSpeeds
        """
        self.chassis_speed = fieldRelativeSpeeds
        self.driveRobotRelative(
            kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds, self.getEstimatedPose().rotation()
            )
        )

    def driveRobotRelative(self, robotRelativeSpeeds: kinematics.ChassisSpeeds):
        """
        Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        """
        targetSpeeds = kinematics.ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02)

        targetStates = self.kinematics.toSwerveModuleStates(targetSpeeds)
        self.setStates(targetStates)

    def setStates(self, targetStates: list[kinematics.SwerveModuleState]):
        """
        Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        """
        targetStates = kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            targetStates, constants.MAX_MODULE_SPEED
        )
        self.frontLeftModule.setTargetState(targetStates[0])
        self.frontRightModule.setTargetState(targetStates[1])
        self.rearLeftModule.setTargetState(targetStates[2])
        self.rearRightModule.setTargetState(targetStates[3])

    def execute(self):
        """
        Calcul et transmet la commande de vitesse et d'angle à chaque swerve module.
        """
        # Évaluation de la commande selon le mode d'opération
        self.__updateOdometry()

        # Calcul des vecteurs
        self.__calculate_vectors()
        self.update_nt()
