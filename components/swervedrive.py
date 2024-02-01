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
from dataclasses import dataclass

import pathplannerlib.telemetry
import wpimath.units
from magicbot import feedback, tunable
from navx import AHRS
from wpimath import (controller, estimator, filter, geometry, kinematics,
                     trajectory)

import constants
from components import swervemodule

MAX_WHEEL_SPEED = 4  # meter per second
MAX_MODULE_SPEED = 4.5
LOWER_INPUT_THRESH = 0.1


def rotate_vector(vector, angle):
    rad = math.radians(angle)
    x = vector[0] * math.cos(rad) - vector[1] * math.sin(rad)
    y = vector[0] * math.sin(rad) + vector[1] * math.cos(rad)
    return (x, y)


@dataclass
class SwerveDriveConfig:
    field_centric: bool
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

    # Pid de rotation. Ajuster via le ShuffleBoard et écrire la nouvelle valeur ici
    kP = tunable(0.004)
    kI = tunable(0.0)
    kD = tunable(0.0)
    max_vel = tunable(constants.MAX_ANGULAR_VEL)
    max_accel = tunable(constants.MAX_ANGULAR_ACCEL)

    target_angle = wpimath.units.degrees(0)
    sim_angle = wpimath.units.degrees(0)

    # Mets les commandes à zéro
    vector_fwd = wpimath.units.meters_per_second(0.0)
    _vector_strafe = wpimath.units.meters_per_second(0.0)
    vector_rcw = wpimath.units.radians_per_second(0.0)

    _requested_angles = {
        "front_left": 0,
        "front_right": 0,
        "rear_left": 0,
        "rear_right": 0,
    }

    _requested_speeds = {
        "front_left": 0,
        "front_right": 0,
        "rear_left": 0,
        "rear_right": 0,
    }

    request_wheel_lock = False
    automove_forward = 0
    automove_strafe = 0
    automove_strength = 0

    def setup(self):
        """
        Appelé après l'injection
        """
        self.navx = AHRS.create_spi(update_rate_hz=50)

        self.chassis_speed = kinematics.ChassisSpeeds()

        self.fwd_limiter = filter.SlewRateLimiter(16)
        self.strafe_limiter = filter.SlewRateLimiter(16)

        self.angle_pid = controller.ProfiledPIDController(
            1,
            0,
            0,
            trajectory.TrapezoidProfile.Constraints(
                constants.MAX_ANGULAR_VEL, constants.MAX_ANGULAR_ACCEL
            ),
        )
        self.angle_pid.enableContinuousInput(-180, 180)
        self.angle_pid.setTolerance(-1, 1)

        self.kinematics = kinematics.SwerveDrive4Kinematics(
            geometry.Translation2d(self.cfg.base_width / 2, self.cfg.base_length / 2),
            geometry.Translation2d(self.cfg.base_width / 2, -self.cfg.base_length / 2),
            geometry.Translation2d(-self.cfg.base_width / 2, self.cfg.base_length / 2),
            geometry.Translation2d(-self.cfg.base_width / 2, -self.cfg.base_length / 2),
        )

        self.odometry = estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_gyro_angle(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.rearLeftModule.getPosition(),
                self.rearRightModule.getPosition(),
            ),
            geometry.Pose2d(),
        )
        self.odometry.setVisionMeasurementStdDevs((0.5, 0.5, math.pi / 2))

    def on_enable(self):
        # Zero the navx
        self.navx_zero_angle()

        # Make sure nothing is moving
        self.flush()

        # Adjust the network table PID values
        self.angle_pid.setP(self.kP)
        self.angle_pid.setI(self.kI)
        self.angle_pid.setD(self.kD)
        constraint = trajectory.TrapezoidProfile.Constraints(
            self.max_vel,
            self.max_accel,
        )
        self.angle_pid.setConstraints(constraint)

        # TODO: Move this on on_enable of swervemodule
        self.frontLeftModule.refresh_nt_config()
        self.frontRightModule.refresh_nt_config()
        self.rearLeftModule.refresh_nt_config()
        self.rearRightModule.refresh_nt_config()

    @staticmethod
    def square_input(input):
        """Retourne la valeur au carré en conservant le signe"""
        return math.copysign(input * input, input)

    def flush(self):
        """
        Cette méthode devrait être appelé pour remettre à zéro le drivetrain.
        Remotes les module swerve à zéro en même temps.
        """
        self.vector_fwd = 0.0
        self.vector_strafe = 0.0
        self.vector_rcw = 0.0

        self._requested_angles["front_left"] = 0
        self._requested_angles["front_right"] = 0
        self._requested_angles["rear_left"] = 0
        self._requested_angles["rear_right"] = 0

        self._requested_speeds["front_left"] = 0
        self._requested_speeds["front_right"] = 0
        self._requested_speeds["rear_left"] = 0
        self._requested_speeds["rear_right"] = 0

        self.frontLeftModule.flush()
        self.frontRightModule.flush()
        self.rearLeftModule.flush()
        self.rearRightModule.flush()

    def get_gyro_angle(self):
        if self.cfg.is_simulation:
            return geometry.Rotation2d(self.sim_angle)
        else:
            if self.navx.isConnected() is True:
                return self.navx.getRotation2d()
            else:
                # I really don't know if this is a good idea
                return self.getEstimatedPose().rotation()

    def navx_zero_angle(self):
        self.navx.reset()
        self.target_angle = self.get_angle()

    @feedback
    def navx_pitch(self):
        return self.navx.getPitch()

    @feedback
    def navx_yaw(self):
        return self.navx.getYaw()

    @feedback
    def navx_roll(self):
        return self.navx.getRoll()

    @feedback
    def navx_angle(self):
        return self.navx.getAngle()

    @feedback
    def navx_angle_error(self):
        return self.target_angle - self.get_angle()

    def get_angle(self):
        """
        Retourne l'angle absolue basée sur le zéro
        De -180 à 180 degrée
        """
        return self.getEstimatedPose().rotation().degrees()

    def set_fwd(self, fwd):
        """
        Écrit le vecteur 'forward'
        :param fwd: Valeur en m/s
        """
        self.vector_fwd = fwd

    def set_strafe(self, strafe):
        """
        Écrit la valeur 'strafe'
        :param strafe: Valeur en m/s
        """
        self.vector_strafe = strafe

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
        vector = rotate_vector([-forward, strafe], self.get_angle())
        self.automove_forward = vector[0]
        self.automove_strafe = vector[1]
        self.automove_strength = strength

    def set_controller_values(self, forward, strafe, angle_stick_x, angle_stick_y):
        # Élimite la zone morte du joystick (petits déplacements)
        if math.sqrt(angle_stick_x**2 + angle_stick_y**2) > 0.5:
            angle = (math.degrees(math.atan2(angle_stick_y, angle_stick_x)) + 360) % 360
            angle += 270
            angle %= 360
            angle -= 180
            angle = -angle
            self.set_angle(angle)

        forward = self.square_input(forward)
        strafe = self.square_input(strafe)

        if abs(forward) < LOWER_INPUT_THRESH:
            forward = 0
        if abs(strafe) < LOWER_INPUT_THRESH:
            strafe = 0

        forward *= MAX_WHEEL_SPEED
        strafe *= MAX_WHEEL_SPEED

        if self.automove_strength != 0:
            new_fwd = self.automove_forward + (forward * (1 - self.automove_strength))
            new_strafe = self.automove_strafe + (strafe * (1 - self.automove_strength))
            forward = new_fwd
            strafe = new_strafe

        self.set_fwd(forward)
        self.set_strafe(strafe)

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

    @property
    def vector_strafe(self) -> wpimath.units.meters_per_second:
        return self._vector_strafe

    @vector_strafe.setter
    def vector_strafe(self, value: wpimath.units.meters_per_second):
        self._vector_strafe = self.strafe_limiter.calculate(value)

    def _calculate_vectors(self):
        """
        Réalise le calcul des angles et vitesses pour chaque swerve module
        en fonction des commandes reçues
        """

        if self.cfg.field_centric:
            angle_error = self.angle_pid.calculate(self.get_angle(), self.target_angle)
            angle_error = max(min(angle_error, 2), -2)
            # print(f"{self.get_angle():.3f}, {self.target_angle:.3f}, {angle_error:.3f}")
            self.vector_rcw = angle_error

        # Ne fais rien si les vecteurs sont trop petits
        if self.vector_strafe == 0 and self.vector_fwd == 0 and self.request_wheel_lock:
            # On remets à zéro la vitesse
            self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)
            # Place les roues à 45 degrées, utile pour la défense
            self._requested_angles["front_left"] = 45
            self._requested_angles["front_right"] = -45
            self._requested_angles["rear_left"] = -45
            self._requested_angles["rear_right"] = 45
            self.request_wheel_lock = False
            return

        if abs(self.vector_rcw) <= 0.002:
            self.vector_rcw = 0
        rot = self.vector_rcw

        fwdSpeed = self.fwd_limiter.calculate(self.vector_fwd)
        strafeSpeed = self.strafe_limiter.calculate(-self.vector_strafe)

        if self.cfg.field_centric:
            self.chassis_speed = kinematics.ChassisSpeeds.discretize(
                kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    fwdSpeed, strafeSpeed, rot, self.getEstimatedPose().rotation()
                ),
                0.02,
            )
            swerveModuleStates = self.kinematics.toSwerveModuleStates(
                self.chassis_speed
            )
            self.sim_angle = self.sim_angle + (rot * 0.02 * 20)
        else:
            swerveModuleStates = self.kinematics.toSwerveModuleStates(
                kinematics.ChassisSpeeds.discretize(
                    kinematics.ChassisSpeeds(fwdSpeed, strafeSpeed, rot),
                    0.02,
                )
            )

        kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, MAX_WHEEL_SPEED
        )

        self.frontLeftModule.setTargetState(swerveModuleStates[0])
        self.frontRightModule.setTargetState(swerveModuleStates[1])
        self.rearLeftModule.setTargetState(swerveModuleStates[2])
        self.rearRightModule.setTargetState(swerveModuleStates[3])

        # Remise à zéro des valeurs demandées par sécurité
        self.vector_fwd = 0.0
        self.vector_strafe = 0.0
        self.vector_rcw = 0.0

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.get_gyro_angle(),
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
        states = (
            self.frontLeftModule.getState(),
            self.frontRightModule.getState(),
            self.rearLeftModule.getState(),
            self.rearRightModule.getState(),
        )
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
            self.get_gyro_angle(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.rearLeftModule.getPosition(),
                self.rearRightModule.getPosition(),
            ),
            pose,
        )

    def execute(self):
        """
        Calcul et transmet la commande de vitesse et d'angle à chaque swerve module.
        """
        # Évaluation de la commande selon le mode d'opération
        self.updateOdometry()
        self.automove_forward = 0
        self.automove_strafe = 0
        self.automove_strength = 0

        # Calcul des vecteurs
        self._calculate_vectors()

        # Remets les vitesse à zéro
        self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)
        self.request_wheel_lock = False

    # def getRobotRelativeSpeeds(self):
    #     """
    #     ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    #     """
    #     return self.kinematics.toChassisSpeeds(self.getModuleStates())
    #
    # def getFieldRelativeSpeeds(self):
    #     """
    #     ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    #     """
    #     speed = kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
    #         self.kinematics.toChassisSpeeds(self.getModuleStates()),
    #         self.getEstimatedPose().rotation(),
    #     )
    #     return speed
    #

    # def driveRobotRelative(self, robotRelativeSpeeds: kinematics.ChassisSpeeds):
    #     """
    #     Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    #     """
    #     targetSpeeds = kinematics.ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02)
    #
    #     targetStates = self.kinematics.toSwerveModuleStates(targetSpeeds)
    #     self.setStates(targetStates)
    #
    # def setStates(
    #     self,
    #     targetStates: tuple[
    #         kinematics.SwerveModuleState,
    #         kinematics.SwerveModuleState,
    #         kinematics.SwerveModuleState,
    #         kinematics.SwerveModuleState,
    #     ],
    # ):
    #     """
    #     Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    #     """
    #     targetStates = kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
    #         targetStates, MAX_MODULE_SPEED
    #     )
    #     self.frontLeftModule.setTargetState(targetStates[0])
    #     self.frontRightModule.setTargetState(targetStates[1])
    #     self.rearLeftModule.setTargetState(targetStates[2])
    #     self.rearRightModule.setTargetState(targetStates[3])

    # def controller_relative_rotate(self):
    #     """Fait bouger le robot avec un contrôleur"""
    #     angle_stick_x = self.controller_angle_stick_x
    #     rotate = self.square_input(angle_stick_x)
    #     self._rotate_robot(rotate)

    # def _rotate_robot(self, rcw):
    #     """
    #     ROTATION NON FIELD CENTRIC!!
    #     Écrit la valeur de rotation
    #     :param rcw: Valeur de -1 à 1
    #     """
    #     self.vector_rcw = rcw
