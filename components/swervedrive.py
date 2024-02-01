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
import ntcore
from dataclasses import dataclass
from wpimath import geometry, filter, kinematics, controller, trajectory, estimator
from navx import AHRS
from components import swervemodule
import pathplannerlib.telemetry
import constants


MAX_WHEEL_SPEED = 4  # meter per second
MAX_MODULE_SPEED = 4.5


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
    navx: AHRS
    nt: ntcore.NetworkTable

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

        # Place les modules dans un dictionaire
        self.modules = {
            "front_left": self.frontLeftModule,
            "front_right": self.frontRightModule,
            "rear_left": self.rearLeftModule,
            "rear_right": self.rearRightModule,
        }

        # Mets les commandes à zéro
        self._requested_vectors = {"fwd": 0, "strafe": 0, "rcw": 0}

        self._requested_angles = {
            "front_left": 0,
            "front_right": 0,
            "rear_left": 0,
            "rear_right": 0,
        }

        self._requested_speeds = {
            "front_left": 0,
            "front_right": 0,
            "rear_left": 0,
            "rear_right": 0,
        }

        self.debug = False
        self.nt.putBoolean("swerve/debug", False)
        self.field_centric = self.cfg.field_centric

        self.lost_navx = False

        self.controller_forward = 0
        self.controller_strafe = 0
        self.controller_angle_stick_x = 0
        self.controller_angle_stick_y = 0
        self.target_angle = 0
        self.request_wheel_lock = False
        self.automove_forward = 0
        self.automove_strafe = 0
        self.automove_strength = 0

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

        # Pid de rotation. Ajuster via le ShuffleBoard et écrire la nouvelle valeur ici
        self.nt.putNumber("swerve/angle_pid/Kp", 0.004)
        self.nt.putNumber("swerve/angle_pid/Ki", 0)
        self.nt.putNumber("swerve/angle_pid/Kd", 0)
        self.nt.putNumber("swerve/angle_pid/max_vel", constants.MAX_ANGULAR_VEL)
        self.nt.putNumber("swerve/angle_pid/max_acc", constants.MAX_ANGULAR_ACCEL)

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

    @staticmethod
    def square_input(input):
        """Retourne la valeur au carré en conservant le signe"""
        return math.copysign(input * input, input)

    @staticmethod
    def normalize(data):
        """
        Rends les vecteurs unitaire (S'assure que la longueur du vecteur est de 1)
        :param data: Données à normaliser
        :returns: Les données normalisées
        """
        maxMagnitude = max(abs(x) for x in data)

        if maxMagnitude > 1.0:
            for i in range(len(data)):
                data[i] = data[i] / maxMagnitude

        return data

    @staticmethod
    def normalizeDictionary(data):
        """
        Trouve le vecteur de taille maximale dans les données et
        divise les autres par sa longueur.
        :param data: Le dictionaire à normaliser
        :returns: Le dictionaire normalisé
        """
        maxMagnitude = max(abs(x) for x in data.values())

        if maxMagnitude > 1.0:
            for key in data:
                data[key] = data[key] / maxMagnitude

        return data

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
        self._requested_vectors = {"fwd": 0, "strafe": 0, "rcw": 0}

        self._requested_angles = {
            "front_left": 0,
            "front_right": 0,
            "rear_left": 0,
            "rear_right": 0,
        }

        self._requested_speeds = {
            "front_left": 0,
            "front_right": 0,
            "rear_left": 0,
            "rear_right": 0,
        }

        for module in self.modules.values():
            module.flush()

    def init(self):
        self.init_navx()
        self.flush()
        self.refresh_nt_config()
        for key in self.modules:
            self.modules[key].refresh_nt_config()

    def refresh_nt_config(self):
        self.angle_pid.setP(self.nt.getNumber("swerve/angle_pid/Kp", 0))
        self.angle_pid.setI(self.nt.getNumber("swerve/angle_pid/Ki", 0))
        self.angle_pid.setD(self.nt.getNumber("swerve/angle_pid/Kd", 0))
        constraint = trajectory.TrapezoidProfile.Constraints(
            self.nt.getNumber("swerve/angle_pid/max_vel", 0),
            self.nt.getNumber("swerve/angle_pid/max_acc", 0),
        )
        self.angle_pid.setConstraints(constraint)
        self.debug = self.nt.getBoolean("swerve/debug", False)

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

    def set_fwd(self, fwd):
        """
        Écrit le vecteur 'forward'
        :param fwd: Valeur en m/s
        """
        self._requested_vectors["fwd"] = fwd

    def set_strafe(self, strafe):
        """
        Écrit la valeur 'strafe'
        :param strafe: Valeur en m/s
        """
        self._requested_vectors["strafe"] = strafe

    def set_angle(self, angle):
        """Écrit l'angle absolue à atteindre de 0 à 360"""
        self.target_angle = angle

    def _rotate_robot(self, rcw):
        """
        ROTATION NON FIELD CENTRIC!!
        Écrit la valeur de rotation
        :param rcw: Valeur de -1 à 1
        """
        self._requested_vectors["rcw"] = rcw

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
        self.controller_forward = forward
        self.controller_strafe = strafe
        self.controller_angle_stick_x = angle_stick_x
        self.controller_angle_stick_y = angle_stick_y

        self.compute_look_at_stick_angle()

    def compute_move(self):
        """Fait bouger le robot avec un contrôleur"""
        forward = self.square_input(-self.controller_forward)
        strafe = self.square_input(self.controller_strafe)

        if abs(forward) < self.lower_input_thresh:
            forward = 0
        if abs(strafe) < self.lower_input_thresh:
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

    def controller_relative_rotate(self):
        """Fait bouger le robot avec un contrôleur"""
        angle_stick_x = self.controller_angle_stick_x
        rotate = self.square_input(angle_stick_x)
        self._rotate_robot(rotate)

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

    def _calculate_vectors(self):
        """
        Réalise le calcul des angles et vitesses pour chaque swerve module
        en fonction des commandes reçues
        """

        if self.field_centric:
            if self.navx.isConnected() is True:
                if self.lost_navx is True:
                    # self.angle_pid.reset()
                    self.target_angle = self.get_angle()
                    self.lost_navx = False
                angle_error = self.angle_pid.calculate(
                    self.get_angle(), self.target_angle
                )
                angle_error = max(min(angle_error, 2), -2)
                # print(f"{self.get_angle():.3f}, {self.target_angle:.3f}, {angle_error:.3f}")
            else:
                self.lost_navx = True
                return
            self._requested_vectors["rcw"] = angle_error

        # Ne fais rien si les vecteurs sont trop petits
        if (
            self._requested_vectors["strafe"] == 0
            and self._requested_vectors["fwd"] == 0
            and self.request_wheel_lock
        ):
            # On remets à zéro la vitesse
            self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)
            # Place les roues à 45 degrées, utile pour la défense
            self._requested_angles["front_left"] = 45
            self._requested_angles["front_right"] = -45
            self._requested_angles["rear_left"] = -45
            self._requested_angles["rear_right"] = 45
            self.request_wheel_lock = False
            return

        fwdSpeed = self._requested_vectors["fwd"]
        strafeSpeed = -self._requested_vectors["strafe"]
        if abs(self._requested_vectors["rcw"]) <= 0.002:
            self._requested_vectors["rcw"] = 0
        rot = self._requested_vectors["rcw"]
        fwdSpeed = self.fwd_limiter.calculate(fwdSpeed)
        strafeSpeed = self.strafe_limiter.calculate(strafeSpeed)
        self.nt.putNumber("debug/fwdSpeed", fwdSpeed)
        self.nt.putNumber("debug/strageSpeed", strafeSpeed)
        if self.field_centric:
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
        self._requested_vectors["fwd"] = 0.0
        self._requested_vectors["strafe"] = 0.0
        self._requested_vectors["rcw"] = 0.0

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

    def updateOdometry(self) -> None:
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
            targetStates, MAX_MODULE_SPEED
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
        self.updateOdometry()
        self.compute_move()
        self.controller_forward = 0
        self.controller_strafe = 0
        self.controller_angle_stick_x = 0
        self.controller_angle_stick_y = 0
        self.automove_forward = 0
        self.automove_strafe = 0
        self.automove_strength = 0

        # Calcul des vecteurs
        self._calculate_vectors()

        # Remets les vitesse à zéro
        self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)
        self.request_wheel_lock = False

        self.update_nt()
