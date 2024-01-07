import math
from dataclasses import dataclass
import ntcore
from components import swervemodule
from navx import AHRS
from wpimath.controller import PIDController


@dataclass
class SwerveDriveConfig:
    base_width: float
    base_length: float
    enable_debug: bool


class SwerveDrive:
    # Modules injectés depuis le code de ../robot.py
    cfg: SwerveDriveConfig
    frontLeftModule: swervemodule.SwerveModule
    frontRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule
    navx: AHRS
    nt: ntcore.NetworkTable

    lower_input_thresh = 0.1
    rotation_multiplier = 180
    xy_multiplier = 1
    NavxYawZero = 0
    NavxPitchZero = 0
    NavxRollZero = 0
    zero_done = False
    debug = False

    def setup(self):
        """
        Appelé après l'injection
        """

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

        self.debug = self.cfg.enable_debug

        for key in self.modules:
            self.modules[key].enable_debug(self.debug)

        self.lost_navx = False

        self.target_angle = 0
        self.request_wheel_lock = False

        self.angle_pid = PIDController(1, 0, 0)
        self.angle_pid.enableContinuousInput(0, 360)
        self.angle_pid.setTolerance(-2, 2)

        # Pid de rotation. Ajuster via le ShuffleBoard et écrire la nouvelle valeur ici
        self.nt.putNumber("swerve/angle_pid/Kp", 0.008)
        self.nt.putNumber("swerve/angle_pid/Ki", 0)
        self.nt.putNumber("swerve/angle_pid/Kd", 0)

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
        self.NavxYawZero = self.navx.getYaw()
        self.target_angle = self.get_angle()

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
        self.get_nt_config()
        for key in self.modules:
            self.modules[key].get_nt_config()

    def get_nt_config(self):
        self.angle_pid.setP(self.nt.getNumber("swerve/angle_pid/Kp", 0))
        self.angle_pid.setI(self.nt.getNumber("swerve/angle_pid/Ki", 0))
        self.angle_pid.setD(self.nt.getNumber("swerve/angle_pid/Kd", 0))

    def get_angle(self):
        """
        Retourne l'angle absolue basée sur le zéro
        De 0 à 360 degrée
        """
        return (self.navx.getYaw() - self.NavxYawZero + 180) % 360

    def set_fwd(self, fwd):
        """
        Écrit le vecteur 'forward'
        :param fwd: Valeur de -1 à 1
        """
        fwd *= self.xy_multiplier
        self._requested_vectors["fwd"] = fwd

    def set_strafe(self, strafe):
        """
        Écrit la valeur 'strafe'
        :param strafe: Valeur de -1 to 1
        """
        strafe *= self.xy_multiplier
        self._requested_vectors["strafe"] = strafe

    def set_angle(self, angle):
        """Écrit l'angle absolue à atteindre de 0 à 360"""
        self.target_angle = angle

    def _rotate_robot(self, rcw):
        """
        ROTATION NON FIELD CENTRIC, NE PAS UTILISER!
        Écrit la valeur de rotation
        :param rcw: Valeur de -1 à 1
        """
        rcw *= self.rotation_multiplier
        self._requested_vectors["rcw"] = rcw

    def move(self, forward, strafe):
        """Fait bouger le robot, utile pour les modes autonomes"""
        self.set_fwd(forward)
        self.set_strafe(strafe)

    def controller_move(self, forward, strafe, angle_stick_x, angle_stick_y):
        """Fait bouger le robot avec un contrôleur"""
        forward = self.square_input(-forward)
        strafe = self.square_input(strafe)

        if abs(forward) < self.lower_input_thresh:
            forward = 0

        if abs(strafe) < self.lower_input_thresh:
            strafe = 0

        self.set_fwd(forward)
        self.set_strafe(strafe)

        # Rotation non field centric, ne pas utiliser!
        # self._rotate_robot(angle_stick_x)

        # Élimite la zone morte du joystick (petits déplacements)
        if math.sqrt(angle_stick_x**2 + angle_stick_y**2) > 0.5:
            angle = (math.degrees(math.atan2(angle_stick_y, angle_stick_x)) + 360) % 360
            angle += 90
            angle %= 360
            self.set_angle(angle)

    def snap_angle_nearest_180(self):
        """Force le robot à regarder vers l'avant ou l'arrière"""
        angle_deg = self.get_angle()
        remainder = angle_deg % 180
        if remainder < 90:
            rounded_angle_deg = angle_deg // 180 * 180
        else:
            rounded_angle_deg = (angle_deg // 180 + 1) * 180
        self.set_angle(rounded_angle_deg % 360)

    def _round_angle_nearest_90(self):
        """Force le robot à regarder vers le côté le plus prêt"""
        angle_deg = self.get_angle()
        remainder = angle_deg % 90
        if remainder < 45:
            rounded_angle_deg = angle_deg // 90 * 90
        else:
            rounded_angle_deg = (angle_deg // 90 + 1) * 90
        self.set_angle(rounded_angle_deg % 360)

    def _calculate_vectors(self):
        """
        Réalise le calcul des angles et vitesses pour chaque swerve module
        en fonction des commandes reçues
        """

        if self.navx.isConnected() is True:
            if self.lost_navx is True:
                self.angle_pid.reset()
                self.target_angle = self.get_angle()
                self.lost_navx = False
            angle_error = self.angle_pid.calculate(self.get_angle(), self.target_angle)
            angle_error = max(min(angle_error, 1), -1)
        else:
            self.lost_navx = True
            return

        (
            self._requested_vectors["fwd"],
            self._requested_vectors["strafe"],
            self._requested_vectors["rcw"],
        ) = self.normalize(
            [
                self._requested_vectors["fwd"],
                self._requested_vectors["strafe"],
                angle_error,
            ]
        )

        # Ne fais rien si les vecteurs sont trop petits
        if (
            self._requested_vectors["rcw"] == 0
            and self._requested_vectors["strafe"] == 0
            and self._requested_vectors["fwd"] == 0
        ):
            self._requested_speeds = dict.fromkeys(
                self._requested_speeds, 0
            )  # On remets à zéro la vitesse, mais pas l'angle

            if self.request_wheel_lock:
                # Place les roues à 45 degrées, utile pour la défense
                self._requested_angles["front_left"] = 45
                self._requested_angles["front_right"] = -45
                self._requested_angles["rear_left"] = -45
                self._requested_angles["rear_right"] = 45
                self.request_wheel_lock = False
            return

        # Convertis les vecteurs pour qu'ils soient field centric
        navxYaw = self.get_angle() / 360 * 2 * math.pi
        temp = self._requested_vectors["fwd"] * math.cos(
            navxYaw
        ) + self._requested_vectors["strafe"] * math.sin(navxYaw)
        self._requested_vectors["strafe"] = -self._requested_vectors["fwd"] * math.sin(
            navxYaw
        ) + self._requested_vectors["strafe"] * math.cos(navxYaw)
        self._requested_vectors["fwd"] = temp
        # Où 0 est l'angle du gyro, measusé en sens horaire à partir du zéro (zéro pointant vers le bas du terrain):

        ratio = math.hypot(self.cfg.base_length, self.cfg.base_width)

        # Calcul des vitesses par quadrant
        A = self._requested_vectors["strafe"] - (
            self._requested_vectors["rcw"] * (self.cfg.base_length / ratio)
        )
        B = self._requested_vectors["strafe"] + (
            self._requested_vectors["rcw"] * (self.cfg.base_length / ratio)
        )
        C = self._requested_vectors["fwd"] - (
            self._requested_vectors["rcw"] * (self.cfg.base_width / ratio)
        )
        D = self._requested_vectors["fwd"] + (
            self._requested_vectors["rcw"] * (self.cfg.base_width / ratio)
        )

        # Calcul des vitesses et angle pour chaque roues
        frontRight_speed = math.hypot(B, C) * 1
        frontRight_angle = math.degrees(math.atan2(B, C))

        frontLeft_speed = math.hypot(B, D) * 1
        frontLeft_angle = math.degrees(math.atan2(B, D))

        rearLeft_speed = math.hypot(A, D) * 1
        rearLeft_angle = math.degrees(math.atan2(A, D))

        rearRight_speed = math.hypot(A, C) * 1
        rearRight_angle = math.degrees(math.atan2(A, C))

        self._requested_speeds["front_left"] = frontLeft_speed
        self._requested_speeds["front_right"] = frontRight_speed
        self._requested_speeds["rear_left"] = rearLeft_speed
        self._requested_speeds["rear_right"] = rearRight_speed

        self._requested_angles["front_left"] = frontLeft_angle
        self._requested_angles["front_right"] = frontRight_angle
        self._requested_angles["rear_left"] = rearLeft_angle
        self._requested_angles["rear_right"] = rearRight_angle

        self._requested_speeds = self.normalizeDictionary(self._requested_speeds)

        # Remise à zéro des valeurs demandées par sécurité
        self._requested_vectors["fwd"] = 0.0
        self._requested_vectors["strafe"] = 0.0
        self._requested_vectors["rcw"] = 0.0

    def enable_debug(self, enable_debug, debug_modules=False):
        """
        Affiche des information de débogage
        """
        self.debug = enable_debug

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

    def execute(self):
        """
        Calcul et transmet la commande de vitesse et d'angle à chaque swerve module.
        """
        # Calcul des vecteurs
        self._calculate_vectors()

        # Écrit le résultat dans chaque swerve module
        for key in self.modules:
            self.modules[key].move(
                self._requested_speeds[key], self._requested_angles[key]
            )

        # Remets les vitesse à zéro
        self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)

        self.update_nt()
