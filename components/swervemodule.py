from dataclasses import dataclass

import phoenix5
import ntcore
from wpimath.controller import PIDController


# Classe de configuration des swerve
@dataclass
class SwerveModuleConfig:
    nt_name: str
    inverted: bool
    allow_reverse: bool


class SwerveModule:
    # Vas chercher moteur, encodeur et configuration par injection
    driveMotor: phoenix5.WPI_TalonFX
    rotateMotor: phoenix5.WPI_TalonFX
    encoder: phoenix5._ctre.sensors.WPI_CANCoder
    cfg: SwerveModuleConfig

    def setup(self):
        """
        Appelé après l'injection
        """
        self.lastPosition = 0
        self.debug = False
        self.driveMotor.setInverted(self.cfg.inverted)

        # Accélération maximale du moteur de 0 à 100% en 0.1 seconde
        self.driveMotor.configOpenloopRamp(0.1)

        self._requested_degree = 0
        self._requested_speed = 0

        self.rotation_pid = PIDController(0, 0, 0)  # PID configuré via le ShuffleBoard
        self.rotation_pid.enableContinuousInput(
            0, 360
        )  # 0 et 360 sont considérés comme la même valeur

        self.encoder_zero = 0
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")
        self.nt.putNumber("swerve/rotation_pid/Kp", 0.008)
        self.nt.putNumber("swerve/rotation_pid/Ki", 0)
        self.nt.putNumber("swerve/rotation_pid/Kd", 0)

    def enable_debug(self, enable_debug: bool):
        self.debug = enable_debug

    def get_nt_config(self):
        self.rotation_pid.setP(self.nt.getNumber("swerve/rotation_pid/Kp", 0))
        self.rotation_pid.setI(self.nt.getNumber("swerve/rotation_pid/Ki", 0))
        self.rotation_pid.setD(self.nt.getNumber("swerve/rotation_pid/Kd", 0))
        self.encoder_zero = self.nt.getNumber(f"{self.cfg.nt_name}/rotation_zero", 0)

    def get_encoder_zero(self):
        """Retourne la valeur de zéro configuré"""
        return self.encoder_zero

    def flush(self):
        """
        Remets à Zéro l'angle et la vitesse demandé
        ainsi que le PID
        """
        self._requested_degree = self.get_encoder_zero()
        self._requested_speed = 0
        self.rotation_pid.reset()

    def get_encoder_abs_position(self):
        """Retourne la position actuelle de l'encodeur"""
        return (self.encoder.getAbsolutePosition() + self.get_encoder_zero()) % 360

    def move(self, speed, deg):
        """
        Spécifie la vitesse et l'angle requise.
        :param speed: Vitesse de la roue, de -1 à 1
        :param deg: Angle de la roue, de 0 à 359
        """
        deg %= 360  # Empêche les valeurs au dessus de 359 degrée

        if self.cfg.allow_reverse:
            # Si la différence d'angle est de plus de 90 degrée, on inverse la vitesse et pivote moins
            if abs(deg - self.get_encoder_abs_position()) > 90:
                speed *= -1
                deg += 180
                deg %= 360

        self._requested_speed = speed
        self._requested_degree = deg

    def execute(self):
        """
        Utilise le PID pour se rendre à la position demandée.
        Modifie la puissance des moteurs pour celle demandée.

        Appelé à chaque itération/boucle
        """
        # Calcul de l'angle avec le PID
        error = self.rotation_pid.calculate(
            self.get_encoder_abs_position(), self._requested_degree
        )
        output = max(min(error, 1), -1)
        self.rotateMotor.set(output)

        # Commande de vitesse au moteur
        self.driveMotor.set(self._requested_speed)

        self.update_nt()

    def update_nt(self):
        """
        Affiche des données pour du débogage
        """
        if self.debug:
            self.nt.putNumber(
                f"drive/{self.cfg.nt_name}/degrees", self.get_encoder_abs_position()
            )
            self.nt.putNumber(
                f"drive/{self.cfg.nt_name}_angle_error",
                self._requested_degree - self.get_encoder_abs_position(),
            )
            self.nt.putNumber(
                f"drive/{self.cfg.nt_name}/requested_degree", self._requested_degree
            )
            self.nt.putNumber(
                f"drive/{self.cfg.nt_name}/requested_speed", self._requested_speed
            )
