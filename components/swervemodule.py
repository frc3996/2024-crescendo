from dataclasses import dataclass

import phoenix6
import ntcore
import wpimath
from wpimath.controller import PIDController
from wpimath import kinematics, units, geometry
import math


# Classe de configuration des swerve
@dataclass
class SwerveModuleConfig:
    nt_name: str
    inverted: bool
    allow_reverse: bool


class SwerveModule:
    # Vas chercher moteur, encodeur et configuration par injection
    driveMotor: phoenix6.hardware.TalonFX
    rotateMotor: phoenix6.hardware.TalonFX
    encoder: phoenix6.hardware.CANcoder
    cfg: SwerveModuleConfig

    def setup(self):
        """
        Appelé après l'injection
        """
        # General
        # meter_per_second = (rps / gear_ratio) * wheel_circumference_meter
        # rps = meter_per_second * gear_ratio / wheel_circumference_meter
        fudge_factor = 1
        wheel_circumference_meter = math.pi * units.inchesToMeters(4.0)
        wheel_gear_ratio = 8.14  # L1=8.14; L2=6.75; L3=6.12
        self.velocity_to_rps_conversion_factor = wheel_gear_ratio / wheel_circumference_meter * fudge_factor
        self.currentPosition = kinematics.SwerveModulePosition()
        self.currentState = kinematics.SwerveModuleState()
        self.lastPosition = 0
        self.debug = False
        self.calibration_mode = 0

        # Network Table
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")
        self.nt.putBoolean(f"{self.cfg.nt_name}/debug", False)
        self.nt.putNumber("swerve/rotation_pid/Kp", 0.008)
        self.nt.putNumber("swerve/rotation_pid/Ki", 0)
        self.nt.putNumber("swerve/rotation_pid/Kd", 0)

        # Drive Motor
        config = phoenix6.configs.TalonFXConfiguration()
        config.open_loop_ramps = phoenix6.configs.OpenLoopRampsConfigs().with_duty_cycle_open_loop_ramp_period(0.1)
        config.slot0.k_p = 0.08
        config.slot0.k_i = 0.0
        config.slot0.k_d = 0.0001
        config.slot0.k_v = 0.12
        config.voltage.peak_forward_voltage = 8
        config.voltage.peak_forward_voltage = -8
        motor_config = phoenix6.configs.MotorOutputConfigs()
        motor_config.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
        config.motor_output = motor_config
        self.driveMotor.configurator.apply(config)
        self.driveMotor_control = phoenix6.controls.VelocityVoltage(0, 0, True, 0, 0, False, False, False)
        self._requested_speed = 0

        # Rotation Motor
        config = phoenix6.configs.TalonFXConfiguration()
        config.open_loop_ramps = phoenix6.configs.OpenLoopRampsConfigs().with_duty_cycle_open_loop_ramp_period(0.01)
        motor_config = phoenix6.configs.MotorOutputConfigs()
        motor_config.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
        config.motor_output = motor_config
        self.rotateMotor.configurator.apply(config)
        self._requested_degree = 0
        self.rotateMotor_control = phoenix6.controls.DutyCycleOut(0)
        self.rotation_pid = PIDController(0, 0, 0)  # PID configuré via le ShuffleBoard
        self.rotation_pid.enableContinuousInput(
            0, 360
        )  # 0 et 360 sont considérés comme la même valeur
        cancoder_config = phoenix6.configs.CANcoderConfiguration()
        self.encoder.configurator.apply(cancoder_config)
        self.encoder_zero = 0

    def refresh_nt_config(self):
        self.rotation_pid.setP(self.nt.getNumber("swerve/rotation_pid/Kp", 0))
        self.rotation_pid.setI(self.nt.getNumber("swerve/rotation_pid/Ki", 0))
        self.rotation_pid.setD(self.nt.getNumber("swerve/rotation_pid/Kd", 0))
        self.debug = self.nt.getBoolean(f"{self.cfg.nt_name}/debug", False)
        self.encoder_zero = self.nt.getNumber(f"{self.cfg.nt_name}/rotation_zero", 0)
        self.calibration_mode = self.nt.getNumber("config/zero_calibration_mode", 0)

    def get_encoder_zero(self):
        """Retourne la valeur de zéro configuré"""
        if self.calibration_mode == 1:
            self.encoder_zero = self.nt.getNumber(f"{self.cfg.nt_name}/rotation_zero", 0)
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
        abs_pos = (self.encoder.get_absolute_position().value + 0.5) * 360
        computed_value = (abs_pos + self.get_encoder_zero()) % 360
        return computed_value

    def kinematic_move(self):
        """
        Spécifie la vitesse et l'angle requise.
        :param speed: Vitesse de la roue, de -1 à 1
        :param deg: Angle de la roue, de 0 à 359
        """
        deg = self.currentState.angle.degrees() + 360
        speed = self.currentState.speed
        deg %= 360  # Empêche les valeurs au dessus de 359 degrée

        if self.cfg.allow_reverse:
            # Si la différence d'angle est de plus de 90 degrée, on inverse la vitesse et pivote moins
            offset = abs(deg - self.get_encoder_abs_position())
            if offset > 90 and offset < 270:
                speed *= -1
                deg += 180
                deg %= 360

        self._requested_degree = deg
        self._requested_speed = speed

    def resetPose(self):
        self.currentPosition = kinematics.SwerveModulePosition(0, self.currentPosition.angle)

    def setTargetState(self, targetState):
        self.currentState = kinematics.SwerveModuleState.optimize(targetState, self.currentState.angle)
        self.currentPosition = kinematics.SwerveModulePosition(self.currentPosition.distance + (self.currentState.speed * 0.02), self.currentState.angle)

    def getState(self):
        """
        For PathPlannerLib
        Return Swerve module state
        """
        return self.currentState

    def getActualPosition(self):
        """
        For PathPlannerLib
        Return Swerve module position
        """
        current_position = self.driveMotor.get_position().value / self.velocity_to_rps_conversion_factor
        current_angle = geometry.Rotation2d.fromDegrees(self.get_encoder_abs_position() - 180)
        # print(f"{self.cfg.nt_name} rotation abs position: {self.get_encoder_abs_position()}")
        return kinematics.SwerveModulePosition(current_position, current_angle)

    def getPosition(self):
        """
        For PathPlannerLib
        Return Swerve module position
        """

        return self.currentPosition

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

    def execute(self):
        """
        Utilise le PID pour se rendre à la position demandée.
        Modifie la puissance des moteurs pour celle demandée.

        Appelé à chaque itération/boucle
        """
        # Calcul de l'angle avec le PID
        self.kinematic_move()

        # Commande d'angle
        if self.calibration_mode == 1:
            self._requested_degree = 0
        error = self.rotation_pid.calculate(
            self.get_encoder_abs_position(), self._requested_degree
        )
        self.rotateMotor_control.output = max(min(error, 1), -1)
        self.rotateMotor.set_control(self.rotateMotor_control)


        # Commande de vitesse au moteur, conversion de la vitesse en RPM
        if self.calibration_mode == 1:
            self._requested_speed = 0.05
        rps = self._requested_speed * self.velocity_to_rps_conversion_factor
        self.driveMotor.set_control(self.driveMotor_control.with_velocity(rps))
        self._requested_speed = 0

        self.update_nt()
