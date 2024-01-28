#!/usr/bin/env python3

"""
Patron pour une base pilotable Swerve Drive

Un NAVX est nécessaire, car le positionnement est absolue (field centric).
Le zéro est fait au démarrage du robot, donc il est important de bien le positionner.
Presser le bouton A va également refaire son zéro.

INTAKE
    Moteur entrée
    Moteur sortie
    Beam Sensor

BRAS
    Moteur Tête
       Encodeur Abs
    Moteur Bras
        Limit Switch
        Encodeur Abs
        Piston

SWERVES (FALCONS x4)
PIXIE
LIME LIGHT

GRIMPEUR
    2x Moteurs (NEO)
    2x Limit Switch

INDICATEUR LUMINEUX

"""

import math

import ntcore
import phoenix6
import rev
import wpilib
from magicbot import MagicRobot, feedback
from navx import AHRS
from wpimath.geometry import Rotation2d

import constants
from common import arduino_light, limelight
from components import (Intake, LoBras, LoBrasArm, LoBrasArmFollower,
                        LoBrasHead, Shooter, ShooterFollower, robot_actions,
                        swervedrive, swervemodule)


class MyRobot(MagicRobot):
    """
    Après avoir créer les 'components' de bas niveau, tel que 'drivetrain' ou 'intake', utiliser leur nom suivi d'un trait souligné (_)
    pour injecter des objets au composant.

    ex.:
    Après avoir créer dans 'components' un fichier intake_driver.py, utiliser une variable nommée "intake_beltMotor: phoenix6.hardware.TalonFX" déclare le type de la variable.
    Quand 'beltMotor' sera appelée depuis 'intake', ce sera un objet de type 'WPI_TalonFX'.

    Utiliser un signe égale (=) pendant la déclaration des variables tel que "intake_beltMotor = phoenix6.hardware.TalonFX(11)" crée l'objet.
    Quand 'beltMotor' est appelé depuis le composant 'intake' et ce sera un WPI_TalonFX avec un ID CAN de 11.

    Utilisez le signe = dans la fonction 'createObjects' pour vous assurer que les données sont biens transmises à leur composantes.

    Pour plus d'information: https://robotpy.readthedocs.io/en/stable/frameworks/magicbot.html
    """

    # Swerve Drive
    # Ici on crée un composant par swerve (swervemodule), suivi d'un dernier composant qui regroupe l'ensemble de toutes les swerves (swervedrive)
    # `inverted` doit être ajusté si le moteur tourne dans la mauvaise direction
    # `allow_reverse` est utiliser pour changer la direction de rotation d'un moteur plutôt que de faire une rotation de plus de 180 degrée
    # `nt_name` sera le nom sous lequel sera groupé les données des modules dans les networktables
    frontLeftModule: swervemodule.SwerveModule
    frontRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule
    drivetrain: swervedrive.SwerveDrive

    # I HATE MAGICBOT AND ITS STUPID INJECTION
    lobras_arm: LoBrasArm
    lobras_arm_follower: LoBrasArmFollower
    lobras_head: LoBrasHead
    lobras: LoBras

    shooter: Shooter
    shooter_follower: ShooterFollower
    intake: Intake

    navx: AHRS
    robot_actions: robot_actions.RobotActions

    # Networktables pour de la configuration et retour d'information
    nt: ntcore.NetworkTable

    def createObjects(self):
        """
        C'est ici que les composants sont vraiment créé avec le signe =.
        Les composants avec un préfix connu tel que "intake_" vont être injectés.
        """
        # NetworkTable
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")

        self.limelight_intake = limelight.Limelight("limelight")
        self.limelight_shoot = limelight.Limelight("limelight-shoot")
        self.arduino_light = arduino_light.I2CArduinoLight(wpilib.I2C.Port.kMXP, 0x42)
        self.status_light = wpilib.Solenoid(10, wpilib.PneumaticsModuleType.CTREPCM, 1)

        # Configuration de la base swerve
        self.initSwerve()

        # General
        self.gamepad1 = wpilib.XboxController(0)
        # self.gamepad1 = wpilib.PS5Controller(0)
        self.pdp = wpilib.PowerDistribution()
        self.digitaltest = wpilib.DigitalInput(9)

    @feedback
    def getDigitalIO(self):
        return self.digitaltest.get()

    def initSwerve(self):
        """
        Configuration de la base Swerve Drive
        """
        # On assigne nos moteurs à nos swerve
        # Il est important d'utiliser le logiciel de la compagnie pour trouver (ou configurer) les CAN id
        # On utilise également les encodeurs absolues CAN pour orienter la roue
        self.drivetrain_cfg = swervedrive.SwerveDriveConfig(
            field_centric=True,
            base_width=27,
            base_length=19,
            is_simulation=self.isSimulation(),
        )

        self.frontLeftModule_driveMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_DRIVE_FL
        )
        self.frontLeftModule_rotateMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_ROTATE_FL
        )
        self.frontLeftModule_encoder = phoenix6.hardware.CANcoder(
            constants.CANIds.SWERVE_CANCODER_FL
        )
        self.frontLeftModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="frontLeftModule",
            inverted=False,
            allow_reverse=True,
            is_simulation=self.isSimulation(),
            rotation_zero=193,
        )

        self.frontRightModule_driveMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_DRIVE_FR
        )
        self.frontRightModule_rotateMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_ROTATE_FR
        )
        self.frontRightModule_encoder = phoenix6.hardware.CANcoder(
            constants.CANIds.SWERVE_CANCODER_FR
        )
        self.frontRightModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="frontRightModule",
            inverted=True,
            allow_reverse=True,
            is_simulation=self.isSimulation(),
            rotation_zero=76,
        )

        self.rearLeftModule_driveMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_DRIVE_RL
        )
        self.rearLeftModule_rotateMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_ROTATE_RL
        )
        self.rearLeftModule_encoder = phoenix6.hardware.CANcoder(
            constants.CANIds.SWERVE_CANCODER_RL
        )
        self.rearLeftModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="rearLeftModule",
            inverted=True,
            allow_reverse=True,
            is_simulation=self.isSimulation(),
            rotation_zero=216,
        )

        self.rearRightModule_driveMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_DRIVE_RR
        )
        self.rearRightModule_rotateMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_ROTATE_RR
        )
        self.rearRightModule_encoder = phoenix6.hardware.CANcoder(
            constants.CANIds.SWERVE_CANCODER_RR
        )
        self.rearRightModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="rearRightModule",
            inverted=False,
            allow_reverse=True,
            is_simulation=self.isSimulation(),
            rotation_zero=318,
        )

        # Et le navx nécessaire pour un control "Field Centric"
        self.navx = AHRS.create_spi(update_rate_hz=50)

    def disabledPeriodic(self):
        """Mets à jours le dashboard, même quand le robot est désactivé"""
        self.update_nt()

    def autonomousInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode autonome."""
        self.drivetrain.init()

    def autonomous(self):
        """Pour les modes auto de MagicBot, voir le dossier ./autonomous"""
        super().autonomous()

    def teleopInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode téléopéré."""
        self.drivetrain.init()
        self.arduino_light.set_RGB(0, 0, 0)
        self.status_light.set(0)

    def teleopPeriodic(self):
        """Cette fonction est appelée de façon périodique lors du mode téléopéré."""

        # self.drivetrain.set_controller_values(
        #     self.gamepad1.getLeftY(),
        #     self.gamepad1.getLeftX(),
        #     self.gamepad1.getRightX(),
        #     self.gamepad1.getRightY(),
        # )

        # Reset navx zero
        if self.gamepad1.getXButton():
            self.drivetrain.navx_zero_angle()

        # if self.gamepad1.getL1Button():  # getLeftBumper():
        #     self.drivetrain.request_wheel_lock = True

        # if self.gamepad1.getSquareButton():  # getAButton():
        #     self.robot_actions.autointake_with_limelight()
        joystickx = self.gamepad1.getLeftX() * 360

        if self.gamepad1.getAButton():  # getBButton():
            self.lobras.set_arm_angle(joystickx)
        if self.gamepad1.getXButton():  # getBButton():
            self.lobras.set_head_angle(joystickx)
        if self.gamepad1.getBButton():  # getYButton():
            self.lobras.set_angle(0, 0)
        if self.gamepad1.getYButton():  # getYButton():
            self.lobras.set_angle(100, 180)
            self.shooter.enable_shooter()
        if self.gamepad1.getLeftBumper():
            self.shooter.enable_shooter()
        if self.gamepad1.getRightBumper():
            self.shooter.disable_shooter()
            # self.robot_actions.auto_test()
        # else:
        #    self.robot_actions.reset_auto()
        # self.robot_actions.retract()

    def update_nt(self):
        """Affiche les données sur le ShuffleBoard"""
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
