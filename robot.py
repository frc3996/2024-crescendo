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

import ntcore
import phoenix6
import wpilib
from magicbot import MagicRobot
from magicbot import tunable
from wpimath.geometry import Rotation2d
from navx import AHRS
import rev
import constants
from autonomous.auto_modes import RunAuto
from common import arduino_light, tools
from components.climber import Climber, ClimberFollower
from components.field import FieldLayout
from components.intake import Intake
from components.limelight import LimeLightVision
from components.lobra import LoBrasArm, LoBrasArmFollower, LoBrasHead
from components.pixy import Pixy
from components.robot_actions import (ActionDewinch, ActionGrabAuto,
                                      ActionHighShootAuto, ActionLowShootAuto,
                                      ActionLowShootTune, ActionPathTester,
                                      ActionShootAmpAssisted,
                                      ActionShootAmpAuto, ActionStow,
                                      ActionWinch)
from components.shooter import Shooter, ShooterFollower, ShooterMain
from components.swervedrive import SwerveDrive, SwerveDriveConfig
from components.swervemodule import SwerveModule, SwerveModuleConfig


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

    runAuto: RunAuto

    # HIGH Level components first (components that use components)
    actionGrabAuto: ActionGrabAuto
    actionStow: ActionStow
    actionLowShootAuto: ActionLowShootAuto
    actionHighShootAuto: ActionHighShootAuto
    actionShootAmpAssisted: ActionShootAmpAssisted
    actionShootAmpAuto: ActionShootAmpAuto
    actionDewinch: ActionDewinch
    actionWinch: ActionWinch
    actionPathTester: ActionPathTester
    actionLowShootTune: ActionLowShootTune

    # LOW Level components after

    # NAVX
    navx: AHRS

    # Lobras
    lobras_arm: LoBrasArm
    lobras_arm_follower: LoBrasArmFollower
    lobras_head: LoBrasHead

    # SwerveDrive
    frontLeftModule: SwerveModule
    frontRightModule: SwerveModule
    rearLeftModule: SwerveModule
    rearRightModule: SwerveModule
    drivetrain: SwerveDrive

    # Shooter
    shooter: Shooter
    shooter_main: ShooterMain
    shooter_follower: ShooterFollower

    # Climber
    climber: Climber
    climber_follower: ClimberFollower

    # Intake
    intake: Intake

    # Pixy
    pixy: Pixy

    # FieldLayout
    field_layout: FieldLayout

    # XXX: Re-Enable vision after we're done testing
    limelight_vision: LimeLightVision

    # Networktables pour de la configuration et retour d'information
    nt: ntcore.NetworkTable
    is_sim: bool
    fast_climb_arm_angle = tunable(125)

    def createObjects(self):
        """
        C'est ici que les composants sont vraiment créé avec le signe =.
        Les composants avec un préfix connu tel que "intake_" vont être injectés.
        """
        # NetworkTable
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")
        self.is_sim = self.isSimulation()

        self.arduino_light = arduino_light.I2CArduinoLight(wpilib.I2C.Port.kMXP, 0x42)

        # OMFG. The fuck is wrong with you. What have we ever done yo you to deserve this
        # There is a place for such as you in hell. I wish you harm :)
        # NOTE: Remove comment to increase power over 9000
        # self.status_light = wpilib.Solenoid(10, wpilib.PneumaticsModuleType.CTREPCM, 1)

        # NAVX
        # self.navx = AHRS.create_spi(update_rate_hz=50)
        self.navx = AHRS(wpilib.SerialPort.Port.kMXP)

        # Configuration de la base swerve
        self.initSwerve()

        # General
        self.gamepad = wpilib.XboxController(0)
        self.gamepad2 = wpilib.XboxController(1)
        # self.gamepad = wpilib.PS5Controller(0)
        self.pdp = wpilib.PowerDistribution(1, wpilib.PowerDistribution.ModuleType.kRev)
        self.pdp.clearStickyFaults()

        # What's this?
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def initSwerve(self):
        """
        Configuration de la base Swerve Drive
        """
        # On assigne nos moteurs à nos swerve
        # Il est important d'utiliser le logiciel de la compagnie pour trouver (ou configurer) les CAN id
        # On utilise également les encodeurs absolues CAN pour orienter la roue
        self.drivetrain_cfg = SwerveDriveConfig(
            base_width=20.75,
            base_length=22.75,
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
        self.frontLeftModule_cfg = SwerveModuleConfig(
            nt_name="frontLeftModule",
            inverted=False,
            allow_reverse=True,
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
        self.frontRightModule_cfg = SwerveModuleConfig(
            nt_name="frontRightModule",
            inverted=True,
            allow_reverse=True,
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
        self.rearLeftModule_cfg = SwerveModuleConfig(
            nt_name="rearLeftModule",
            inverted=True,
            allow_reverse=True,
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
        self.rearRightModule_cfg = SwerveModuleConfig(
            nt_name="rearRightModule",
            inverted=False,
            allow_reverse=True,
            rotation_zero=318,
        )

    def disabledPeriodic(self):
        """Mets à jours le dashboard, même quand le robot est désactivé"""
        pass
        # self.limelight_vision.execute()

    def autonomousInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode autonome."""
        self.actionStow.done()
        self.limelight_vision.light_off()
        pass

    def autonomous(self):
        """Pour les modes auto de MagicBot, voir le dossier ./autonomous"""
        super().autonomous()

    def teleopInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot en
        tre en mode téléopéré."""
        self.pdp.clearStickyFaults()
        self.limelight_vision.light_off()
        self.arduino_light.set_RGB(0, 0, 0)
        self.actionStow.engage()
        self.drivetrain.permanent_snap = False

    def teleopPeriodic(self):
        """Cette fonction est appelée de façon périodique lors du mode téléopéré."""

        self.drivetrain.set_controller_values(
            self.gamepad.getLeftY(),
            self.gamepad.getLeftX(),
            self.gamepad.getRightX(),
            self.gamepad.getRightY(),
        )

        speed_factor_when_arm_high = tools.map_value(self.lobras_arm.get_angle(), 0, 45, 1, 0.2)
        speed_factor_when_arm_high = tools.fit_to_boundaries(speed_factor_when_arm_high, 0.4, 1)
        self.drivetrain.set_tmp_speed_factor(factor_movement=speed_factor_when_arm_high, factor_rotation=speed_factor_when_arm_high)

        ## XXX: This is not needed as limelight is resetting zeo for us
        # # Reset navx zero
        # if self.gamepad1.getRightStickButton():
        #     self.drivetrain.navx_zero_angle()
        if self.actionStow.is_executing:
            return

        if self.gamepad.getRightTriggerAxis() > 0.75:
            self.actionGrabAuto.engage()
            pass
        elif self.gamepad.getRightBumper():
            self.actionShootAmpAssisted.engage()
            pass
        elif self.gamepad.getLeftTriggerAxis() > 0.75:
            self.drivetrain.set_tmp_speed_factor(factor_movement=0.2)
            self.actionLowShootAuto.engage()
            pass
        elif self.gamepad.getLeftBumper():
            self.actionHighShootAuto.engage()
            pass
        elif self.gamepad.getBButtonPressed():
            self.intake.outtake()
            pass
        elif self.gamepad.getBButtonReleased():
            self.intake.disable()
            pass
        elif self.gamepad.getXButton():
            self.actionPathTester.engage()
            pass
        elif self.gamepad.getYButton():
            self.lobras_arm.set_angle(self.fast_climb_arm_angle)
            pass
        elif self.gamepad.getAButton():
            self.lobras_arm.set_angle(0)
            pass
        # else:
        #     # NOW CALLED FROM OTHER ACTION, WHEN NEEDED
        #     self.actionStow.engage()

        # Gamepad 2
        if self.gamepad2.getXButton():
            angle = Rotation2d.fromDegrees(-60.69 + 180) if tools.is_red() else Rotation2d.fromDegrees(-60.69)
            self.drivetrain.snap_angle(angle)
        elif self.gamepad2.getYButton():
            angle = Rotation2d.fromDegrees(180 + 180) if tools.is_red() else Rotation2d.fromDegrees(180)
            self.drivetrain.snap_angle(angle)
        elif self.gamepad2.getBButton():
            angle = Rotation2d.fromDegrees(60.69 + 180) if tools.is_red() else Rotation2d.fromDegrees(60.69)
            self.drivetrain.snap_angle(angle)
        elif self.gamepad2.getBButton():
            angle = Rotation2d.fromDegrees(60.69 + 180) if tools.is_red() else Rotation2d.fromDegrees(60.69)
            self.drivetrain.snap_angle(angle)
