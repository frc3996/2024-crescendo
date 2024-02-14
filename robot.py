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
import wpilib
from magicbot import MagicRobot, feedback, tunable
from navx import AHRS
from wpimath.geometry import Pose2d, Rotation2d

import constants
from common import arduino_light, game
from common.tools import rescale_js
from components.chassis import ChassisComponent
from components.climber import Climber, ClimberFollower
from components.field import FieldLayout
from components.intake import Intake
from components.limelight import LimeLightVision
from components.lobra import LoBrasArm, LoBrasArmFollower, LoBrasHead
from components.pathplanner import PathPlanner
from components.pixy import Pixy
from components.robot_actions import (ActionDewinch, ActionDummy,
                                      ActionGrabAuto, ActionGrabManual,
                                      ActionHighShootAuto, ActionLowShoot,
                                      ActionLowShootAuto, ActionOuttake,
                                      ActionShoot, ActionShootAmp,
                                      ActionShootAmpAssisted,
                                      ActionShootAmpAuto, ActionStow,
                                      ActionWinch, FeedAndRetract)
from components.shooter import Shooter, ShooterFollower, ShooterMain
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

    # HIGH Level components first (components that use components)
    actionGrabAuto: ActionGrabAuto
    actionGrabManual: ActionGrabManual
    actionOuttake: ActionOuttake
    actionShoot: ActionShoot
    actionStow: ActionStow
    actionLowShoot: ActionLowShoot
    actionLowShootAuto: ActionLowShootAuto
    actionHighShootAuto: ActionHighShootAuto
    actionShootAmp: ActionShootAmp
    feedAndRetract: FeedAndRetract
    actionShootAmpAssisted: ActionShootAmpAssisted
    actionShootAmpAuto: ActionShootAmpAuto
    actionDewinch: ActionDewinch
    actionWinch: ActionWinch
    actionDummy: ActionDummy

    # LOW Level components after

    # NAVX
    navx: AHRS

    # Lobras
    lobras_arm: LoBrasArm
    lobras_arm_follower: LoBrasArmFollower
    lobras_head: LoBrasHead

    # SwerveDrive
    drivetrain: ChassisComponent
    pathplanner: PathPlanner

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
    # limelight_vision: LimeLightVision

    # Networktables pour de la configuration et retour d'information
    nt: ntcore.NetworkTable
    is_sim: bool

    max_speed = tunable(4)  # m/s

    def createObjects(self):
        """
        C'est ici que les composants sont vraiment créé avec le signe =.
        Les composants avec un préfix connu tel que "intake_" vont être injectés.
        """
        # NetworkTable
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")
        self.is_sim = self.isSimulation()

        # self.limelight_intake = limelight.LimeLightVision("limelight")
        # self.limelight_shoot = limelight.LimeLightVision("limelight-shoot")
        self.arduino_light = arduino_light.I2CArduinoLight(wpilib.I2C.Port.kMXP, 0x42)
        self.status_light = wpilib.Solenoid(10, wpilib.PneumaticsModuleType.CTREPCM, 1)

        # NAVX
        self.navx = AHRS.create_i2c(wpilib.I2C.Port.kMXP, update_rate_hz=50)

        # General
        self.keyboard0 = wpilib.Joystick(0)
        self.keyboard1 = wpilib.Joystick(1)
        self.gamepad1 = wpilib.XboxController(0)
        # self.gamepad1 = wpilib.PS5Controller(0)
        self.pdp = wpilib.PowerDistribution()

        # What's this?
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def disabledPeriodic(self):
        """Mets à jours le dashboard, même quand le robot est désactivé"""
        pass

    def autonomousInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode autonome."""
        pass

    def autonomous(self):
        """Pour les modes auto de MagicBot, voir le dossier ./autonomous"""
        super().autonomous()

    def teleopInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode téléopéré."""
        self.arduino_light.set_RGB(0, 0, 0)
        self.status_light.set(0)
        self.actionStow.engage()
        # self.drivetrain.resetPose(Pose2d(-0.038099999999999995, 5.547867999999999, 0))

    def drive(self):
        # Driving
        spin_rate = 4
        drive_x = -rescale_js(self.keyboard0.getRawAxis(0), 0.1) * self.max_speed
        drive_y = -rescale_js(self.keyboard0.getRawAxis(1), 0.1) * self.max_speed
        drive_z = (
            -rescale_js(self.keyboard1.getRawAxis(0), 0.1, exponential=2) * spin_rate
        )
        # local_driving = self.gamepad1.getYButton()

        if game.is_red():
            drive_x = -drive_x
            drive_y = -drive_y

        # if local_driving:
        # self.drivetrain.drive_local(drive_x, drive_y, drive_z)
        # else:
        self.drivetrain.drive_field(drive_x, drive_y, drive_z)

        # give rotational access to the driver
        if drive_z != 0:
            self.drivetrain.stop_snapping()

        dpad = self.gamepad1.getPOV()
        if dpad != -1:
            if game.is_red():
                self.drivetrain.snap_to_heading(-math.radians(dpad) + math.pi)
            else:
                self.drivetrain.snap_to_heading(-math.radians(dpad))

        # Set current robot direction to forward
        if self.gamepad1.getXButton():
            self.drivetrain.zero_yaw()

    def teleopPeriodic(self):
        """Cette fonction est appelée de façon périodique lors du mode téléopéré."""

        self.drive()

        # # Reset navx zero
        # if self.gamepad1.getRightStickButton():
        #     self.drivetrain.navx_zero_angle()

        if self.gamepad1.getRightTriggerAxis() > 0.75:
            self.actionGrabAuto.engage()
            pass
        elif self.gamepad1.getRightBumper():
            # self.drivetrain.set_tmp_speed_factor(0.5)
            self.actionShootAmpAssisted.engage()
            pass
        elif self.gamepad1.getLeftTriggerAxis() > 0.75:
            # self.drivetrain.set_tmp_speed_factor(0.5)
            self.actionLowShootAuto.engage()
            pass
        elif self.gamepad1.getLeftBumper():
            # self.drivetrain.set_tmp_speed_factor(0.5)
            self.actionHighShootAuto.engage()
            pass
        elif self.gamepad1.getAButton():
            self.actionWinch.engage()
            pass
        elif self.gamepad1.getBButton():
            self.actionOuttake.engage()
            pass
        elif self.gamepad1.getXButton():
            pass
        elif self.gamepad1.getYButton():
            self.actionDewinch.engage()
            pass
        # else:
        #     # NOW CALLED FROM OTHER ACTION, WHEN NEEDED
        #     self.actionStow.engage()
