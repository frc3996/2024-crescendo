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

import ntcore  # Outils pour les NetworkTables
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import phoenix6
from common import gamepad_helper as gh  # Outil pour faciliter l'utilisation des contrôleurs
from magicbot import MagicRobot

from navx import AHRS
from subsystems.drivetrain import Drivetrain, kMaxAngularSpeed, kMaxSpeed


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

    # Networktables pour de la configuration et retour d'information
    swerve: Drivetrain
    nt: ntcore.NetworkTable
    gyro: AHRS

    def createObjects(self):
        """
        C'est ici que les composants sont vraiment créé avec le signe =.
        Les composants avec un préfix connu tel que "intake_" vont être injectés.
        """
        # NetworkTable
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")

        # Et le navx nécessaire pour un control "Field Centric"
        self.gyro = AHRS.create_spi(update_rate_hz=50)

        # Configuration de la base swerve
        self.swerve = Drivetrain(self.gyro)

        # General
        self.controller = wpilib.XboxController(0)
        self.pdp = wpilib.PowerDistribution()


        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # Phoenix6 can log error message to an external usb drive

        # phoenix6.signal_logger.SignalLogger.set_path("path/to/usb")
        # phoenix6.signal_logger.SignalLogger.start()

    # def disabledPeriodic(self):
    #     """Mets à jours le dashboard, même quand le robot est désactivé"""
    #     self.update_nt()

    # def autonomousInit(self):
    #     """Cette fonction est appelée une seule fois lorsque le robot entre en mode autonome."""
    #     pass

    # def autonomous(self):
    #     """Pour les modes auto de MagicBot, voir le dossier ./autonomous"""
    #     super().autonomous()

    # def teleopInit(self):
    #     """Cette fonction est appelée une seule fois lorsque le robot entre en mode téléopéré."""
    #     pass

    # def teleopPeriodic(self):
    #     """Cette fonction est appelée de façon périodique lors du mode téléopéré."""
    #     self.update_nt()
    #     # Reset navx zero
    #     if self.gamepad1.getRawButton(gh.BUTTON_A):
    #         self.gyro.reset()

    #     self.drive.drive(
    #         0,
    #         0,
    #         0,
    #         False,
    #         0.020,
    #     )

    # def update_nt(self):
    #     """Affiche les données sur le ShuffleBoard"""
    #     pass

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(True)

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.1)
            )
            * kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.1)
            )
            * kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.1)
            )
            * kMaxSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, 0.020)


if __name__ == "__main__":
    wpilib.run(MyRobot)
