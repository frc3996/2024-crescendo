#!/usr/bin/env python3

"""
Patron pour une base pilotable Swerve Drive

Un NAVX est nécessaire, car le positionnement est absolue (field centric).
Le zéro est fait au démarrage du robot, donc il est important de bien le positionner.
Presser le bouton A va également refaire son zéro.
"""


import phoenix5
import ntcore  # Outils pour les NetworkTables
import wpilib
from common import gamepad_helper as gh  # Outil pour faciliter l'utilisation des contrôleurs
from components import swervedrive, swervemodule  # Nos composantes logiciels. Permet de grouper les composantes par fonctionalitée
from magicbot import MagicRobot
from navx import AHRS  # Gyro NAVX


class MyRobot(MagicRobot):
    """
    Après avoir créer les 'components' de bas niveau, tel que 'drivetrain' ou 'intake', utiliser leur nom suivi d'un trait souligné (_)
    pour injecter des objets au composant.

    ex.:
    Après avoir créer dans 'components' un fichier intake_driver.py, utiliser une variable nommée "intake_beltMotor: phoenix5.WPI_TalonFX" déclare le type de la variable.
    Quand 'beltMotor' sera appelée depuis 'intake', ce sera un objet de type 'WPI_TalonFX'.

    Utiliser un signe égale (=) pendant la déclaration des variables tel que "intake_beltMotor = phoenix5.WPI_TalonFX(11)" crée l'objet.
    Quand 'beltMotor' est appelé depuis le composant 'intake' et ce sera un WPI_TalonFX avec un ID CAN de 11.

    Utilisez le signe = dans la fonction 'createObjects' pour vous assurer que les données sont biens transmises à leur composantes.

    Pour plus d'information: https://robotpy.readthedocs.io/en/stable/frameworks/magicbot.html
    """

    # Swerve Drive
    # Ici on crée un composant par swerve (swervemodule), suivi d'un dernier composant qui regroupe l'ensemble de toutes les swerves (swervedrive)
    # `inverted` doit être ajusté si le moteur tourne dans la mauvaise direction
    # `allow_reverse` est utiliser pour changer la direction de rotation d'un moteur plutôt que de faire une rotation de plus de 180 degrée
    # `nt_name` sera le nom sous lequel sera groupé les données des modules dans les networktables
    drive: swervedrive.SwerveDrive
    frontLeftModule: swervemodule.SwerveModule
    frontRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule
    navx: AHRS

    # Networktables pour de la configuration et retour d'information
    nt: ntcore.NetworkTable

    def createObjects(self):
        """
        C'est ici que les composants sont vraiment créé avec le signe =.
        Les composants avec un préfix connu tel que "intake_" vont être injectés.
        """
        # NetworkTable
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")

        # Configuration de la base swerve
        self.initSwerve()

        # General
        self.gamepad1 = wpilib.Joystick(0)
        self.pdp = wpilib.PowerDistribution()

    def initSwerve(self):
        """
        Configuration de la base Swerve Drive
        """
        # On assigne nos moteurs à nos swerve
        # Il est important d'utiliser le logiciel de la compagnie pour trouver (ou configurer) les CAN id
        # On utilise également les encodeurs absolues CAN pour orienter la roue
        self.drive_cfg = swervedrive.SwerveDriveConfig(
            base_width=10.5, base_length=19, enable_debug=False
        )

        self.frontLeftModule_driveMotor = phoenix5.WPI_TalonFX(26)
        self.frontLeftModule_rotateMotor = phoenix5.WPI_TalonFX(25)
        self.frontLeftModule_encoder = phoenix5._ctre.sensors.WPI_CANCoder(11)
        self.frontLeftModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="frontLeftModule", inverted=False, allow_reverse=True
        )

        self.frontRightModule_driveMotor = phoenix5.WPI_TalonFX(28)
        self.frontRightModule_rotateMotor = phoenix5.WPI_TalonFX(27)
        self.frontRightModule_encoder = phoenix5._ctre.sensors.WPI_CANCoder(14)
        self.frontRightModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="frontRightModule", inverted=True, allow_reverse=True
        )

        self.rearLeftModule_driveMotor = phoenix5.WPI_TalonFX(24)
        self.rearLeftModule_rotateMotor = phoenix5.WPI_TalonFX(23)
        self.rearLeftModule_encoder = phoenix5._ctre.sensors.WPI_CANCoder(12)
        self.rearLeftModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="rearLeftModule", inverted=True, allow_reverse=True
        )

        self.rearRightModule_driveMotor = phoenix5.WPI_TalonFX(21)
        self.rearRightModule_rotateMotor = phoenix5.WPI_TalonFX(22)
        self.rearRightModule_encoder = phoenix5._ctre.sensors.WPI_CANCoder(13)
        self.rearRightModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="rearRightModule", inverted=False, allow_reverse=True
        )

        # Le ShuffleBoard est utilisé afin d'ajuster le zéro des roues.
        # Un fois testé, les valeurs peuvent-être modifiées ici.
        self.nt.putNumber("frontLeftModule/rotation_zero", 166)
        self.nt.putNumber("frontRightModule/rotation_zero", 109)
        self.nt.putNumber("rearLeftModule/rotation_zero", 330)
        self.nt.putNumber("rearRightModule/rotation_zero", 44)

        # Et le navx nécessaire pour un control "Field Centric"
        self.navx = AHRS.create_spi(update_rate_hz=50)

    def disabledPeriodic(self):
        """Mets à jours le dashboard, même quand le robot est désactivé"""
        self.update_nt()

    def autonomousInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode autonome."""
        self.drive.init()

    def autonomous(self):
        """Pour les modes auto de MagicBot, voir le dossier ./autonomous"""
        super().autonomous()

    def teleopInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode téléopéré."""
        self.drive.init()

    def teleopPeriodic(self):
        """Cette fonction est appelée de façon périodique lors du mode téléopéré."""
        self.update_nt()
        # Reset navx zero
        if self.gamepad1.getRawButton(gh.BUTTON_A):
            self.drive.navx_zero_angle()

        self.drive.controller_move(
            self.gamepad1.getRawAxis(gh.AXIS_LEFT_Y),
            self.gamepad1.getRawAxis(gh.AXIS_LEFT_X),
            self.gamepad1.getRawAxis(gh.AXIS_RIGHT_X),
            self.gamepad1.getRawAxis(gh.AXIS_RIGHT_Y),
        )

    def update_nt(self):
        """Affiche les données sur le ShuffleBoard"""
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
