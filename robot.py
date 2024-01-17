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


import phoenix6
import rev
import ntcore  # Outils pour les NetworkTables
import wpilib
from common import gamepad_helper as gh  # Outil pour faciliter l'utilisation des contrôleurs
from common import limelight, arduino_light
from magicbot import MagicRobot
from navx import AHRS


from components import swervemodule, swervedrive, lobra, intake, robot_actions


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
    lobras: lobra.LoBras
    intake: intake.Intake
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
        self.initIntake()
        self.initLobras()

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
        self.drivetrain_cfg = swervedrive.SwerveDriveConfig(
            field_centric=True, base_width=27, base_length=19, enable_debug=False
        )

        self.frontLeftModule_driveMotor = phoenix6.hardware.TalonFX(26)
        self.frontLeftModule_rotateMotor = phoenix6.hardware.TalonFX(25)
        self.frontLeftModule_encoder = phoenix6.hardware.CANcoder(11)
        self.frontLeftModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="frontLeftModule", inverted=False, allow_reverse=True
        )

        self.frontRightModule_driveMotor = phoenix6.hardware.TalonFX(28)
        self.frontRightModule_rotateMotor = phoenix6.hardware.TalonFX(27)
        self.frontRightModule_encoder = phoenix6.hardware.CANcoder(14)
        self.frontRightModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="frontRightModule", inverted=True, allow_reverse=True
        )

        self.rearLeftModule_driveMotor = phoenix6.hardware.TalonFX(24)
        self.rearLeftModule_rotateMotor = phoenix6.hardware.TalonFX(23)
        self.rearLeftModule_encoder = phoenix6.hardware.CANcoder(12)
        self.rearLeftModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="rearLeftModule", inverted=True, allow_reverse=True
        )

        self.rearRightModule_driveMotor = phoenix6.hardware.TalonFX(21)
        self.rearRightModule_rotateMotor = phoenix6.hardware.TalonFX(22)
        self.rearRightModule_encoder = phoenix6.hardware.CANcoder(13)
        self.rearRightModule_cfg = swervemodule.SwerveModuleConfig(
            nt_name="rearRightModule", inverted=False, allow_reverse=True
        )

        # Le ShuffleBoard est utilisé afin d'ajuster le zéro des roues.
        # Un fois testé, les valeurs peuvent-être modifiées ici.
        self.nt.putNumber("config/zero_calibration_mode", 0)

        self.nt.putNumber("frontLeftModule/rotation_zero", 193)
        self.nt.putNumber("frontRightModule/rotation_zero", 76)
        self.nt.putNumber("rearLeftModule/rotation_zero", 216)
        self.nt.putNumber("rearRightModule/rotation_zero", 318)

        # Et le navx nécessaire pour un control "Field Centric"
        self.navx = AHRS.create_spi(update_rate_hz=50)

    def initIntake(self):
        self.intake_input_motor = rev.CANSparkMax(101, rev.CANSparkMax.MotorType.kBrushless)
        self.intake_output_motor = rev.CANSparkMax(102, rev.CANSparkMax.MotorType.kBrushless)
        self.intake_beam_sensor = wpilib.AnalogInput(2)

    def initLobras(self):
        self.lobras_head_motor = rev.CANSparkMax(103, rev.CANSparkMax.MotorType.kBrushless)
        self.lobras_arm_motor = rev.CANSparkMax(104, rev.CANSparkMax.MotorType.kBrushless)
        self.lobras_arm_limit_switch = wpilib.DigitalInput(1)
        self.lobras_pneumatic_brake = wpilib.Solenoid(10, wpilib.PneumaticsModuleType.CTREPCM, 0)

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
        self.arduino_light.set_RGB(0,0,0)
        self.status_light.set(0)

    def teleopPeriodic(self):
        """Cette fonction est appelée de façon périodique lors du mode téléopéré."""
        self.update_nt()

        self.drivetrain.set_controller_values(
            self.gamepad1.getRawAxis(gh.AXIS_LEFT_Y),
            self.gamepad1.getRawAxis(gh.AXIS_LEFT_X),
            self.gamepad1.getRawAxis(gh.AXIS_RIGHT_X),
            self.gamepad1.getRawAxis(gh.AXIS_RIGHT_Y),
        )

        # Reset navx zero
        if self.gamepad1.getRawButton(gh.BUTTON_X):
            self.drivetrain.navx_zero_angle()

        if self.gamepad1.getRawButton(gh.BUTTON_LEFT_BUMPER):
            self.drivetrain.request_wheel_lock = True

        if self.gamepad1.getRawButton(gh.BUTTON_A):
            self.robot_actions.autointake_with_limelight()
        elif self.gamepad1.getRawButton(gh.BUTTON_B):
            self.robot_actions.autoshoot_amp()
        elif self.gamepad1.getRawButton(gh.BUTTON_Y):
            self.robot_actions.auto_test()
        else:
            self.robot_actions.retract()


    def update_nt(self):
        """Affiche les données sur le ShuffleBoard"""
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
