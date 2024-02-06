import math

import magicbot
import ntcore  # Outils pour les NetworkTables
import rev
import wpilib
from wpimath.geometry import Rotation2d

# Encodeur et moteur dans le meme sense?
kArmEncoderInverted = False

# Brake ou coast
kArmMotorIdleMode = rev.CANSparkMax.IdleMode.kBrake

# Position minimale
kArmSoftLimitReverse = math.pi / 2

# Position maximale
kArmSoftLimitForward = 3 * math.pi / 2

# Valeurs de PID
kArmP = 0.7
kArmI = 0
kArmD = 0
kArmFF = 0

# Duty cycle maximal utiliser par le PID
kArmMinOutput = -1
kArmMaxOutput = 1

# DO NOT TOUCH: Limit de courant (MAX 80, defaut=20)
kArmMotorCurrentLimit = 20

# DO NOT TOUCH: Facteur de position entre encodeur et pid controller
kArmEncoderPositionFactor = 2 * math.pi  # radians
kArmEncoderVelocityFactor = (2 * math.pi) / 60  # radians per second
kArmEncoderPositionPIDMinInput = 0  # radians
kArmEncoderPositionPIDMaxInput = kArmEncoderPositionFactor  # radians


class MyRobot(magicbot.MagicRobot):
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

    # Networktables pour de la configuration et retour d'information

    kArmP = magicbot.tunable(kArmP)
    kArmI = magicbot.tunable(kArmI)
    kArmD = magicbot.tunable(kArmD)
    kArmFF = magicbot.tunable(kArmFF)

    def createObjects(self):
        """
        C'est ici que les composants sont vraiment créé avec le signe =.
        Les composants avec un préfix connu tel que "intake_" vont être injectés.
        """
        # NetworkTable
        self.arm_motor_left = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)

        # Factory reset, on remet les spark dans un etat connu avant de les
        # configurer. C'est utile si on dois les remplacer
        # self.arm_motor_left.restoreFactoryDefaults()

        # Configurer les enconneurs et les controlleurs PIDs
        self.arm_position_encoder = self.arm_motor_left.getAbsoluteEncoder(
            rev.SparkAbsoluteEncoder.Type.kDutyCycle
        )
        self.arm_pid_controller = self.arm_motor_left.getPIDController()
        self.arm_pid_controller.setFeedbackDevice(self.arm_position_encoder)

        # Appliquer les facteur de conversion pour position et velocite pour le
        # bras. On veux ces valeurs en radians et radians par secondes pour etre compatible
        # avec les kinematic swerve de wpilib
        self.arm_position_encoder.setPositionConversionFactor(kArmEncoderPositionFactor)
        self.arm_position_encoder.setVelocityConversionFactor(kArmEncoderVelocityFactor)

        # Soft limit forward
        self.arm_motor_left.enableSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kForward, True
        )
        self.arm_motor_left.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kForward, kArmSoftLimitForward
        )

        # Soft limit backward
        self.arm_motor_left.enableSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kReverse, True
        )
        self.arm_motor_left.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kReverse, kArmSoftLimitReverse
        )

        # Inverser l'encodeur si necessaire
        self.arm_position_encoder.setInverted(kArmEncoderInverted)

        # Activer le wrap around pour le moteur, ceci permet au controlleur PID
        # de passer par 0 pour se rendre a son setpoint. Par example de passer
        # de 360 degrees a 10 degrees va passer par 0 plutot que l'autre
        # direction
        # self.arm_pid_controller.setPositionPIDWrappingEnabled(True)
        # self.arm_pid_controller.setPositionPIDWrappingMinInput(
        #     kArmEncoderPositionPIDMinInput
        # )
        # self.arm_pid_controller.setPositionPIDWrappingMaxInput(
        #     kArmEncoderPositionPIDMaxInput
        # )

        # Configurer le PID pour le moteur
        # self.arm_pid_controller.setP(kArmP)
        # self.arm_pid_controller.setI(kArmI)
        # self.arm_pid_controller.setD(kArmD)
        # self.arm_pid_controller.setFF(kArmFF)
        self.arm_pid_controller.setOutputRange(kArmMinOutput, kArmMaxOutput)

        # Configrer le idle mode et le courant maximal
        self.arm_motor_left.setIdleMode(kArmMotorIdleMode)
        self.arm_motor_left.setSmartCurrentLimit(kArmMotorCurrentLimit)

        # Sauvegarer la configuration. Si un SPARK MAX brown out durant operation
        # il va conserver ces configurations
        self.arm_motor_left.burnFlash()

        self.joystick = wpilib.PS5Controller(0)

    def disabledPeriodic(self):
        """Mets à jours le dashboard, même quand le robot est désactivé"""
        pass

    def autonomousInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode autonome."""
        pass

    def autonomous(self):
        """Pour les modes auto de MagicBot, voir le dossier ./autonomous"""
        pass

    def teleopInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode téléopéré."""
        pass

    def teleopPeriodic(self):
        """Cette fonction est appelée de façon périodique lors du mode téléopéré."""

        # Configurer le PID pour le moteur
        self.arm_pid_controller.setP(self.kArmP)
        self.arm_pid_controller.setI(self.kArmI)
        self.arm_pid_controller.setD(self.kArmD)
        self.arm_pid_controller.setFF(self.kArmFF)

        rotation = Rotation2d.fromDegrees(self.joystick.getLeftX() * 360)
        self.arm_pid_controller.setReference(
            rotation.radians(), rev.CANSparkMax.ControlType.kPosition
        )


if __name__ == "__main__":
    wpilib.run(MyRobot)
