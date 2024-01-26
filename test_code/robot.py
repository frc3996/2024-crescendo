
import rev
import ntcore  # Outils pour les NetworkTables
import wpilib
from magicbot import MagicRobot
from wpimath.controller import PIDController
import gamepad_helper

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

    # Networktables pour de la configuration et retour d'information

    def createObjects(self):
        """
        C'est ici que les composants sont vraiment créé avec le signe =.
        Les composants avec un préfix connu tel que "intake_" vont être injectés.
        """
        # NetworkTable
        self.arm_motor_left = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)
        self.arm_motor_left.restoreFactoryDefaults()
        self.arm_motor_left.setOpenLoopRampRate(2)
        self.arm_motor_left.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        # self.arm_pid_controller = self.arm_motor_left.getPIDController()
        self.arm_position_encoder = self.arm_motor_left.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        # self.arm_position_encoder = self.arm_motor_left.getEncoder()
        # self.arm_position_encoder.setPosition(0)
        # self.arm_pid_controller.setFeedbackDevice(self.arm_position_encoder)
        self.arm_pid = PIDController(0.1, 0.02, 0.001)
        self.arm_pid.enableContinuousInput(0, 1)
        self.arm_pid



        # kP = 0.000002
        # kI = 0  # 0.0001
        # kD = 0
        # kIz = 0
        # kFF = 0.000
        # kMaxOutput = 1
        # kMinOutput = -1
        # self.arm_pid_controller.setPositionPIDWrappingEnabled(True)
        # self.arm_pid_controller.setPositionPIDWrappingMaxInput(100)
        # self.arm_pid_controller.setPositionPIDWrappingMinInput(-100)
        # self.arm_pid_controller.setP(kP)
        # self.arm_pid_controller.setI(kI)
        # self.arm_pid_controller.setD(kD)
        # self.arm_pid_controller.setIZone(kIz)
        # self.arm_pid_controller.setFF(kFF)
        # self.arm_pid_controller.setOutputRange(kMinOutput, kMaxOutput)

        self.gamepad1 = wpilib.Joystick(0)

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

        # error = ((self.gamepad1.getRawAxis(gamepad_helper.AXIS_LEFT_Y)+1) / 2) - self.arm_position_encoder.getPosition()
        # error *= 1
        print(self.arm_position_encoder.getPosition()*360)
        error = self.arm_pid.calculate(self.arm_position_encoder.getPosition(), ((self.gamepad1.getRawAxis(gamepad_helper.AXIS_LEFT_Y)+1) / 2))
        # print(round(error,3))
        self.arm_motor_left.set(error)


if __name__ == "__main__":
    wpilib.run(MyRobot)
