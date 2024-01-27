import math

import magicbot
import rev
import wpilib
import wpimath.units
from magicbot import feedback
from wpimath.geometry import Rotation2d
import constants

class LoBrasHead:
    # Enconneur

    ## Encodeur et moteur dans le meme sense?
    kEncoderInverted = True

    ## Position minimale
    kSoftLimitReverse = math.radians(46)

    ## Position maximale
    kSoftLimitForward = math.radians(285)

    # Duty cycle maximal utiliser par le PID
    kMinOutput = -1
    kMaxOutput = 1

    ## DO NOT TOUCH: Facteur de position entre encodeur et pid controller
    kEncoderPositionFactor = 2 * math.pi  # radians
    kEncoderVelocityFactor = (2 * math.pi) / 60  # radians per second
    kEncoderPositionPIDMinInput = 0  # radians
    kEncoderPositionPIDMaxInput = kEncoderPositionFactor  # radians

    # Moteur

    ## Brake ou coast
    kMotorIdleMode = rev.CANSparkMax.IdleMode.kBrake

    # Limit de courant (MAX 80, defaut=20)
    kMotorCurrentLimit = 20

    # Valeurs de PID
    kP = magicbot.tunable(0.2)
    kI = magicbot.tunable(0.0)
    kD = magicbot.tunable(0.0)
    kFF = magicbot.tunable(0.0)

    ## On ne se teleporte pas a une position
    kMotorClosedLoopRampRate = magicbot.tunable(0.2)

    def setup(self):
        self.motor = rev.CANSparkMax(constants.CANIds.HEAD_ANGLE_MOTOR, rev.CANSparkMax.MotorType.kBrushless)

        # Factory reset, on remet les spark dans un etat connu avant de les
        # configurer. C'est utile si on dois les remplacer
        self.motor.restoreFactoryDefaults()

        self.motor.setInverted(True)
        # Configurer les enconneurs et les controlleurs PIDs
        self.encoder = self.motor.getAbsoluteEncoder(
            rev.SparkAbsoluteEncoder.Type.kDutyCycle
        )
        self.pid = self.motor.getPIDController()
        self.pid.setFeedbackDevice(self.encoder)

        # Appliquer les facteur de conversion pour position et velocite pour le
        # bras. On veux ces valeurs en radians et radians par secondes pour etre compatible
        # avec les kinematic swerve de wpilib
        self.encoder.setPositionConversionFactor(self.kEncoderPositionFactor)
        self.encoder.setVelocityConversionFactor(self.kEncoderVelocityFactor)

        # Soft limit forward
        self.motor.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, True)
        self.motor.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kForward, self.kSoftLimitForward
        )

        # Soft limit backward
        self.motor.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, True)
        self.motor.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kReverse, self.kSoftLimitReverse
        )

        # Inverser l'encodeur si necessaire
        self.encoder.setZeroOffset(0)
        self.encoder.setInverted(self.kEncoderInverted)

        # XXX: Desactiver puisque le bras ne dois jamais prendre le chemin le
        #      plus court
        # Activer le wrap around pour le moteur, ceci permet au controlleur PID
        # de passer par 0 pour se rendre a son setpoint. Par example de passer
        # de 360 degrees a 10 degrees va passer par 0 plutot que l'autre
        # direction
        # self.pid.setPositionPIDWrappingEnabled(True)
        # self.pid.setPositionPIDWrappingMinInput(
        #     kLeftHeadEncoderPositionPIDMinInput
        # )
        # self.pid.setPositionPIDWrappingMaxInput(
        #     kLeftHeadEncoderPositionPIDMaxInput
        # )

        # On rampe la vitesse
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)

        # Configurer le PID pour le moteur
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)
        self.pid.setOutputRange(self.kMinOutput, self.kMaxOutput)

        # Configrer le idle mode et le courant maximal
        self.motor.setIdleMode(self.kMotorIdleMode)
        self.motor.setSmartCurrentLimit(self.kMotorCurrentLimit)

        # Sauvegarer la configuration. Si un SPARK MAX brown-out durant operation
        # il va conserver ces configurations
        self.motor.burnFlash()

    def setAngle(self, angle: float):
        # Rotation 2D returns from -pi to +pi. Normalize from 0 to 2pi.
        angle = math.radians(angle)
        angle += self.kSoftLimitReverse
        self.pid.setReference(angle, rev.CANSparkMax.ControlType.kPosition)

    def getPosition(self) -> float:
        return self.encoder.getPosition()

    def execute(self):
        # return
        # Update the tunables
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)


class LoBrasArm:
    # Enconneur

    ## Encodeur et moteur dans le meme sense?
    kEncoderInverted = False

    ## Position minimale
    kSoftLimitReverse = math.radians(105)

    ## Position maximale
    kSoftLimitForward = math.radians(210)

    # Duty cycle maximal utiliser par le PID
    kMinOutput = -1
    kMaxOutput = 1

    ## DO NOT TOUCH: Facteur de position entre encodeur et pid controller
    kEncoderPositionFactor = 2 * math.pi  # radians
    kEncoderVelocityFactor = (2 * math.pi) / 60  # radians per second
    kEncoderPositionPIDMinInput = 0  # radians
    kEncoderPositionPIDMaxInput = kEncoderPositionFactor  # radians

    # Moteur

    ## Brake ou coast
    kMotorIdleMode = rev.CANSparkMax.IdleMode.kBrake

    # Limit de courant (MAX 80, defaut=20)
    kMotorCurrentLimit = 20

    # Valeurs de PID
    kP = magicbot.tunable(1)
    kI = magicbot.tunable(0.0)
    kD = magicbot.tunable(0.0)
    kFF = magicbot.tunable(0.0)

    ## On rampe la vitesse
    kMotorClosedLoopRampRate = magicbot.tunable(0.2)

    def setup(self):
        # self.arm_limit_switch = wpilib.DigitalInput(1)

        self.motor = rev.CANSparkMax(constants.CANIds.ARM_RIGHT, rev.CANSparkMax.MotorType.kBrushless)

        # Factory reset, on remet les spark dans un etat connu avant de les
        # configurer. C'est utile si on dois les remplacer
        self.motor.restoreFactoryDefaults()

        self.motor.setInverted(True)
        # Configurer les enconneurs et les controlleurs PIDs
        self.encoder = self.motor.getAbsoluteEncoder(
            rev.SparkAbsoluteEncoder.Type.kDutyCycle
        )
        self.pid = self.motor.getPIDController()
        self.pid.setFeedbackDevice(self.encoder)

        # Appliquer les facteur de conversion pour position et velocite pour le
        # bras. On veux ces valeurs en radians et radians par secondes pour etre compatible
        # avec les kinematic swerve de wpilib
        self.encoder.setPositionConversionFactor(self.kEncoderPositionFactor)
        self.encoder.setVelocityConversionFactor(self.kEncoderVelocityFactor)

        # Soft limit forward
        self.motor.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, True)
        self.motor.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kForward, self.kSoftLimitForward
        )

        # Soft limit backward
        self.motor.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, True)
        self.motor.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kReverse, self.kSoftLimitReverse
        )

        # Inverser l'encodeur si necessaire
        self.encoder.setInverted(self.kEncoderInverted)
        self.encoder.setZeroOffset(0)

        # XXX: Desactiver puisque le bras ne dois jamais prendre le chemin le
        #      plus court
        # Activer le wrap around pour le moteur, ceci permet au controlleur PID
        # de passer par 0 pour se rendre a son setpoint. Par example de passer
        # de 360 degrees a 10 degrees va passer par 0 plutot que l'autre
        # direction
        # self.pid.setPositionPIDWrappingEnabled(True)
        # self.pid.setPositionPIDWrappingMinInput(
        #     kEncoderPositionPIDMinInput
        # )
        # self.pid.setPositionPIDWrappingMaxInput(
        #     kEncoderPositionPIDMaxInput
        # )

        # On rampe la vitesse
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)

        # Configurer le PID pour le moteur
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)
        self.pid.setOutputRange(self.kMinOutput, self.kMaxOutput)

        # Configrer le idle mode et le courant maximal
        self.motor.setIdleMode(self.kMotorIdleMode)
        self.motor.setSmartCurrentLimit(self.kMotorCurrentLimit)

        # Sauvegarer la configuration. Si un SPARK MAX brown-out durant operation
        # il va conserver ces configurations
        self.motor.burnFlash()

    def setAngle(self, angle: float):
        # Rotation 2D returns from -pi to +pi. Normalize from 0 to 2pi.
        angle = math.radians(angle)
        angle += self.kSoftLimitReverse
        self.pid.setReference(angle, rev.CANSparkMax.ControlType.kPosition)

    def getPosition(self) -> float:
        return self.encoder.getPosition()

    def execute(self):
        return
        # Update the tunables
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)


class LoBrasArmFollower:
    kFollowerInverted = True
    lobras_arm: LoBrasArm

    def setup(self):
        self.motor = rev.CANSparkMax(constants.CANIds.ARM_LEFT, rev.CANSparkMax.MotorType.kBrushless)
        # Factory reset, on remet les spark dans un etat connu avant de les
        # configurer. C'est utile si on dois les remplacer
        self.motor.restoreFactoryDefaults()

        # En mode CAN, un SPARK MAX est configurer pour suivre un autre
        self.motor.follow(self.lobras_arm.motor, invert=self.kFollowerInverted)

        self.motor.burnFlash()

    def execute(self):
        pass


class LoBras:
    lobras_arm: LoBrasArm
    lobras_arm_follower: LoBrasArmFollower
    lobras_head: LoBrasHead

    def set_arm_angle(self, arm_position: float):
        """Set the target angles, in degrees, from 0 to 360"""
        self.lobras_arm.setAngle(arm_position)

    def set_head_angle(self, head_position: float):
        """Set the target angles, in degrees, from 0 to 360"""
        self.lobras_head.setAngle(head_position)

    def set_angle(self, arm_position: float, head_position: float):
        """Set the target angles, in degrees, from 0 to 360"""
        self.lobras_arm.setAngle(arm_position)
        self.lobras_head.setAngle(head_position)

    @feedback
    def current_arm_position(self):
        return math.degrees(self.lobras_arm.getPosition())

    @feedback
    def current_head_position(self):
        return math.degrees(self.lobras_head.getPosition())

    def intake_mode(self):
        # TODO
        # Place la tête et le bras en position intake
        # self.__set_positions(xx, yy)
        pass

    def speaker_mode(self, fire=False):
        # TODO
        # Place le bras en mode speaker
        # Utilise la limelight pour viser avec la tete

        if fire is False:
            return

        # Tire si limelight + vitesse Ok

    def amp_mode(self):
        # TODO
        # Place la tête et le bras en position amp
        pass

    def note_trap_mode(self):
        # TODO
        # Place la tête et le bras en position note_trap
        pass

    def execute(self):
        pass
