import magicbot
import rev
import wpilib

import constants

INTAKE_SPEED = 10


class IntakeBeam:
    def setup(self):
        self.beam = wpilib.AnalogInput(2)

    # Informational methods
    def object_in_sensor(self):
        return self.beam.getValue() < 324

    def execute(self):
        pass


class IntakeGrabber:
    def setup(self):
        self.input_motor = rev.CANSparkMax(
            constants.CANIds.INTAKE_GRABBER, rev.CANSparkMax.MotorType.kBrushless
        )

    def execute(self):
        pass


class IntakeShooter:
    # Valeurs de PID
    kP = magicbot.tunable(0.0003)
    kI = magicbot.tunable(0.0)
    kD = magicbot.tunable(0.0)
    kFF = magicbot.tunable(0.0)

    velocity = magicbot.tunable(8000)

    ## On rampe la vitesse
    kMotorClosedLoopRampRate = magicbot.tunable(0.3)

    # Duty cycle maximal utiliser par le PID
    kMinOutput = -1
    kMaxOutput = 1

    def setup(self):
        self.motor = rev.CANSparkMax(
            constants.CANIds.SHOOTER_LEFT, rev.CANSparkMax.MotorType.kBrushless
        )

        self.motor.restoreFactoryDefaults()
        self.motor.setInverted(True)
        self.encoder = self.motor.getEncoder()
        self.pid = self.motor.getPIDController()
        self.pid.setFeedbackDevice(self.encoder)

        # On rampe la vitesse
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)

        # Configurer le PID pour le moteur
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)
        self.pid.setOutputRange(self.kMinOutput, self.kMaxOutput)

        self.motor.burnFlash()

    # Control methods
    def enable_shooter(self):
        self.pid.setReference(self.velocity, rev.CANSparkMax.ControlType.kVelocity)

    def disable_shooter(self):
        self.pid.setReference(0, rev.CANSparkMax.ControlType.kVelocity)

    def is_ready(self):
        return abs(self.encoder.getVelocity() - self.velocity) < 1

    def execute(self):
        # Update the tunables
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)


class IntakeShooterFollower:
    kFollowerInverted = True
    intake_shooter: IntakeShooter

    def setup(self):
        self.motor = rev.CANSparkMax(
            constants.CANIds.SHOOTER_RIGHT, rev.CANSparkMax.MotorType.kBrushless
        )
        # Factory reset, on remet les spark dans un etat connu avant de les
        # configurer. C'est utile si on dois les remplacer
        self.motor.restoreFactoryDefaults()

        # En mode CAN, un SPARK MAX est configurer pour suivre un autre
        self.motor.follow(self.intake_shooter.motor, invert=self.kFollowerInverted)

        self.motor.burnFlash()

    def execute(self):
        pass


class Intake:
    shooter: IntakeShooter
    grabber: IntakeGrabber
    beam: IntakeBeam

    def setup(self):
        self.shot_done = False
        self.input_motor_setpoint = 0
        self.output_motor_setpoint = 0
        self.beam_sensor_last_detection = wpilib.Timer()
        self.beam_sensor_last_detection.start()

    # Control methods
    def do_intake(self):
        self.shot_done = False
        if self.beam.object_in_sensor() is True:
            return True

        self.input_motor_setpoint = INTAKE_SPEED
        return False

    def do_shoot(self, fire=False):
        if self.shot_done:
            return True

        if not self.beam.object_in_sensor():
            self.shot_done = True
            return True

        self.output_motor_setpoint = SHOOTER_SPEED

        if not self.shooter.is_ready():
            return

        if fire is True:
            self.input_motor_setpoint = INTAKE_SPEED

    def execute(self):
        if self.beam.object_in_sensor():
            self.beam_sensor_last_detection.reset()

        self.input_motor_setpoint = 0
        self.output_motor_setpoint = 0
