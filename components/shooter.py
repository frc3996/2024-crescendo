import magicbot
import rev
from magicbot import StateMachine, default_state, state, timed_state

import constants


class Shooter:
    # Valeurs de PID
    kP = magicbot.tunable(0.0003)
    kI = magicbot.tunable(0.0)
    kD = magicbot.tunable(0.0)
    kFF = magicbot.tunable(0.0)

    # MAX SPEED IS 5676
    velocity = magicbot.tunable(5000)

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
    def enable(self):
        self.pid.setReference(self.velocity, rev.CANSparkMax.ControlType.kVelocity)

    def disable(self):
        self.pid.setReference(0, rev.CANSparkMax.ControlType.kDutyCycle)

    def is_enabled(self):
        if self.encoder.getVelocity() > 0:
            return True
        return False

    def is_ready(self):
        return abs(self.encoder.getVelocity() - self.velocity) < 100

    def execute(self):
        return
        # Update the tunables
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)


class ShooterFollower:
    kFollowerInverted = True
    shooter: Shooter

    def setup(self):
        self.motor = rev.CANSparkMax(
            constants.CANIds.SHOOTER_RIGHT, rev.CANSparkMax.MotorType.kBrushless
        )
        # Factory reset, on remet les spark dans un etat connu avant de les
        # configurer. C'est utile si on dois les remplacer
        self.motor.restoreFactoryDefaults()

        # En mode CAN, un SPARK MAX est configurer pour suivre un autre
        self.motor.follow(self.shooter.motor, invert=self.kFollowerInverted)

        self.motor.burnFlash()

    def execute(self):
        pass
