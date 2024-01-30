import magicbot
import rev
from magicbot import StateMachine, default_state, state, timed_state

import constants
from components import Intake, IntakeControl

from .lobra import LoBras, LoBrasArm, LoBrasHead


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
        return abs(self.encoder.getVelocity() - self.velocity) < 1

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


class ShooterControl(StateMachine):
    lobras_arm: LoBrasArm
    lobras_head: LoBrasHead
    shooter: Shooter
    intake_control: IntakeControl

    def fire(self):
        """This function is called from teleop or autonomous to cause the
        shooter to fire"""
        self.engage()

    @state(first=True)
    def position_arm(self):
        self.lobras_arm.set_angle(100)
        if self.lobras_arm.is_ready():
            self.next_state_now("position_head")

    @state
    def position_head(self):
        """Premier etat, position la tete"""
        self.lobras_head.set_angle(195)

        if self.lobras_head.is_ready():
            self.next_state_now("prepare_to_fire")

    @state
    def prepare_to_fire(self):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        self.shooter.enable()

        # if self.shooter.is_ready():
        self.next_state_now("firing")

    @state
    def firing(self):
        """Fires the ball"""
        self.intake_control.feed()
        if not self.intake_control.is_executing:
            self.next_state_now("stop")

    @state
    def stop(self):
        """Always called to stop the motor"""
        self.shooter.disable()

    def done(self) -> None:
        self.shooter.disable()
        return super().done()
