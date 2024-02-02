import magicbot
import rev
from magicbot import StateMachine, default_state, state, timed_state, will_reset_to, feedback
from wpimath import controller
from common import tools

import constants


class Shooter:
    # Valeurs de PID
    kP = magicbot.tunable(0.005)
    kI = magicbot.tunable(0.0)
    kD = magicbot.tunable(0.0)
    kFF = magicbot.tunable(0.0)

    # MAX SPEED IS 5676
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
        self.motor.setControlFramePeriodMs(0)  # Control frame from the rio?
        self.motor.setIdleMode(self.motor.IdleMode.kCoast)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus0, 20)  # Faults and output (default 10ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus3, 500)   # Analog sensor (default 50ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus4, 60000)   # Alternate encoder (default 20ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus5, 60000)   # Absolute encoder Pos/Angle (default 200ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus6, 60000)   # Absolute encoder Vel/Freq (default 200ms)
        self.motor.setInverted(True)
        self.encoder = self.motor.getEncoder()
        # self.pid = self.motor.getPIDController()
        # self.pid.setFeedbackDevice(self.encoder)
        self.software_pid = controller.PIDController(self.kP, self.kI, self.kD)
        self.__target_velocity = 0

        # On rampe la vitesse
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)

        # Configurer le PID pour le moteur
        # self.pid.setP(self.kP)
        # self.pid.setI(self.kI)
        # self.pid.setD(self.kD)
        # self.pid.setFF(self.kFF)
        # self.pid.setOutputRange(self.kMinOutput, self.kMaxOutput)

        self.motor.burnFlash()

    # Control methods
    def enable(self):
        self.__target_velocity = self.velocity
        # self.pid.setReference(self.velocity, rev.CANSparkMax.ControlType.kDutyCycle)

    def disable(self):
        self.__target_velocity = 0
        # self.pid.setReference(0, rev.CANSparkMax.ControlType.kDutyCycle)

    def is_enabled(self):
        if self.getVelocity() > 0:
            return True
        return False

    @feedback
    def getVelocity(self):
        return self.encoder.getVelocity()

    def is_ready(self):
        return abs(self.getVelocity()*2 - self.velocity) < 400

    def execute(self):
        error = self.software_pid.calculate(self.getVelocity(), self.__target_velocity)
        self.motor.set(error)
        return

    def on_enable(self):
        # Update the tunables
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)
        self.software_pid.setPID(self.kP, self.kI, self.kD)
        # self.pid.setP(self.kP)
        # self.pid.setI(self.kI)
        # self.pid.setD(self.kD)
        # self.pid.setFF(self.kFF)


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
        self.motor.setControlFramePeriodMs(0)  # Control frame from the rio?
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus0, 20)  # Faults and output (default 10ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus3, 500)   # Analog sensor (default 50ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus4, 60000)   # Alternate encoder (default 20ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus5, 60000)   # Absolute encoder Pos/Angle (default 200ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus6, 60000)   # Absolute encoder Vel/Freq (default 200ms)

        # En mode CAN, un SPARK MAX est configurer pour suivre un autre
        self.motor.follow(self.shooter.motor, invert=self.kFollowerInverted)

        self.motor.burnFlash()

    def execute(self):
        pass
