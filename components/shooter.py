import magicbot
import rev
from magicbot import StateMachine, default_state, state, timed_state, will_reset_to, feedback
from wpimath import controller
from common import tools

import constants


class ShooterMain:
    # Valeurs de PID
    CANID = constants.CANIds.SHOOTER_LEFT
    kP = magicbot.tunable(0.00037)
    kI = magicbot.tunable(0.0000007)
    kD = magicbot.tunable(0.0)
    kFF = magicbot.tunable(0.0)
    kMotorClosedLoopRampRate = magicbot.tunable(0.0)
    kInverted = True
    delta = magicbot.tunable(100)

    # Duty cycle maximal utiliser par le PID
    kMinOutput = -1
    kMaxOutput = 1

    def setup(self):
        self.motor = rev.CANSparkMax(
            self.CANID, rev.CANSparkMax.MotorType.kBrushless
        )
        # self.motor.restoreFactoryDefaults()
        self.motor.setControlFramePeriodMs(0)  # Control frame from the rio?
        self.motor.setIdleMode(self.motor.IdleMode.kCoast)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus0, 20)  # Faults and output (default 10ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus3, 500)   # Analog sensor (default 50ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus4, 60000)   # Alternate encoder (default 20ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus5, 60000)   # Absolute encoder Pos/Angle (default 200ms)
        self.motor.setPeriodicFramePeriod(self.motor.PeriodicFrame.kStatus6, 60000)   # Absolute encoder Vel/Freq (default 200ms)
        self.motor.setInverted(self.kInverted)
        self.encoder = self.motor.getEncoder()
        self.pid = self.motor.getPIDController()
        self.pid.setFeedbackDevice(self.encoder)
        # self.software_pid = controller.PIDController(self.kP, self.kI, self.kD)
        self.__target_velocity = 0

        # On rampe la vitesse
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)

        # Configurer le PID pour le moteur
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)
        self.pid.setOutputRange(self.kMinOutput, self.kMaxOutput)

        self.motor.burnFlash()

    def set_velocity(self, velocity):
        self.__target_velocity = velocity
        self.pid.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)

    def disable(self):
        self.__target_velocity = 0
        self.motor.set(0)
        # self.pid.setReference(0, rev.CANSparkMax.ControlType.kVelocity)

    def is_enabled(self):
        if self.getVelocity() > 0:
            return True
        return False

    # @feedback
    def getVelocity(self):
        return self.encoder.getVelocity()

    def is_ready(self):
        return abs(self.getVelocity() - self.__target_velocity) < self.delta

    def on_enable(self):
        # Update the tunables
        self.motor.setClosedLoopRampRate(self.kMotorClosedLoopRampRate)
        # self.software_pid.setPID(self.kP, self.kI, self.kD)
        self.pid.setP(self.kP)
        self.pid.setI(self.kI)
        self.pid.setD(self.kD)
        self.pid.setFF(self.kFF)

    def execute(self):
        # error = self.software_pid.calculate(self.getVelocity(), self.__target_velocity)
        # self.motor.set(error)
        return


class ShooterFollower(ShooterMain):
    # Valeurs de PID
    CANID = constants.CANIds.SHOOTER_RIGHT
    kP = magicbot.tunable(0.00030)
    kI = magicbot.tunable(0.0000008)
    kD = magicbot.tunable(0.0)
    kFF = magicbot.tunable(0.0)
    kMotorClosedLoopRampRate = magicbot.tunable(0.0)
    kInverted = False

    # Duty cycle maximal utiliser par le PID
    kMinOutput = -1
    kMaxOutput = 1


class Shooter:
    shooter_main: ShooterMain
    shooter_follower: ShooterFollower

    # MAX SPEED IS 5676
    main_speaker_velocity = magicbot.tunable(4500)
    follower_speaker_velocity = magicbot.tunable(4500)

    amp_velocity = magicbot.tunable(4000)


    # Control methods
    def shoot_amp(self):
        self.shooter_main.set_velocity(self.amp_velocity)
        self.shooter_follower.set_velocity(self.amp_velocity)

    # Control methods
    def shoot_speaker(self):
        self.shooter_main.set_velocity(self.main_speaker_velocity)
        self.shooter_follower.set_velocity(self.follower_speaker_velocity)

    # Control methods
    def disable(self):
        self.shooter_main.disable()
        self.shooter_follower.disable()

    def is_ready(self):
        return self.shooter_main.is_ready() and self.shooter_follower.is_ready()

    def execute(self):
        pass
