#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import phoenix6

kWheelRadius = 0.0508
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class RikiCANcoder(phoenix6.hardware.CANcoder):
    dpr: float
    def __init__(self, device_id: int, canbus: str = ""):
        super().__init__(device_id, canbus)
        self.previous_position = 0

    def setDistancePerRotation(self, dpr: float):
        self.dpr = dpr 

    def getRate(self):
        return self.dpr * self.get_velocity().value

    def getDistance(self):
        current_position = self.get_position().value
        distance = (current_position - self.previous_position) * self.dpr
        self.previous_position = current_position
        return distance
        


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        driveEncoderChannel: int,
        turningEncoderChannel: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel
        :param turningMotorChannel
        :param driveEncoderChannel
        :param turningEncoderChannel
        """
        self.driveMotor = phoenix6.hardware.TalonFX(driveMotorChannel)
        self.turningMotor = phoenix6.hardware.TalonFX(turningMotorChannel)
        self.driveEncoder = RikiCANcoder(driveEncoderChannel)
        self.turningEncoder = RikiCANcoder(turningEncoderChannel)

        self.init_driveMotor(self.driveMotor)
        self.init_turningMotor(self.turningMotor)

        self.init_cancoder(self.driveEncoder)
        self.init_cancoder(self.turningEncoder)


        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(1, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            1,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        # Set the distance per pulse for the drive encoder. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.
        self.driveEncoder.setDistancePerRotation(
            math.tau * kWheelRadius
        )

        # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        # This is the the angle through an entire rotation (2 * pi) divided by the
        # encoder resolution.
        self.turningEncoder.setDistancePerRotation(math.tau)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)


    def init_cancoder(self, cancoder: phoenix6.hardware.CANcoder):
        cancoder_config = phoenix6.configs.CANcoderConfiguration()
        cancoder.configurator.apply(cancoder_config)


    def init_driveMotor(self, motor: phoenix6.hardware.TalonFX):
        config = phoenix6.configs.TalonFXConfiguration()

        config.open_loop_ramps = phoenix6.configs.OpenLoopRampsConfigs()
        config.open_loop_ramps.with_duty_cycle_open_loop_ramp_period(0.1)

        motor_config = phoenix6.configs.MotorOutputConfigs()
        motor_config.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        config.motor_output = motor_config

        # self.driveMotor_control = phoenix6.controls.DutyCycleOut(0)
        # self.rotateMotor_control = phoenix6.controls.DutyCycleOut(0)

        motor.configurator.apply(config)

    def init_turningMotor(self, motor: phoenix6.hardware.TalonFX):
        config = phoenix6.configs.TalonFXConfiguration()

        config.open_loop_ramps = phoenix6.configs.OpenLoopRampsConfigs()
        config.open_loop_ramps.with_duty_cycle_open_loop_ramp_period(0.01)

        motor_config = phoenix6.configs.MotorOutputConfigs()
        motor_config.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE

        config.motor_output = motor_config

        motor.configurator.apply(config)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getRate(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getRate(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getDistance())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getRate(), state.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getDistance(), state.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)

