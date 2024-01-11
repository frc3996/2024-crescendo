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

kWheelRadius = 2.5
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class RikiCANcoder(phoenix6.hardware.CANcoder):
    dpr: float
    def __init__(self, device_id: int, canbus: str = ""):
        super().__init__(device_id, canbus)

    def setDistancePerRotation(self, dpr: float):
        self.dpr = dpr 

    def getRate(self):
        """
        Get the current rate of the encoder. Units are distance per second as scaled by the value from setDistancePerRotation().
        """
        return self.dpr * self.get_velocity().value

    def getDistance(self):
        """
        Get the distance the robot has driven since the last reset as scaled by the value from setDistancePerRotation(double).
        """
        distance = self.get_position().value * self.dpr
        # print(f"RikiCANcoder {self.name}  distance {current_position} {self.previous_position} {distance}")
        return distance
        
class RikiTalonFX(phoenix6.hardware.TalonFX):
    dpr: float
    def __init__(self, device_id: int, canbus: str = ""):
        super().__init__(device_id, canbus)

    def setDistancePerRotation(self, dpr: float):
        self.dpr = dpr 

    def getRate(self):
        """
        Get the current rate of the encoder. Units are distance per second as scaled by the value from setDistancePerRotation().
        """
        return self.dpr * self.get_rotor_velocity().value

    def getDistance(self):
        """
        Get the distance the robot has driven since the last reset as scaled by the value from setDistancePerRotation(double).
        """
        distance = self.dpr * self.get_rotor_position().value
        # print(f"RikiTalonFX {self.name} distance {current_position} {self.previous_position} {distance}")
        return distance
    
    def setVoltage(self, voltage):
        self.set_control(phoenix6.controls.VoltageOut(voltage))
        self.set_control(phoenix6.controls.VoltageOut(voltage))

        

class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        turningEncoderChannel: int,
        name: str,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel
        :param turningMotorChannel
        :param driveEncoderChannel
        :param turningEncoderChannel
        """
        self.driveMotor = RikiTalonFX(driveMotorChannel)
        self.driveMotor.name = name
        self.driveEncoder = self.driveMotor

        self.turningMotor = RikiTalonFX(turningMotorChannel)
        self.turningMotor.name = name

        self.turningEncoder = RikiCANcoder(turningEncoderChannel)
        self.turningEncoder.name = name

        self.init_driveMotor(self.driveMotor)
        self.init_turningMotor(self.turningMotor)
        self.init_cancoder(self.turningEncoder)


        # # Gains are for example purposes only - must be determined for your own robot!
        # self.drivePIDController = wpimath.controller.PIDController(-0.008, 0, 0)

        # # Gains are for example purposes only - must be determined for your own robot!
        # self.turningPIDController = wpimath.controller.ProfiledPIDController(
        #     -0.008,
        #     0,
        #     0,
        #     wpimath.trajectory.TrapezoidProfile.Constraints(
        #         kModuleMaxAngularVelocity,
        #         kModuleMaxAngularAcceleration,
        #     ),
        # )

        # # Gains are for example purposes only - must be determined for your own robot!
        # self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0)
        # self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0)

        # Set the distance per rotation for the drive encoder.
        self.driveMotor.setDistancePerRotation(
            math.tau * kWheelRadius
        )

        # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        self.turningEncoder.setDistancePerRotation(math.tau)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        # self.turningPIDController.enableContinuousInput(-math.pi, math.pi)


    def init_cancoder(self, cancoder: phoenix6.hardware.CANcoder):
        cancoder_config = phoenix6.configs.CANcoderConfiguration()
        cancoder.configurator.apply(cancoder_config)
        cancoder.set_position(0)

    def init_driveMotor(self, motor: phoenix6.hardware.TalonFX):
        config = phoenix6.configs.TalonFXConfiguration()

        config.open_loop_ramps = phoenix6.configs.OpenLoopRampsConfigs()
        config.open_loop_ramps.with_duty_cycle_open_loop_ramp_period(0.1)

        motor_config = phoenix6.configs.MotorOutputConfigs()
        motor_config.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        config.motor_output = motor_config
        motor.set_position(0)
        motor.configurator.apply(config)

    def init_turningMotor(self, motor: phoenix6.hardware.TalonFX):
        config = phoenix6.configs.TalonFXConfiguration()

        config.open_loop_ramps = phoenix6.configs.OpenLoopRampsConfigs()
        config.open_loop_ramps.with_duty_cycle_open_loop_ramp_period(0.01)

        motor_config = phoenix6.configs.MotorOutputConfigs()
        motor_config.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE

        config.motor_output = motor_config
        motor.set_position(0)
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

        # # Calculate the drive output from the drive PID controller.
        # driveOutput = self.drivePIDController.calculate(
        #     self.driveEncoder.getRate(), state.speed
        # )

        # driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller.
        # turnOutput = self.turningPIDController.calculate(
        #     self.turningEncoder.getDistance(), state.angle.radians()
        # )

        # turnFeedforward = self.turnFeedforward.calculate(
        #     self.turningPIDController.getSetpoint().velocity
        # )

        # self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        # self.turningMotor.setVoltage(turnOutput + turnFeedforward)

        print(f"state.speed {state.speed}, state.angle.rad {state.angle.radians()}")
        self.driveMotor.set_control(phoenix6.controls.DutyCycleOut(state.speed))
        self.turningMotor.set_control(phoenix6.controls.PositionDutyCycle(state.angle.radians(), 1))
