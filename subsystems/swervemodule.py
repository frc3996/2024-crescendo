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
import constants
from typing import cast

kWheelRadius = 0.025
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau

class RikiCANcoder(phoenix6.hardware.CANcoder):
    dpr: float
    def __init__(self, device_id: int, canbus: str = "", name = ""):
        super().__init__(device_id, canbus)
        self.name = name
        cancoder_config = phoenix6.configs.CANcoderConfiguration()
        self.configurator.apply(cancoder_config)

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

class RikiTalonFXConfiguration(phoenix6.configs.TalonFXConfiguration, phoenix6.configs.config_groups.SupportsSerialization):
    def __init__(self, inverted=False, name=""):
        super().__init__()
        
        self.name = name
        self.feedback.feedback_sensor_source = phoenix6.configs.talon_fx_configs.FeedbackSensorSourceValue.ROTOR_SENSOR
        self.feedback.feedback_remote_sensor_id = 0

        wpilib.SmartDashboard.putNumber("P Gain", constants.kP)
        wpilib.SmartDashboard.putNumber("I Gain", constants.kI)
        wpilib.SmartDashboard.putNumber("D Gain", constants.kD)
        wpilib.SmartDashboard.putNumber("Feed Forward", constants.kV)

        self.open_loop_ramps = phoenix6.configs.OpenLoopRampsConfigs()
        self.open_loop_ramps.with_duty_cycle_open_loop_ramp_period(0.1)

        motor_config = phoenix6.configs.MotorOutputConfigs()
        if inverted:
            motor_config.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        else:
            motor_config.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE

        self.motor_output = motor_config

    def updateConfigs(self):
        """
        That's a bit ugly, but we don't want to apply the config every 20ms
        """
        p = cast(float, wpilib.SmartDashboard.getNumber(f"{self.name} P Gain", 0))
        i = cast(float, wpilib.SmartDashboard.getNumber(f"{self.name} I Gain", 0))
        d = cast(float, wpilib.SmartDashboard.getNumber(f"{self.name} D Gain", 0))
        v = cast(float, wpilib.SmartDashboard.getNumber(f"{self.name} Feed Forward", 0))

        apply = False
        if p != self.slot0.k_p:
            # Proportional Gain
            self.slot0.k_p = p
            apply = True
        if i != self.slot0.k_i:
            # Integral Gain
            self.slot0.k_i = i
            apply = True
        if d != self.slot0.k_d:
            # Derivative Gain
            self.slot0.k_d = d
            apply = True
        if v != self.slot0.k_v:
            # Velocity Feedforward Gain
            self.slot0.k_v = v
            apply = True

        # Static Feedforward Gain
        self.slot0.k_s = 0
        # Acceleration Feedforward Gain
        self.slot0.k_a = 0
        # Gravity Feedforward Gain
        self.slot0.k_g = 0

        return apply


class RikiTalonFX(phoenix6.hardware.TalonFX):
    dpr: float
    def __init__(self, device_id: int, canbus: str = "", name = ""):
        super().__init__(device_id, canbus)
        self.name = name
        self.config = RikiTalonFXConfiguration(inverted=False, name=name)

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

    def setVelocity(self, velocity):
        "Convert from velocity in m/s to rot/s"
        self.set_control(phoenix6.controls.VelocityVoltage(velocity/self.dpr))

    def setRotation(self, angle):
        "Convert from angle (in rad) to rotation"
        self.set_control(phoenix6.controls.PositionVoltage(angle/self.dpr))

    def updateConfigs(self): 
        if self.config.updateConfigs():
            self.configurator.apply(self.config)

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

        # Setting up motors
        self.driveMotor = RikiTalonFX(driveMotorChannel, name=f"{name} drive")
        self.driveMotor.set_position(0)

        self.turningMotor = RikiTalonFX(turningMotorChannel, name=f"{name} turning")

        # Seting up encoders
        self.driveEncoder = self.driveMotor

        self.turningEncoder = RikiCANcoder(turningEncoderChannel, name=name)
        self.turningEncoder.set_position(0)


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
        
        # Update the pid from network tables
        self.driveMotor.updateConfigs()
        self.turningMotor.updateConfigs()

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

        self.driveMotor.setVelocity(state.speed)
        self.turningMotor.setRotation(state.angle.radians())
