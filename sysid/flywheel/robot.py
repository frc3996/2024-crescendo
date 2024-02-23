#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import numpy
import rev
import wpilib
import wpimath
import wpimath.controller
import wpimath.estimator
import wpimath.system
import wpimath.system.plant
import wpimath.units

kMotorPort = 20
kJoystickPort = 0
kSpinupRadPerSec = wpimath.units.rotationsPerMinuteToRadiansPerSecond(500.0)

# Volts per (radian per second)
kFlywheelKv = 0.023

# Volts per (radian per second squared)
kFlywheelKa = 0.001


class MyRobot(wpilib.TimedRobot):
    """
    This is a sample program to demonstrate how to use a state-space controller to control a
    flywheel.
    """

    def robotInit(self) -> None:
        # The plant holds a state-space model of our flywheel. This system has the following properties:
        #
        # States: [velocity], in radians per second.
        # Inputs (what we can "put in"): [voltage], in volts.
        # Outputs (what we can measure): [velocity], in radians per second.
        #
        # The Kv and Ka constants are found using the FRC Characterization toolsuite.
        # self.flywheelPlant = (
        #     wpimath.system.plant.LinearSystemId.identifyVelocitySystemRadians(
        #         kFlywheelKv, kFlywheelKa
        #     )
        # )
        # The plant holds a state-space model of our flywheel. This system has the
        # following properties:
        #
        # States: [velocity], in radians per second.
        # Inputs (what we can "put in"): [voltage], in volts.
        # Outputs (what we can measure): [velocity], in radians per second.
        self.flywheelPlant = wpimath.system.plant.LinearSystemId.flywheelSystem(
            wpimath.system.plant.DCMotor.NEO(20),
            0.00032,  # kg * m^2
            1,
        )

        # The observer fuses our encoder data and voltage inputs to reject noise.
        self.observer = wpimath.estimator.KalmanFilter_1_1_1(
            self.flywheelPlant,
            (3,),  # How accurate we think our model is
            (0.01,),  # How accurate we think our encoder data is
            0.020,
        )

        # A LQR uses feedback to create voltage commands.
        self.controller = wpimath.controller.LinearQuadraticRegulator_1_1(
            self.flywheelPlant,
            (8.0,),  # Velocity error tolerance
            (12,),  # Control effort (voltage) tolerance
            0.020,
        )

        # The state-space loop combines a controller, observer, feedforward and plant for easy control.
        self.loop = wpimath.system.LinearSystemLoop_1_1_1(
            self.flywheelPlant, self.controller, self.observer, 12.0, 0.020
        )

        self.motor = rev.CANSparkMax(20, rev.CANSparkMax.MotorType.kBrushless)
        self.motor.restoreFactoryDefaults()

        # An encoder set up to measure flywheel velocity in radians per second.
        # self.encoder = wpilib.Encoder(kEncoderAChannel, kEncoderBChannel)
        self.encoder = self.motor.getEncoder()

        # A joystick to read the trigger from.
        self.joystick = wpilib.Joystick(kJoystickPort)

        # We go 2 pi radians per 4096 clicks.
        # self.encoder.setDistancePerPulse(math.tau / 4096)

    def teleopInit(self) -> None:
        self.loop.reset(numpy.array([self.encoder.getVelocity()]))

    def teleopPeriodic(self) -> None:
        # Sets the target speed of our flywheel. This is similar to setting the setpoint of a
        # PID controller.
        if self.joystick.getTriggerPressed():
            # We just pressed the trigger, so let's set our next reference
            self.loop.setNextR(numpy.array([kSpinupRadPerSec]))

        elif self.joystick.getTriggerReleased():
            # We just released the trigger, so let's spin down
            self.loop.setNextR(numpy.array([0]))

        # Correct our Kalman filter's state vector estimate with encoder data.
        radPerMin = self.encoder.getVelocity()
        radPerSec = wpimath.units.rotationsPerMinuteToRadiansPerSecond(radPerMin)
        self.loop.correct(numpy.array([radPerSec]))

        # Update our LQR to generate new voltage commands and use the voltages to predict the next state with out Kalman filter.
        self.loop.predict(0.020)

        # Send the new calculated voltage to the motors.
        # voltage = duty cycle * battery voltage, so
        # duty cycle = voltage / battery voltage
        nextVoltage = self.loop.U()
        print(f"Voltage {nextVoltage}")
        self.motor.setVoltage(nextVoltage.item())
