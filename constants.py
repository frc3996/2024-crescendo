#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean constants. Don't use this for any other purpose!
#

import math

import wpilib

# Motors
kLeftMotor1Port = 1
kLeftMotor1Inverted = False
kLeftMotor2Port = 2
kLeftMotor2Inverted = False
kRightMotor1Port = 3
kRightMotor1Inverted = True
kRightMotor2Port = 4
kRightMotor2Inverted = True

# PID
kP = 5e-5
kI = 1e-6
kD = 0
kV = 0.000156
kMaxOutput = 1
kMinOutput = -1
max_rpm = 5700

# Smart Motion coefficients
max_vel = 2000  # rpm
max_acc = 1500
# max_vel = 100  # rpm
# max_acc = 10
min_vel = 0
allowed_err = 0

# The max velocity and acceleration for our autonomous when using ramsete
kMaxSpeedMetersPerSecond = 1
kMaxAccelerationMetersPerSecondSquared = 0.75
kMaxVoltage = 10

kMaxCentripetalAcceleration = 1

kMaxVoltage = 10

# sysid filtered results from 2022 (git 172d5c42, window size=10)
kS_linear = 1.0898
kV_linear = 3.1382
kA_linear = 1.7421

kS_angular = 2.424
kV_angular = 3.3557
kA_angular = 1.461

# Encoders
# kLeftEncoderPorts = (0, 1)
# kRightEncoderPorts = (2, 3)
# kLeftEncoderReversed = False
# kRightEncoderReversed = True
#
# kEncoderCPR = 1024
# kWheelDiameterInches = 6
# # Assumes the encoders are directly mounted on the wheel shafts
# kEncoderDistancePerPulse = (kWheelDiameterInches * math.pi) / kEncoderCPR
#
# # Hatch
# kHatchSolenoidModuleType = wpilib.PneumaticsModuleType.CTREPCM
# kHatchSolenoidModule = 0
# kHatchSolenoidPorts = (0, 1)
#
# # Autonomous
# kAutoDriveDistanceInches = 60
# kAutoBackupDistanceInches = 20
# kAutoDriveSpeed = 0.5
#
# # Operator Interface
# kDriverControllerPort = 0
#
# Physical parameters
kDriveTrainMotorCount = 2
kTrackWidth = 0.381 * 2
kGearingRatio = 8
kWheelRadius = 0.0508

# kEncoderResolution = -


# PCM
pcm_can_id = 30
