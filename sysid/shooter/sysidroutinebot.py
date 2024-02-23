# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from commands2 import Command
from commands2.button import CommandPS4Controller, CommandXboxController
from commands2.sysid import SysIdRoutine
from subsystems.drive import Drive
from subsystems.shooter import Shooter
from wpilib import PS5Controller


class SysIdRoutineBot:
    """This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.subsystem = Shooter()
        # self.subsystem = Drive()

        # The driver's controller
        self.controller = CommandXboxController(0)

    def configureBindings(self) -> None:
        """Use this method to define bindings between conditions and commands. These are useful for
        automating robot behaviors based on button and sensor input.

        Should be called during :meth:`.Robot.robotInit`.

        Event binding methods are available on the :class:`.Trigger` class.
        """

        # Control the drive with split-stick arcade controls
        self.subsystem.setDefaultCommand(self.subsystem.defaultCommand())

        # Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        # once.
        self.controller.a().whileTrue(
            self.subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        self.controller.x().whileTrue(
            self.subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        self.controller.y().whileTrue(
            self.subsystem.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        self.controller.b().whileTrue(
            self.subsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

    def getAutonomousCommand(self) -> Command:
        """Use this to define the command that runs during autonomous.

        Scheduled during :meth:`.Robot.autonomousInit`.
        """

        # Do nothing
        return self.subsystem.run(lambda: None)
