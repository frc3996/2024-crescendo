import typing
from dataclasses import dataclass
from typing import cast

import wpilib
import wpilib.simulation
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics import drivetrains
import hal.simulation
from pyfrc.physics.core import PhysicsInterface
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.system.plant import DCMotor
from pyfrc.physics.units import units

import constants

if typing.TYPE_CHECKING:
    from robot import MyRobot

RED = wpilib.Color8Bit(wpilib.Color.kRed)
BLUE = wpilib.Color8Bit(wpilib.Color.kBlue)
GREEN = wpilib.Color8Bit(wpilib.Color.kGreen)
GRAY = wpilib.Color8Bit(wpilib.Color.kGray)

EDGE = 8
INDEXER_LEN = 12
SENSOR_Y = 15

ENTRY_MOTOR_START = 0
ENTRY_MOTOR_END = EDGE + 2

ENTRY_SENSOR_POS = 9
EXIT_SENSOR_POS = 20

# climber simulation
CLIMBER_X = 21
CLIMBER_RAISED_LEN = 18
CLIMBER_LOWERED_LEN = 12


@dataclass
class Shape:
    root: wpilib.MechanismRoot2d
    items: typing.List[wpilib.MechanismLigament2d]

    def setPosition(self, x: float, y: float):
        self.root.setPosition(x, y)

    def setColor(self, c: wpilib.Color8Bit):
        for item in self.items:
            item.setColor(c)


class Mechanism(wpilib.Mechanism2d):
    def __init__(self, width: float, height: float) -> None:
        super().__init__(width, height)

    def make_point(self, name: str, cx: float, cy: float, sz: int = 10) -> Shape:
        root = self.getRoot(name, cx, cy)
        return self._make(root, 0.50, 0, 1, sz)

    def make_triangle(
        self, name: str, cx: float, cy: float, side: float, line_width=6
    ) -> Shape:
        x = cx + side / 2
        y = cy - side / 2
        root = self.getRoot(name, x, y)
        return self._make(root, side, 120, 3, line_width)

    def make_hex(
        self, name: str, x: float, y: float, side: float, line_width=6
    ) -> Shape:
        # x, y are near bottom right corner
        root = self.getRoot(name, x, y)
        return self._make(root, side, 60, 6, line_width)

    def _make(
        self,
        root: wpilib.MechanismRoot2d,
        side: float,
        angle: float,
        n: int,
        line_width,
    ) -> Shape:
        item = root
        items = []
        for i in range(n):
            item = item.appendLigament(f"{i}", side, angle, line_width)
            items.append(item)

        return Shape(root, items)

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller: PhysicsInterface = physics_controller

        # Setup drive, as long as the ports are matching..
        self.physics_controller = physics_controller

        # Motors
        self.l_motor = wpilib.simulation.PWMSim(1)
        self.r_motor = wpilib.simulation.PWMSim(2)

        # NavX (SPI interface)
        self.navx = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                    # robot mass
            10.71,                              # drivetrain gear ratio
            2,                                  # motors per side
            22 * units.inch,                    # robot wheelbase
            23 * units.inch + bumper_width * 2, # robot width
            32 * units.inch + bumper_width * 2, # robot length
            6 * units.inch,                     # wheel diameter
        )

        self.draw_robot()

    def draw_robot(self):
        # drawn robot model (scale 1inch=1)

        self.model = Mechanism(30, 40)
        wpilib.SmartDashboard.putData("Model", self.model)

        outside = self.model.getRoot("outside", EDGE, 10)
        outside.appendLigament("l1", INDEXER_LEN, 0, color=GRAY)

        inside = self.model.getRoot("inside", EDGE, 20)
        inside.appendLigament("l1", INDEXER_LEN, 0, color=GRAY)

        self.entry_sensor_pt = self.model.make_point(
            "entry-sensor", ENTRY_SENSOR_POS, SENSOR_Y
        )
        
        self.exit_sensor_pt = self.model.make_point(
            "exit-sensor",
            EXIT_SENSOR_POS,
            SENSOR_Y,
        )

        self.entry_motor_pt = self.model.make_hex("intake-motor", 4, 20, 2.5, 4)
        self.entry_motor_pt.setColor(GRAY)
        
        self.shooter = self.model.make_hex("shooter", 27, 20, 3, 2)
        self.shooter.setColor(GRAY)

        # climber
        climber = self.model.getRoot("climber", CLIMBER_X, 20)
        self.climb_extender = climber.appendLigament(
            "ext", CLIMBER_LOWERED_LEN, 90, color=GRAY
        )
        l1 = self.climb_extender.appendLigament("l1", 1, 60, color=GRAY)
        l2 = l1.appendLigament("l2", 1, 60, color=GRAY)
        l2.appendLigament("l3", 1, 60, color=GRAY)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        transform = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        pose = self.physics_controller.move_robot(transform)

        # Update the gyro simulation
        self.navx_yaw.set(-pose.rotation().degrees())

        # Update the Field2d
        field = cast(wpilib.Field2d, self.physics_controller.field)
        field.setRobotPose(pose)
