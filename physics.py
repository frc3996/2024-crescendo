#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain, linear_deadzone
import wpimath.geometry
import wpimath.kinematics
from components.swervedrive import rotate_vector


# import typing

# if typing.TYPE_CHECKING:
from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot objet
        """

        self.physics_controller = physics_controller
        self.robot = robot
        self.startingPositionSet = wpimath.geometry.Pose2d()
        self.physics_controller.field.setRobotPose(self.startingPositionSet)


    def update_sim(self, now, tm_diff):
        # print(self.robot.drivetrain.target_angle - self.robot.drivetrain.get_angle())
        vector = rotate_vector([self.robot.drivetrain.chassis_speed.vx, self.robot.drivetrain.chassis_speed.vy], -self.physics_controller.get_pose().rotation().degrees())
        # print(vector[0], vector[1], self.robot.drivetrain.get_angle())
        cspeed = wpimath.kinematics.ChassisSpeeds(vector[0], vector[1], self.robot.drivetrain.sim_rot*200)
        pose = self.physics_controller.drive(
            cspeed, tm_diff
        )
        # print(pose.rotation().degrees())
        self.robot.drivetrain.set_sim_angle_offset(pose.rotation().degrees())

        self.robot.drivetrain.resetPose(pose)
