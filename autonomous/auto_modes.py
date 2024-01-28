import ntcore
from magicbot.state_machine import timed_state

from common import path_helper
from components import robot_actions, swervedrive

from .base_auto import BaseAuto


class Path1(BaseAuto):
    MODE_NAME = "Path1"
    DEFAULT = True

    # Injection
    drivetrain: swervedrive.SwerveDrive
    nt: ntcore.NetworkTable
    robot_actions: robot_actions.RobotActions

    @timed_state(duration=10, next_state="execute_path_1", first=True)
    def init(self):
        self.path_part_1 = path_helper.PathHelper(self.drivetrain, "part_1")
        self.path_part_1.init_path(force_robot_starting_position=True)

        self.path_part_2 = path_helper.PathHelper(self.drivetrain, "test_path2")

        self.next_state("execute_path_1")

    @timed_state(duration=10, next_state="shoot")
    def execute_path_1(self):
        self.path_part_1.auto_move()
        if self.path_part_1.path_reached_end():
            self.next_state("shoot")

    @timed_state(duration=10, next_state="path_part_2_init")
    def shoot(self):
        self.next_state("path_part_2_init")
        if self.robot_actions.autoshoot_speaker():
            self.next_state("path_part_2_init")

    @timed_state(duration=10, next_state="execute_path_2")
    def path_part_2_init(self):
        self.path_part_2.init_path()
        self.next_state("execute_path_2")

    @timed_state(duration=10, next_state="finish")
    def execute_path_2(self):
        self.path_part_2.auto_move()
        if self.path_part_2.path_reached_end():
            self.next_state("finish")
