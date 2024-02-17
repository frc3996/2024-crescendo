import ntcore
from magicbot.state_machine import timed_state, AutonomousStateMachine, StateMachine, state
from magicbot import tunable, feedback

from common.path_helper import PathHelper, PathPlannerPath
from components import robot_actions, swervedrive

from .base_auto import BaseAuto

from components.robot_actions import (ActionGrabAuto, ActionLowShoot, ActionLowShootAuto, ActionHighShootAuto, ActionShoot,
                                      ActionShootAmp, ActionStow, ActionDewinch, ActionWinch, ActionDummy,
                                      ActionShootAmpAuto, ActionGrabManual, ActionOuttake, ActionShootAmpAssisted,
                                      )
import json
import os
from wpimath import geometry


# class Path1(BaseAuto):
#     MODE_NAME = "Path1"
#     DEFAULT = True

#     # Injection
#     drivetrain: swervedrive.SwerveDrive
#     nt: ntcore.NetworkTable
#     actionShoot: robot_actions.ActionShoot

#     @timed_state(duration=10, next_state="execute_path_1", first=True)
#     def init(self):
#         # self.auto_file = AutoHelper(self.drivetrain, "amp_shoot")
#         self.path_part_1 = PathHelper(self.drivetrain, "amp_shoot")
#         self.path_part_1.init_path(force_robot_starting_position=True)

#         self.path_part_2 = PathHelper(self.drivetrain, "amp_shoot")

#         self.next_state("execute_path_1")

#     @timed_state(duration=10, next_state="shoot")
#     def execute_path_1(self):
#         self.path_part_1.auto_move()
#         if self.path_part_1.path_reached_end():
#             self.next_state("shoot")

#     @timed_state(duration=10, next_state="path_part_2_init")
#     def shoot(self):
#         self.next_state("path_part_2_init")
#         if self.actionShoot.engage():
#             self.next_state("path_part_2_init")

#     @timed_state(duration=10, next_state="execute_path_2")
#     def path_part_2_init(self):
#         self.path_part_2.init_path()
#         self.next_state("execute_path_2")

#     @timed_state(duration=10, next_state="finish")
#     def execute_path_2(self):
#         self.path_part_2.auto_move()
#         if self.path_part_2.path_reached_end():
#             self.next_state("finish")


def get_next_auto_command(auto_name):
    if not auto_name.endswith(".auto"):
        auto_name += ".auto"
    file = os.path.join(
        os.path.dirname(__file__),
        "..",
        "deploy",
        "pathplanner",
        "autos",
        auto_name,
    )

    with open(file, 'r') as f:
        auto_json = json.load(f)

    return auto_json


class RunAuto(AutonomousStateMachine):
    MODE_NAME = "RunAuto"
    DEFAULT = True

    auto_name = tunable("3_notes")
    path_kp = tunable(2)
    path_ki = tunable(0)
    path_kd = tunable(0)
    path_profile = tunable(2)

    drivetrain: swervedrive.SwerveDrive

    actionGrabAuto: ActionGrabAuto
    actionLowShootAuto: ActionLowShootAuto
    actionHighShootAuto: ActionHighShootAuto
    actionShootAmpAuto: ActionShootAmpAuto
    actionStow: ActionStow

    current_command = None

    @state(first=True)
    def get_auto_mode(self):
        json_commands = get_next_auto_command(self.auto_name)
        self.auto_commands = json_commands["command"]["data"]["commands"]  #type: list
        reset_pose = json_commands["startingPose"]
        new_pose = geometry.Pose2d(
            reset_pose["position"]["x"],
            reset_pose["position"]["y"],
            reset_pose["rotation"],
        )
        self.drivetrain.resetPose(new_pose)
        self.next_state("execute_next_command")

    @state()
    def execute_next_command(self):
        if self.auto_commands:
            self.current_command = self.auto_commands.pop(0)
        else:
            self.done()
        self.next_state(self.current_command["type"])

    @state
    def named(self):
        print(self.current_command)
        command_name = self.current_command["data"]["name"]
        command = getattr(self, command_name) #type: StateMachine
        command.engage()
        if not command.is_executing:
            self.next_state("execute_next_command")

    @state
    def path(self, initial_call):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        if initial_call:
            print(self.current_command)
            path_name = self.current_command["data"]["pathName"]
            self.auto_path = PathHelper(self.drivetrain, path_name, kp=self.path_kp, ki=self.path_ki, kd=self.path_kd, profile_kp=self.path_profile)
            self.auto_path.init_path()

        self.auto_path.move_to_end()
        if self.auto_path.robot_reached_end_position():
            self.next_state("execute_next_command")
