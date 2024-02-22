import json
import os

from magicbot import tunable
from magicbot.state_machine import AutonomousStateMachine, StateMachine, state
from wpimath import geometry

from common.path_helper import PathHelper
from components.robot_actions import (ActionGrabAuto, ActionHighShootAuto,
                                      ActionLowShootAuto, ActionShootAmpAuto,
                                      ActionStow)
from components.swervedrive import SwerveDrive


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

    with open(file, "r") as f:
        auto_json = json.load(f)

    return auto_json


class RunAuto(AutonomousStateMachine):
    MODE_NAME = "RunAuto"
    DEFAULT = True

    # Tunables
    auto_name = tunable("1 amp + 3_speakers")
    path_kp = tunable(2)
    path_ki = tunable(0)
    path_kd = tunable(0)
    path_profile = tunable(2)

    # Injection
    drivetrain: SwerveDrive
    actionGrabAuto: ActionGrabAuto
    actionLowShootAuto: ActionLowShootAuto
    actionHighShootAuto: ActionHighShootAuto
    actionShootAmpAuto: ActionShootAmpAuto
    actionStow: ActionStow

    current_command = {}

    @state(first=True)
    def get_auto_mode(self):
        json_commands = get_next_auto_command(self.auto_name)
        self.auto_commands = json_commands["command"]["data"]["commands"]  # type: list
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
    def named(self, initial_call):
        command_name = self.current_command["data"]["name"]
        command = getattr(self, command_name)  # type: StateMachine
        command.engage()
        if not command.is_executing and not initial_call:
            self.next_state("execute_next_command")

    @state
    def path(self, initial_call):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        if initial_call:
            path_name = self.current_command["data"]["pathName"]
            self.auto_path = PathHelper(
                self.drivetrain,
                path_name,
                kp=self.path_kp,
                ki=self.path_ki,
                kd=self.path_kd,
                profile_kp=self.path_profile,
            )
            self.auto_path.init_path()

        self.auto_path.auto_move()
        if self.auto_path.robot_reached_end_position():
            self.next_state("execute_next_command")
