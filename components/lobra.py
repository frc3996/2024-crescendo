
import math
import wpilib
import rev
import ntcore
from wpimath import controller, trajectory


# https://robotpy.readthedocs.io/projects/utilities/en/stable/magicbot.html
from magicbot import feedback, tunable, will_reset_to, state_machine


class LoBras:
    head_motor: rev.CANSparkMax
    arm_motor_left: rev.CANSparkMax
    arm_motor_right: rev.CANSparkMax
    arm_limit_switch: wpilib.DigitalInput
    pneumatic_brake: wpilib.Solenoid
    nt: ntcore.NetworkTable

    def setup(self):
        self.arm_motor_right.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.arm_motor_right.setOpenLoopRampRate(0.25)
        self.arm_motor_right.follow(self.arm_motor_left, invert=True)

        self.arm_motor_left.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.arm_motor_left.setOpenLoopRampRate(0.25)
        self.arm_position_encoder = self.arm_motor_left.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        constraint = trajectory.TrapezoidProfileRadians.Constraints(0, 0)
        self.arm_pid = controller.ProfiledPIDControllerRadians(0, 0, 0, constraint)
        self.current_arm_target = self.arm_position_encoder.getPosition()

        self.head_motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.head_motor.setOpenLoopRampRate(0.25)
        self.head_position_encoder = self.head_motor.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        constraint = trajectory.TrapezoidProfileRadians.Constraints(0, 0)
        self.head_pid = controller.ProfiledPIDControllerRadians(0, 0, 0, constraint)
        self.current_head_target = self.head_position_encoder.getPosition()

        self.nt.putNumber("lobras/arm_pid/Kp", 0.002)
        self.nt.putNumber("lobras/arm_pid/Ki", 0)
        self.nt.putNumber("lobras/arm_pid/Kd", 0)
        self.nt.putNumber("lobras/arm_pid/max_vel", 0.02)
        self.nt.putNumber("lobras/arm_pid/max_acc", 0.02)

        self.nt.putNumber("lobras/head_pid/Kp", 0.002)
        self.nt.putNumber("lobras/head_pid/Ki", 0)
        self.nt.putNumber("lobras/head_pid/Kd", 0)
        self.nt.putNumber("lobras/head_pid/max_vel", 0.02)
        self.nt.putNumber("lobras/head_pid/max_acc", 0.02)

        self.update_nt_config()

    def update_nt_config(self):
        constraint = trajectory.TrapezoidProfileRadians.Constraints(
            self.nt.getNumber()("lobras/arm_pid/max_vel", 0),
            self.nt.getNumber()("lobras/arm_pid/max_acc", 0)
        )
        self.arm_pid.setPID(
            self.nt.getNumber()("lobras/arm_pid/Kp", 0),
            self.nt.getNumber()("lobras/arm_pid/Ki", 0),
            self.nt.getNumber()("lobras/arm_pid/Kd", 0)
        )
        self.arm_pid.setConstraints(constraint)


        constraint = trajectory.TrapezoidProfileRadians.Constraints(
            self.nt.getNumber()("lobras/head_pid/max_vel", 0),
            self.nt.getNumber()("lobras/head_pid/max_acc", 0)
        )
        self.head_pid.setPID(
            self.nt.getNumber("lobras/head_pid/Kp", 0),
            self.nt.getNumber("lobras/head_pid/Ki", 0),
            self.nt.getNumber("lobras/head_pid/Kd", 0)
        )
        self.head_pid.setConstraints(constraint)


    def set_angle(self, arm_position, head_position):
        """Set the target angles, in degrees, from 0 to 360"""
        self.current_arm_target = arm_position
        self.current_head_target = head_position

    @feedback
    def current_arm_position(self):
        return self.arm_position_encoder.getPosition() * 360

    @feedback
    def current_head_position(self):
        return self.head_position_encoder.getPosition() * 360

    def intake_mode(self):
        # TODO
        # Place la tête et le bras en position intake
        # self.__set_positions(xx, yy)
        pass

    def speaker_mode(self, fire=False):
        # TODO
        # Place le bras en mode speaker
        # Utilise la limelight pour viser avec la tete

        if fire is False:
            return

        # Tire si limelight + vitesse Ok

    def amp_mode(self):
        # TODO
        # Place la tête et le bras en position amp
        pass

    def note_trap_mode(self):
        # TODO
        # Place la tête et le bras en position note_trap
        pass

    def execute(self):
        self.nt.putNumber("lobras/arm_current_position", self.current_arm_position())
        self.nt.putNumber("lobras/arm_target_position", self.current_arm_target)
        self.nt.putNumber("lobras/head_current_position", self.current_head_position())
        self.nt.putNumber("lobras/head_target_position", self.current_head_target)

        arm_error = self.arm_pid.calculate(math.radians(self.current_arm_position()), math.radians(self.current_arm_target))
        head_error = self.head_pid.calculate(math.radians(self.current_head_position()), math.radians(self.current_head_target))

        print(arm_error, head_error)
        # self.arm_motor_left.set(arm_error)
        # self.arm_motor_left.set(head_error)
