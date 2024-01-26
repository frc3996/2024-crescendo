
import wpilib
import rev
import ntcore
from wpimath.controller import PIDController

class LoBras:
    head_motor: rev.CANSparkMax
    arm_motor_left: rev.CANSparkMax
    arm_motor_right: rev.CANSparkMax
    arm_limit_switch: wpilib.DigitalInput
    pneumatic_brake: wpilib.Solenoid
    nt: ntcore.NetworkTable

    def setup(self):
        self.current_arm_target = 0
        self.arm_motor_left.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        # self.arm_motor_right.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        # self.arm_motor_left.setOpenLoopRampRate(0.25)
        # self.arm_motor_right.setOpenLoopRampRate(0.25)
        # self.arm_motor_right.follow(self.arm_motor_left, invert=True)
        self.arm_pid_controller = self.arm_motor_left.getPIDController()
        self.arm_position_encoder = self.arm_motor_left.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        self.arm_pid_controller.setFeedbackDevice(self.arm_position_encoder)
        # self.arm_position_encoder.setZeroOffset(self.arm_position_encoder.getPosition())
        # self.arm_position_encoder = self.arm_motor_left.getEncoder()
        # self.arm_position_encoder.setPosition(0)
        self.arm_pid = PIDController(0, 0, 0)

        self.nt.putNumber("lobras/arm_pid/Kp", 0.057)
        self.nt.putNumber("lobras/arm_pid/Ki", 0)
        self.nt.putNumber("lobras/arm_pid/Kd", 0)

        kP = 0.000
        kI = 0  # 0.0001
        kD = 0
        kIz = 0
        kFF = 0.02
        kMaxOutput = 1
        kMinOutput = -1
        self.arm_pid_controller.setP(kP)
        self.arm_pid_controller.setI(kI)
        self.arm_pid_controller.setD(kD)
        self.arm_pid_controller.setIZone(kIz)
        self.arm_pid_controller.setFF(kFF)
        self.arm_pid_controller.setOutputRange(kMinOutput, kMaxOutput)

    def update_nt(self):


    def set_angle(self, arm_position, head_position):
        self.current_arm_target = arm_position
        self.current_head_target = head_position

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
        print(self.current_arm_target, self.arm_position_encoder.getPosition())
        self.arm_motor_left.set(self.current_arm_target - self.arm_position_encoder.getPosition())
