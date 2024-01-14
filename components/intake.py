
import wpilib
import phoenix6

INTAKE_SPEED = 10
SHOOTER_SPEED = 10

class Intake:
    input_motor: phoenix6.hardware.TalonFX
    output_motor: phoenix6.hardware.TalonFX
    beam_sensor: wpilib.DigitalInput

    def setup(self):
        self.shot_done = False
        self.input_motor_setpoint = 0
        self.output_motor_setpoint = 0
        self.beam_sensor_last_detection = wpilib.Timer()
        self.beam_sensor_last_detection.start()


    def object_in_sensor(self):
        return self.beam_sensor.get()

    def intake(self):
        self.shot_done = False
        if self.object_in_sensor() is True:
            return True

        self.input_motor_setpoint = INTAKE_SPEED
        return False

    def shot(self, fire=False):
        if self.shot_done:
            return True

        if not self.object_in_sensor() and self.beam_sensor_last_detection.get() > 1:
            self.shot_done = True
            return True

        self.output_motor_setpoint = SHOOTER_SPEED

        if abs(SHOOTER_SPEED - self.output_motor.get_velocity()) > 1:
            # Speed not yet reached
            return

        if fire is True:
            self.input_motor_setpoint = INTAKE_SPEED


    def execute(self):
        if self.object_in_sensor():
            self.beam_sensor_last_detection.reset()

        self.input_motor.set_control(phoenix6.controls.VelocityDutyCycle(self.input_motor_setpoint))
        self.output_motor.set_control(phoenix6.controls.VelocityDutyCycle(self.output_motor_setpoint))

        self.input_motor_setpoint = 0
        self.output_motor_setpoint = 0
