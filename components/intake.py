
import wpilib
import rev

INTAKE_SPEED = 10
SHOOTER_SPEED = 10

class Intake:
    input_motor: rev.CANSparkMax
    output_motor: rev.CANSparkMax
    beam_sensor: wpilib.AnalogInput

    def setup(self):
        self.shot_done = False
        self.input_motor_setpoint = 0
        self.output_motor_setpoint = 0
        self.beam_sensor_last_detection = wpilib.Timer()
        self.beam_sensor_last_detection.start()
        self.object_intaken = False

    def remove_me_shoot_test(self):
        print("JE SHOOT MAIS PAS VRAIMENT, EFFACEZ MOI!!!!!")

    def object_in_sensor(self):
        return self.beam_sensor.getValue() < 324

    def is_object_intaken(self):
        return self.object_intaken

    def intake(self):
        self.shot_done = False
        if self.object_in_sensor() is True or self.object_intaken is True:
            self.object_intaken = True
            return True

        self.input_motor_setpoint = INTAKE_SPEED
        return False

    def shot(self, fire=False):
        if self.shot_done:
            return True

        if not self.object_in_sensor() and self.beam_sensor_last_detection.get() > 1:
            self.object_intaken = False
            self.shot_done = True
            return True

        self.output_motor_setpoint = SHOOTER_SPEED

        if abs(SHOOTER_SPEED - self.output_motor.get_velocity()) > 1:
            # Speed not yet reached
            return

        if fire is True:
            self.input_motor_setpoint = INTAKE_SPEED


    def execute(self):
        print(self.object_in_sensor())
        if self.object_in_sensor():
            self.beam_sensor_last_detection.reset()

        # self.input_motor.set_control(phoenix6.controls.VelocityDutyCycle(self.input_motor_setpoint))
        # self.output_motor.set_control(phoenix6.controls.VelocityDutyCycle(self.output_motor_setpoint))

        self.input_motor_setpoint = 0
        self.output_motor_setpoint = 0
