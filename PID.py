import numpy as np
__name__ = "PIDLoop"


class PIDLoop:

    def __init__(self, kp=1., ki=1., kd=1., set_point=0., min_output=-1., max_output=1., max_integral=1,  time=0.):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.Set_point = set_point
        self.Min_output = min_output
        self.Max_output = max_output
        self.Max_integral = max_integral

        self.Error = 0
        self.Proportional = 0
        self.Derivative = 0
        self.Integral = 0
        self.Last_Error = 0
        self.Last_time = time
        self.Time_diff = 0
        self.Output = 0

    def update(self, feedback_value, time):
        self.Error = self.Set_point - feedback_value
        self.Time_diff = time - self.Last_time
        self.Last_time = time

        self.Proportional = self.Kp * self.Error

        self.Integral += self.Ki * self.Error * self.Time_diff
        # check if integral still in bounds
        if self.Integral < -self.Max_integral:
            self.Integral = -self.Max_integral
        if self.Integral > self.Max_integral:
            self.Integral = self.Max_integral

        self.Derivative = self.Kd / self.Time_diff * (self.Error - self.Last_Error)
        self.Last_Error = self.Error

        self.Output = self.Proportional + self.Derivative + self.Integral
        return self.Output

    def reset(self):
        self.Integral = 0
        self.Last_Error = 0


class LinearFirstOrderController:

    def __init__(self, max_feedback=1, desired_change_of_feedback=1, softness=1):
        self.Max_feedback = max_feedback
        self.Desired_change_of_feedback = desired_change_of_feedback
        self.Softness = softness

    def _attenuation(self, x):
        return 1 / (1 + np.exp(-6 / self.Softness * (np.abs(x) - self.Softness)))

    def feedback(self, delta):
        feedback = - np.sign(delta) * np.minimum(np.sqrt(0.5 * self.Desired_change_of_feedback * np.abs(delta)) *
                                                 self._attenuation(delta),
                                                 self.Max_feedback)
        return feedback


class FlightController:

    def __init__(self, vessel, time):

        self.vessel = vessel

        self.max_roll = 45
        self.max_rollvel = 10
        self.max_rollacc = 5
        self.heading_to_roll_softness = 1
        self.roll_to_rollvel_softness = 0.1
        self.roll_kp = 1
        self.roll_ki = 1
        self.roll_kd = 1
        self.roll_pid = PIDLoop(self.roll_kp, self.roll_ki, self.roll_kd, time=time)
        self.heading_to_roll = LinearFirstOrderController(self.max_roll, self.max_rollvel, self.heading_to_roll_softness
                                                          )
        self.roll_to_rollvel = LinearFirstOrderController(self.max_rollvel, self.max_rollacc,
                                                          self.roll_to_rollvel_softness)

        self.max_climbrate = 10
        self.max_climbrate_change = 2
        self.max_pitchvel = 5
        self.max_pitchacc = 3
        self.altitude_to_climbrate_softness = 1
        self.climbrate_to_pitchvel_softness = 0.1
        self.pitch_kp = 1
        self.pitch_ki = 1
        self.pitch_kd = 1
        self.pitch_pid = PIDLoop(self.pitch_kp, self.pitch_ki, self.pitch_kd, time=time)
        self.altitude_to_climbrate = LinearFirstOrderController(self.max_climbrate, self.max_climbrate_change,
                                                                self.altitude_to_climbrate_softness)
        self.climbrate_to_pitchvel = LinearFirstOrderController(self.max_rollvel, self.max_pitchacc,
                                                                self.climbrate_to_pitchvel_softness)

        self.max_yawvel = 10
        self.max_yawacc = 5
        self.yaw_to_yawvel_softness = 0.1
        self.yaw_kp = 1
        self.yaw_ki = 1
        self.yaw_kd = 1
        self.yaw_pid = PIDLoop(self.yaw_kp, self.yaw_ki, self.yaw_kd, time=time)
        self.yaw_to_yawvel = LinearFirstOrderController(self.max_yawvel, self.max_yawacc, self.yaw_to_yawvel_softness)

        self.max_airspeed_acc = 10
        self.max_airspeed_acc_acc = 1
        self.speed_to_acc_softness = 0.1
        self.thrust_kp = 1
        self.thrust_ki = 1
        self.thrust_kd = 1
        self.thrust_pid = PIDLoop(self.thrust_kp, self.thrust_ki, self.thrust_kd, time=time)
        self.speed_to_acc = LinearFirstOrderController(self.max_airspeed_acc, self.max_airspeed_acc_acc,
                                                       self.speed_to_acc_softness)

    def update(self, delta_heading, delta_altitude, delta_yaw, delta_speed, time):
        roll = self.heading_to_roll.feedback(delta_heading)
        rollvel = self.roll_to_rollvel.feedback(roll)
        roll_control = self.roll_pid.update(rollvel, time)  # - vessel rollvel

        climbrate = self.altitude_to_climbrate.feedback(delta_altitude)
        pitchvel = self.climbrate_to_pitchvel.feedback(climbrate)
        pitch_control = self.pitch_pid.update(pitchvel, time)  # - vessel pitchvel

        yawvel = self.yaw_to_yawvel.feedback(delta_yaw)
        yaw_control = self.yaw_pid.update(yawvel, time)  # - vessel yaw vel

        acc = self.speed_to_acc.feedback(delta_speed)
        thrust_control = self.thrust_pid.update(acc, time)  # - vessel acc

        self.vessel.control.roll = roll_control
        self.vessel.control.pitch = pitch_control
        self.vessel.control.yaw = yaw_control
        self.vessel.control.thrust = thrust_control
