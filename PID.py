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


