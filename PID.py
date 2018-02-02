import numpy as np
__name__ = "PIDLoop"


def attenuation(x, softness):
    return 1 / (1 + np.exp(-6 / softness * (np.abs(x) - softness)))


def linear_acceleration_model(x, max_velocity, acceleration, softness):
    target_velocity = np.sign(x) * np.minimum(np.sqrt(0.5 * acceleration * np.abs(x)) *
                                              attenuation(x, softness),
                                              max_velocity)
    return target_velocity


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
        # check if timed passed on
        if self.Time_diff != 0:
            self.Derivative = self.Kd / self.Time_diff * (self.Error - self.Last_Error)
            self.Last_Error = self.Error

        self.Output = self.Proportional + self.Derivative + self.Integral
        return self.Output

    def reset(self):
        self.Integral = 0
        self.Last_Error = 0


class FlightController:

    def __init__(self, vessel, conn, time):

        self.vessel = vessel
        self.conn = conn

        self.max_roll = 45
        self.max_rollvel = 10
        self.max_rollacc = 5
        self.roll = 0
        self.roll_vel = 0
        self.roll_acc = Derivative()
        self.heading_to_roll_softness = 1
        self.roll_to_rollvel_softness = 0.1
        self.roll_kp = 0.003
        self.roll_ki = 0
        self.roll_kd = 0.0
        self.roll_pid = PIDLoop(self.roll_kp, self.roll_ki, self.roll_kd, time=time)

        self.max_climbrate = 10
        self.max_climbrate_change = 2
        self.max_pitchvel = 5
        self.max_pitchacc = 3
        self.pitch = 0
        self.pitch_vel = 0
        self.pitch_acc = Derivative()
        self.altitude_to_climbrate_softness = 1
        self.climbrate_to_pitchvel_softness = 0.1
        self.pitch_kp = 0.03
        self.pitch_ki = 0
        self.pitch_kd = 0.0
        self.pitch_pid = PIDLoop(self.pitch_kp, self.pitch_ki, self.pitch_kd, time=time)

        self.max_yawvel = 10
        self.max_yawacc = 5
        self.yaw_to_yawvel_softness = 0.1
        self.heading = 0
        self.yaw_vel = 0
        self.yaw_acc = Derivative()
        self.yaw_kp = 0.
        self.yaw_ki = 0
        self.yaw_kd = 0.0
        self.yaw_pid = PIDLoop(self.yaw_kp, self.yaw_ki, self.yaw_kd, time=time)

        self.max_airspeed_acc = 10
        self.max_airspeed_acc_acc = 1
        self.speed_to_acc_softness = 0.1
        self.airspeed = 0
        self.acceleration = Derivative()
        self.thrust_kp = 1
        self.thrust_ki = 0
        self.thrust_kd = 0.0
        self.thrust_pid = PIDLoop(self.thrust_kp, self.thrust_ki, self.thrust_kd, time=time, min_output=0)

    def update_state(self, time):

        # updating navball directions
        self.pitch = self.vessel.flight().pitch
        self.roll = self.vessel.flight().roll
        self.heading = self.vessel.flight().heading

        # updating navball angle velocities
        angular_momentum = self.vessel.angular_velocity(self.vessel.surface_reference_frame)
        angular_momentum = self.conn.space_center.transform_direction(angular_momentum, self.vessel.surface_reference_frame,
                                                                 self.vessel.reference_frame)
        self.pitch_vel = np.dot((1, 0, 0), angular_momentum)
        self.roll_vel = np.dot((0, 1, 0), angular_momentum)
        self.yaw_vel = np.dot((0, 0, 1), angular_momentum)

        # updating navball angle accelerations
        self.pitch_acc.derive(self.pitch_vel, time)
        self.roll_acc.derive(self.roll_vel, time)
        self.yaw_acc.derive(self.yaw_vel, time)

        # update airspeed and acceleration
        self.airspeed = self.vessel.flight().true_air_speed
        self.acceleration.derive(self.airspeed, time)

    def raw_control(self, pitch_acc, roll_acc, yaw_acc, airspeed_change, time):

        self.pitch_pid.Set_point = pitch_acc
        pitch_control = self.pitch_pid.update(self.pitch_acc.last_derivative, time)
        self.roll_pid.Set_point = roll_acc
        roll_control = self.roll_pid.update(self.roll_acc.last_derivative, time)
        self.yaw_pid.Set_point = yaw_acc
        yaw_control = self.yaw_pid.update(self.yaw_acc.last_derivative, time)
        self.thrust_pid.Set_point = airspeed_change
        thrust_control = self.thrust_pid.update(self.acceleration.last_derivative, time)

        self.vessel.control.roll = roll_control
        self.vessel.control.pitch = pitch_control
        self.vessel.control.yaw = yaw_control
        self.vessel.control.throttle = thrust_control

    def rate_control(self, pitch_vel, roll_vel, yaw_vel, airspeed, time):

        pitch_vel_diff = pitch_vel - self.pitch_vel
        pitch_acc = np.sign(pitch_vel_diff) * self.max_pitchacc * attenuation(pitch_vel_diff, 5)
        roll_vel_diff = roll_vel - self.roll_vel
        roll_acc = np.sign(roll_vel_diff) * self.max_rollacc * attenuation(roll_vel_diff, 5)
        yaw_vel_diff = yaw_vel - self.yaw_vel
        yaw_acc = np.sign(yaw_vel_diff) * self.max_yawacc * attenuation(yaw_vel_diff, 5)
        acc = linear_acceleration_model(airspeed - self.airspeed, self.max_airspeed_acc, self.max_airspeed_acc_acc, 5)

        self.raw_control(pitch_acc, roll_acc, yaw_acc, acc, time)

    def attitude_control(self, pitch, roll, yaw, airspeed, time):

        pitch_vel = linear_acceleration_model(pitch - self.pitch, self.max_pitchvel, self.max_pitchacc, 1)
        roll_vel = linear_acceleration_model(roll - self.roll, self.max_rollvel, self.max_rollacc, 1)
        yaw_vel = linear_acceleration_model(yaw - self.heading, self.max_yawvel, self.max_yawacc, 1)

        self.rate_control(pitch_vel, roll_vel, yaw_vel, airspeed, time)

    def tune_pid(self, max_engage_roll=20, max_engage_pitch=20, max_engage_yaw=20):

        inertia = np.array(self.vessel.moment_of_inertia(self.vessel.reference_frame))
        torque = np.array(self.vessel.available_torque(self.vessel.reference_frame))
        torque = np.mean(torque, axis=0)
        acceleration = np.degrees(torque / inertia)
        self.vessel.flight().drag, self.vessel.available_thrust() self.vessel.mass()
        self.roll_pid.Kp = 1 / (max_engage_roll * acceleration[0])
        self.pitch_pid.Kp = 1 / (max_engage_pitch * acceleration[1])
        self.yaw_pid.Kp = 1 / (max_engage_yaw * acceleration[2])

class Derivative:

    def __init__(self, last_time=-1):
        self.last_time = last_time
        self.last_value = 0
        self.last_derivative = 0

    def derive(self, value, time):
        if time != self.last_time:
            derivative = (value - self.last_value) / (time - self.last_time)
            self.last_value = value
            self.last_time = time
            self.last_derivative = derivative
        else:
            derivative = self.last_derivative
        return derivative
