from PID import PIDLoop
import time
import krpc
import numpy as np
conn = krpc.connect(name='Aeris 3A')

vessel = conn.space_center.active_vessel
ut = conn.add_stream(getattr, conn.space_center, 'ut')
print(vessel.angular_velocity(vessel.reference_frame))

desired_roll_acc = 1
desired_roll_speed = 1
desired_roll_angle = 1

max_roll_acc = 3
max_roll_speed = 3
max_roll_angle = 3


def angle_to_anglevel(delta_theta, max_theta_dotdot, max_theta_dot, softness=1.):
    target_vel = -np.sign(delta_theta) * np.minimum(np.sqrt(0.5 * max_theta_dotdot * np.abs(delta_theta)) *
                                                    1 / (1 + np.exp(-6 / softness * (np.abs(delta_theta) - softness))),
                                                    max_theta_dot)
    return target_vel


def speed_to_acc(delta_v, max_acc, max_acc_dot, softness=1.):
    target_acc = -np.sign(delta_v) * np.minimum(np.sqrt(0.5 * max_acc_dot * np.abs(delta_v)) *
                                                1 / (1 + np.exp(-6 / softness * (np.abs(delta_v) - softness))),
                                                max_acc)
    return target_acc


def heading_to_roll(delta_heading, max_roll, max_theta_dot, softness):
    target_roll = -np.sign(delta_heading) * np.minimum(np.sqrt(0.5 * max_theta_dot * np.abs(delta_heading)) *
                                                       1 / (1 + np.exp(-6 / softness * (np.abs(delta_heading) - softness))),
                                                       max_roll)
    return target_roll


def heading_difference(heading1, heading2):
    (heading2 - heading1) % 180


roll_pid = PIDLoop(kp=1, ki=1, kd=1)
thrust_pid = PIDLoop()
while True:
    print(vessel.angular_velocity(vessel.reference_frame))
    a = roll_pid.update(vessel.angular_velocity(vessel.surface_reference_frame)[1], ut())
    vessel.control.roll = a
    print(a)
    time.sleep(0.05)


print(angle_to_anglevel(vessel, 50, 1, 50, 5), ut())
