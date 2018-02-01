from PID import PIDLoop, LinearFirstOrderController, FlightController, Derivative
import time
import krpc
import numpy as np
conn = krpc.connect(name='Aeris 3A')

vessel = conn.space_center.active_vessel
ut = conn.add_stream(getattr, conn.space_center, 'ut')
print(vessel.angular_velocity(vessel.reference_frame))



desired_roll_speed = 1
desired_roll_angle = 1


max_roll_speed = 3
max_roll_angle = 3


def heading_difference(heading1, heading2):
    raw = heading2 - heading1
    if raw > 180:
        difference = -360 + raw
    elif raw < -180:
        difference = 360 + raw
    else:
        difference = raw
    return difference




while True:
    angular_momentum = vessel.angular_velocity(vessel.surface_reference_frame)
    angular_momentum = conn.space_center.transform_direction(angular_momentum, vessel.surface_reference_frame, vessel.reference_frame)
    pitch_ang_vel = np.dot((1, 0, 0), angular_momentum)
    roll_ang_vel = np.dot((0, 1, 0), angular_momentum)
    yaw_ang_vel = np.dot((0, 0, 1), angular_momentum)
    print(pitch_ang_vel)
    conn.drawing.add_direction(angular_momentum, vessel.reference_frame)
    time.sleep(0.05)
