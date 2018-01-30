from PID import PIDLoop, LinearFirstOrderController
import time
#import krpc
import numpy as np
#conn = krpc.connect(name='Aeris 3A')

#vessel = conn.space_center.active_vessel
#ut = conn.add_stream(getattr, conn.space_center, 'ut')
#print(vessel.angular_velocity(vessel.reference_frame))

desired_roll_acc = 1
desired_roll_speed = 1
desired_roll_angle = 1

max_roll_acc = 3
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


# roll control
roll_pid = PIDLoop(kp=1, ki=1, kd=1)
heading_to_roll = LinearFirstOrderController()
roll_to_rollvel = LinearFirstOrderController

# thrust control
thrust_pid = PIDLoop()
speed_to_acc = LinearFirstOrderController()

# pitch control
pitch_pid = PIDLoop()
altitude_to_climbrate = LinearFirstOrderController()
climbrate_to_pitchvel = LinearFirstOrderController()

# yaw control
yaw_pid = PIDLoop()
yaw_to_yawvel = LinearFirstOrderController()


while True:
    print(vessel.angular_velocity(vessel.reference_frame))
    a = roll_pid.update(vessel.angular_velocity(vessel.surface_reference_frame)[1], ut())
    vessel.control.roll = a
    print(a)
    time.sleep(0.05)
