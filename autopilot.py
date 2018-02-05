from PID import PIDLoop, FlightController, Derivative
import time
import krpc
import numpy as np
conn = krpc.connect(name='Aeris 3A')

vessel = conn.space_center.active_vessel
ut = conn.add_stream(getattr, conn.space_center, 'ut')
print(ut())




def heading_difference(heading1, heading2):
    raw = heading2 - heading1
    if raw > 180:
        difference = -360 + raw
    elif raw < -180:
        difference = 360 + raw
    else:
        difference = raw
    return difference

fc = FlightController(vessel, conn, ut())
while True:
    fc.update_state(ut())
    fc.attitude_control(10,0,0,180, ut())
    fc.tune_pid()
