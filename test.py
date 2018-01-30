import numpy as np


def heading_difference(heading1, heading2):
    raw = (heading2 - heading1)
    if raw > 180:
        difference = -360 + raw
    elif raw < -180:
        difference = 360 + raw
    else:
        difference = raw
    return difference

print(heading_difference(340, 0))
