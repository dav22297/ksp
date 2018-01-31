import numpy as np


def attenuation(softness, x):
    return 1 / (1 + np.exp(-6 / softness * (np.abs(x) - softness)))

print(attenuation(1, 1))
