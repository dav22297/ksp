import numpy as np


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

test = Derivative()
print(test.last_derivative, test.last_time, test.last_value)
for i in range(10):
    print(test.derive(-i**2, i/10))
    print((test.last_derivative, test.last_time, test.last_value))
