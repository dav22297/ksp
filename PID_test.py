from PID import PIDLoop
import time
import numpy as np
import matplotlib.pyplot as plt


test = PIDLoop(kp=12000, ki=11000, kd=0.005, set_point=0, min_output=-1, max_output=1)
test.set_time(time.time())
data = []
sum = 0
last_time = time.time()
setpoint = []

def func(x):
    if x > 10000:
        return 1
    else:
        return 0
for i in range(100000):
    if i % 200 == 0:
        test.set_point(func(i))
        test.update(sum, time.time())
    setpoint.append(func(i))

    sum += test.Output * (time.time() - last_time) - sum*0.00001
    last_time = time.time()
    data.append(sum)

plt.plot(range(100000), setpoint)
plt.plot(range(100000), data)
#plt.xlim((8000, 13000))
plt.show()
