import fileinput
import re

import numpy as np
import matplotlib.pyplot as plt

import math
import cmath

x = []
y = []

for line in fileinput.input():
  m = re.match('\((-?[0-9]+),(-?[0-9]+)\) (on|off)', line)
  if (m):
    print(m.group(1), m.group(2), m.group(3))
    if m.group(3) == "on":
      x.append(int(m.group(1)))
      y.append(int(m.group(2)))

plt.plot(x, y)
plt.show()

if len(x) == 0:
  exit

x0 = x[0]
y0 = y[0]

first_line = True
dtheta = 0
for i in range(1, len(x)):
  x1 = x[i]
  y1 = y[i]

  theta = cmath.phase(complex(x1 - x0, y1 - y0)) * (180.0 / math.pi)
  if theta < 0:
    theta = theta + 360

  if not first_line:
    dtheta = theta - last_theta
    if dtheta < -180:
      dtheta = dtheta + 360
    if dtheta > 180:
      dtheta = dtheta - 360
  else:
    first_line = False

  print(theta, dtheta)

  last_theta = theta

  x0 = x1
  y0 = y1

