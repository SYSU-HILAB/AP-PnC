#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np

## generate points from lemniscate of bernoulli
t = np.linspace(0, 2*np.pi, 1000)
x = np.cos(t) / (1 + np.sin(t)**2)
y = np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)


cur = 0.5;
xx = np.cos(t) / (1+cur * np.sin(t)**2)
yy = np.sin(t) * np.cos(t) / (1+cur*np.sin(t)**2) 

cur = 1.0;
xx1 = np.cos(t) / (1+cur * np.sin(t)**2)
yy1 = np.sin(t) * np.cos(t) / (1+cur*np.sin(t)**2) 

cur = 3.0;
xx2 = np.cos(t) / (1+cur * np.sin(t)**2)
yy2 = np.sin(t) * np.cos(t) / (1+cur*np.sin(t)**2) 

##  plot these trajectories in single figure
fig, ax = plt.subplots()
ax.plot(x, y, label='Lemniscate of Bernoulli')
ax.plot(xx, yy, label='Type1')
ax.plot(xx1, yy1, label='Type2')
ax.plot(xx2, yy2, label='Type3')
ax.legend()
plt.show()