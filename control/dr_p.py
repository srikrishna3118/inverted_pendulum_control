from random import random
from math import sqrt
from math import pi, cos, sin

import matplotlib.pyplot as plt
import csv
import numpy as np 
x1 = []
x5 = []
x10 =[]

with open('log.0') as file:
    d = file.readlines()
    for row in d:
        x1.append(np.array(row.strip()).astype(np.float))

with open('log.3') as file:
    d = [next(file) for x in range(11000)]#file.readlines()
    for row in d:
        x5.append(np.array(row).astype(np.float))

with open('log.5') as file:
    d = [next(file) for x in range(11000)]#file.readlines()
    for row in d:
        x10.append(np.array(row).astype(np.float)/2.0)


data_to_plot = [x1,x5,x10]

fig = plt.figure()
#fig, axs = plt.subplots(3,sharex=True)
plt.violinplot(data_to_plot)
plt.ylabel('PID error for the Joint')
#axs[0].plot(x1,'g',label='Master')
#axs[0].legend(loc='upper right')
#axs[1].plot(x5,'r',label='slave')
#axs[2].plot(x10,'b',label='Master')
plt.xticks([1,2,3],['zero nodes', '5 nodes','10 nodes'])
plt.show()
