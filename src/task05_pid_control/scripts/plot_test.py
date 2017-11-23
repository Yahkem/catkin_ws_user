#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

x = np.array([1, 10, 30,90])
y = np.array([1, 10, 50, 999])
# plt.plot(x, y, linewidth=3.0)
# # plt.axis([0, 50, 80, 1000])
# plt.xlabel('X name')
# plt.ylabel('Y name')
# plt.show()

plt.figure(1)                # the first figure
plt.subplot(211)             # the first subplot in the first figure
plt.title('blabla') #after subplot
plt.plot([1, 2, 3], [4,5,6])
plt.subplot(212)             # the second subplot in the first figure
plt.title('222 nazev') #after subplot
plt.plot([1,10,100], [9000, 1, -1230])
plt.show()