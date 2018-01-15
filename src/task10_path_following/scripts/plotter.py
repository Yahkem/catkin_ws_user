#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

data1 = np.loadtxt("lane.txt", delimiter=",")

xs1, ys1 = zip(*data1)
plt.plot(xs1, ys1, "r.", markersize=12)

#
# ax.set_xlim((0, 10.0))
# ax.set_ylim((0, 8.0))
plt.axes().set_aspect("equal", "datalim")
plt.show()
