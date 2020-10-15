#!/usr/bin/env python

import matplotlib.pyplot as plt
import pandas as pd

# read
header_list = ["t", "x", "y", "z", "p", "q", "r"]
log = pd.read_csv('/tmp/DirectKinematicsLog.csv', names = header_list)

# plot
log.plot("t", ["x", "z", "q"], subplots=True)
plt.grid()
plt.show()
