import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np


#fig = plt.figure()
#ax = Axes3D(fig)


df = pd.read_csv("/home/justin/features.csv")
df = df[40000:]
skip = 1
plt.plot(df['z1'])
plt.plot(df['z2'])
plt.plot(df['z3'])
plt.plot(df['z4'])
plt.legend(['z1','z2','z3','z4'])
plt.show()
