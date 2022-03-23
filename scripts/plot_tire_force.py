import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np


#fig = plt.figure()
#ax = Axes3D(fig)
fig, ax = plt.subplots()

skip = 1
df = pd.read_csv("/home/justin/features.csv")
#ax.plot(df['x'][::skip], df['y'][::skip], color='b')
skip = 500
scalar = .01
ax.quiver(df['x'][::skip], df['y'][::skip], df['fx'][::skip]*scalar, df['fy'][::skip]*scalar,  scale=1, color='r')
plt.title("Force vector of front right tire")

plt.show()
