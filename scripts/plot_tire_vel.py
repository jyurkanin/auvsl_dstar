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
skip = 2000
ax.quiver(df['x'][::skip], df['y'][::skip], df['vx_t'][::skip], df['vy_t'][::skip],  scale=4, color='r')
ax.quiver(df['x'][::skip], df['y'][::skip], df['vx'][::skip], df['vy'][::skip],  scale=4, color='g', alpha=.5)
plt.title("Velocity vector of vehicle (green)\nVelocity vector of right front tire cpt frame (red)")

plt.show()
