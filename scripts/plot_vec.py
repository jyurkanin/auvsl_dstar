import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np


fig = plt.figure()
ax = Axes3D(fig)

xmin = -10
xmax = 10
ymin = -10
ymax = 10

#df = pd.read_csv("/home/justin/elev.csv")
#df = df[df['x'] < xmax]
#df = df[df['x'] > xmin]
#df = df[df['y'] < ymax]
#df = df[df['y'] > ymin]
skip = 1
#ax.scatter(df['x'][::skip], df['y'][::skip], df['alt'][::skip])

df = pd.read_csv("/home/justin/features.csv")
#ax.plot(df['x1'][::skip], df['y1'][::skip], df['z1'][::skip], color='b')
#ax.plot(df['x2'][::skip], df['y2'][::skip], df['z2'][::skip], color='r')
#ax.plot(df['x3'][::skip], df['y3'][::skip], df['z3'][::skip], color='g')
#ax.plot(df['x4'][::skip], df['y4'][::skip], df['z4'][::skip], color='y')

#plt.plot(df['x1'][::skip], df['y1'][::skip], color='b')
#plt.plot(df['x2'][::skip], df['y2'][::skip], color='r')
#plt.plot(df['x3'][::skip], df['y3'][::skip], color='g')
#plt.plot(df['x4'][::skip], df['y4'][::skip], color='y')


#df = pd.read_csv("/home/justin/features.csv")
#df = df[df['x'] < xmax]
#df = df[df['x'] > xmin]
#df = df[df['y'] < ymax]
#df = df[df['y'] > ymin]

skip = 100

ax.quiver(df['x'][::skip], df['y'][::skip], df['z'][::skip],   df['zx'][::skip],df['zy'][::skip],df['zz'][::skip],  color='g', arrow_length_ratio=.01, length=.01, alpha=.5)
ax.quiver(df['x'][::skip], df['y'][::skip], df['z'][::skip],   df['vx'][::skip],df['vy'][::skip],df['vz'][::skip],  color='r', arrow_length_ratio=.01, length=.01, alpha=.5)
ax.set_xlabel('$X$')
ax.set_ylabel('$Y$')
ax.set_zlabel('$Z$')

#ax.set_zlim(0,10)
#plt.show()

plt.show()
