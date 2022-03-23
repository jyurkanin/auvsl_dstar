import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D
from sklearn.preprocessing import MinMaxScaler


#fig = plt.figure()
#ax = Axes3D(fig)

#skip = 10
#idx = 1
#stop_idx = 2969



df = pd.read_csv("/home/justin/features.csv")
#plt.plot(df['x'], df['y'])
plt.plot(df['dvy'])
#plt.plot(df['vy'])
plt.show()



#df = pd.read_csv("/home/justin/occ_grid.csv")
#occ = df['occ']
#cutoff = 0
#plt.scatter(df['x'][occ[:] > cutoff], df['y'][occ[:] > cutoff])# .01*occ[occ[:] > cutoff])
#plt.show()
#df = pd.read_csv("/home/justin/xout_file.csv")
#plt.scatter(df['x'], df['y'])

#plt.show()


#df = pd.read_csv("/home/justin/code/AUVSL_ROS/pose.csv")
#plt.scatter(df['x'], df['y'])
#plt.show()
