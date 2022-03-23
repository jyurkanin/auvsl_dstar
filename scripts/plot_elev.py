import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np


fig = plt.figure()
ax = Axes3D(fig)


df = pd.read_csv("/home/justin/elev.csv")
df = df[df['x'] < 40]
df = df[df['x'] > -40]
df = df[df['y'] < 40]
df = df[df['y'] > -40]
skip = 1
ax.scatter(df['x'][::skip], df['y'][::skip], df['alt'][::skip])
#plt.show()

idx = 1
stop_idx = -1
df = pd.read_csv("/home/justin/xout_file.csv")
df = df[df['x'] < 200]
df = df[df['x'] > -200]
df = df[df['y'] < 200]
df = df[df['y'] > -200]
skip = 100
ax.scatter(df['x'][::skip], df['y'][::skip], df['z'][::skip], c='r')
plt.show()

#df = pd.read_csv("/home/justin/alt_file.csv")
#plt.plot(df['r'], df['alt'])
#plt.show()

#df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul2/rantoul_long_odom.csv")
#ax.scatter(df['field.pose.pose.position.x'], df['field.pose.pose.position.y'], df['field.pose.pose.position.z'])
#plt.plot(df['field.pose.pose.position.z'])
#plt.show()
