import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D
from sklearn.preprocessing import MinMaxScaler


fig = plt.figure()
ax = Axes3D(fig)

skip = 10
idx = 1
stop_idx = 2969


df = pd.read_csv("/home/justin/occ_grid.csv")

threshold = 4

#df = df[df['y'] < 5]
#df = df[df['x'] > -5]
occ = df['occ']
ax.scatter(df['x'][occ > threshold], df['y'][occ > threshold], df['occ'][occ > threshold], alpha=.2)

df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_odom_2.csv")
ax.plot(df['field.pose.pose.position.x'], df['field.pose.pose.position.y'], df['field.pose.pose.position.z'])

plt.show()

