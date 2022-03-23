import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D



fig = plt.figure()
ax = Axes3D(fig)

df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_odom_2.csv")
ax.scatter(df['field.pose.pose.position.x'], df['field.pose.pose.position.y'], df['field.pose.pose.position.z'])
plt.show()

#df = pd.read_csv("/home/justin/occ_grid.csv")
#occ = df['occupancy']
#cutoff = 8
#plt.scatter(df['x'][occ[:] > cutoff], df['y'][occ[:] > cutoff])# .01*occ[occ[:] > cutoff])
#plt.show()
#df = pd.read_csv("/home/justin/xout_file.csv")
#plt.scatter(df['x'], df['y'])

#plt.show()


#df = pd.read_csv("/home/justin/code/AUVSL_ROS/pose.csv")
#plt.scatter(df['x'], df['y'])
#plt.show()
