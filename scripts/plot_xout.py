import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np


#fig = plt.figure()
#ax = Axes3D(fig)

#df = pd.read_csv("/home/justin/elev.csv")
#df = df[df['x'] < 10]
#df = df[df['x'] > -10]
#df = df[df['y'] < 10]
#df = df[df['y'] > -10]
#skip = 100
#ax.scatter(df['x'][::skip], df['y'][::skip], df['alt'][::skip])
#plt.show()

#idx = 1
#stop_idx = -1
#df = pd.read_csv("/home/justin/xout_file.csv")
#df = df[df['x'] < 10]
#df = df[df['y'] > -10]
#ax.scatter(df['x'][::100], df['y'][::100], df['z'][::100], c='r')
#plt.show()

#plt.plot(df['x'], .1*df['x'])
#plt.plot(df['x'], df['z'])
#plt.show()

#df = pd.read_csv("/home/justin/alt_file.csv")
#plt.plot(df['r'], df['alt'])
#plt.show()

#df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul2/rantoul_circles_odom.csv")
#ax.scatter(df['field.pose.pose.position.x'], df['field.pose.pose.position.y'], df['field.pose.pose.position.z'])
#plt.plot(df['field.pose.pose.position.z'])
#plt.show()




df = pd.read_csv("/home/justin/xout_file.csv")
sim_len = df.shape[0]
sim_x = df['x'][sim_len-1]
sim_y = df['y'][sim_len-1]
plt.plot(df['x'], df['y'], color='blue')#,  alpha=.1, s = 1)


#df = pd.read_csv("/home/justin/kinematic_long.csv")
#df = df[::20]
#plt.plot(df['x'], df['y'])

df = pd.read_csv("/home/justin/Downloads/CV3/localization_ground_truth/0001_CV_grass_GT.txt", header=None)
#df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/3_odom_t265.csv")
#plt.plot(df['field.pose.pose.position.x'], df['field.pose.pose.position.y'], color="red")
odom_len = df.shape[0]

dx = df['field.pose.pose.position.x'][odom_len-1] - sim_x
dy = df['field.pose.pose.position.y'][odom_len-1] - sim_y
print("Final Error ", np.sqrt((dx*dx)+(dy*dy)))

dx = df['field.pose.pose.position.x'][odom_len-1]
dy = df['field.pose.pose.position.y'][odom_len-1]
print("Distance Start to End ", np.sqrt((dx*dx)+(dy*dy)))

plt.xlabel("m");
plt.ylabel("m");
plt.title("Comparison of Dynamic Model, Kinematic Model, and Odometry Trajectories at 2m/s")
plt.legend(["Dynamic Model","Odometry"])
#plt.text(30,-30,"4.88m error after 120.5m path", fontsize=12)
plt.show()

#df = pd.read_csv("/home/justin/code/AUVSL_ROS/temp.csv")
#plt.plot(df['field.velocity0'])
#plt.plot(df['field.velocity3'])
#plt.show()
