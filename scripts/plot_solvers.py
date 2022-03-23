import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D



#fig = plt.figure()
#ax = Axes3D(fig)

#df = pd.read_csv("/home/justin/elev.csv")
#ax.scatter(df['x'][::10], df['y'][::10], df['alt'][::10])

#idx = 1000
#stop_idx = -1
#df = pd.read_csv("/home/justin/xout_file.csv")
#ax.scatter(df['x'][idx:stop_idx], df['y'][idx:stop_idx], df['z'][idx:stop_idx])

#df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/odom_10hz.csv")
#ax.scatter(df['field.pose.pose.position.x'][idx::stop_idx], df['field.pose.pose.position.y'][idx::stop_idx], df['field.pose.pose.position.z'][idx::stop_idx])

idx = 1
stop_idx = 200

df = pd.read_csv("/home/justin/euler_1000_xout.csv")
print(df.shape)
df = df[::100]
plt.plot(df['x'][idx:stop_idx], df['y'][idx:stop_idx])

df = pd.read_csv("/home/justin/euler_10000_xout.csv")
df = df[::1000]
plt.plot(df['x'][idx:stop_idx], df['y'][idx:stop_idx])



df = pd.read_csv("/home/justin/rk4_1000_xout.csv")
print(df.shape)
df = df[::100]
plt.plot(df['x'][idx:stop_idx], df['y'][idx:stop_idx])

df = pd.read_csv("/home/justin/rk4_10000_xout.csv")
print(df.shape)
df = df[::1000]
plt.plot(df['x'][idx:stop_idx], df['y'][idx:stop_idx])


df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/odom_10hz.csv")
print(df.shape)
plt.plot(df['field.pose.pose.position.x'][idx:stop_idx], df['field.pose.pose.position.y'][idx:stop_idx])

plt.title("Hallway Trajectory vs Simulated Trajectory")
plt.legend(['euler .001s','euler .0001s','rk4 .001s','rk4 .0001s','Trajectory']);
plt.show()
