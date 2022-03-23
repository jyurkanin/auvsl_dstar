import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D
from sklearn.preprocessing import MinMaxScaler, StandardScaler


xmax = 200
xmin = -200
ymax = 200
ymin = -200

df = pd.read_csv("/home/justin/elev.csv")
df = df[df['x'] < xmax]
df = df[df['x'] > xmin]
df = df[df['y'] < ymax]
df = df[df['y'] > ymin]

skip = 1

x = df['x'][::skip].to_numpy()
y = df['y'][::skip].to_numpy()
z = df['alt'][::skip].to_numpy()


scaler = MinMaxScaler()

z_scaled = scaler.fit_transform(z.reshape(-1,1))

plt.scatter(x, y, s=10, c=z_scaled[:,0], cmap='plasma');
plt.colorbar()

df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul2/rantoul_long_odom.csv")
df = df[df['field.pose.pose.position.x'] < xmax]
df = df[df['field.pose.pose.position.x'] > xmin]
df = df[df['field.pose.pose.position.y'] < ymax]
df = df[df['field.pose.pose.position.y'] > ymin]
plt.plot(df['field.pose.pose.position.x'], df['field.pose.pose.position.y'], color='r')


df = pd.read_csv("/home/justin/xout_file.csv")
df = df[df['x'] < xmax]
df = df[df['x'] > xmin]
df = df[df['y'] < ymax]
df = df[df['y'] > ymin]
df = df[::skip]
plt.scatter(df['x'], df['y'], color='b', s=1)

plt.xlabel('m')
plt.ylabel('m')
plt.legend(['Odometry', 'Simulation'])
plt.title('Rantoul Long Test Simulation vs Odometry')
plt.show()
