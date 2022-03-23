import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/Downloads/LD3/localization_ground_truth/0001_LD_grass_GT.txt", header=None)
plt.plot(df[1], df[2], color='r')

#df = pd.read_csv("/home/justin/xout_file.csv")
#plt.plot(df['x'], df['y'], color='b')

plt.xlabel('m')
plt.ylabel('m')
plt.legend(['Odometry', 'Simulation'])
plt.title('Simulation vs Dataset Odometry')
plt.show()

# df = pd.read_csv("/home/justin/Downloads/LD3/extracted_data/odometry/0001_odom_data.txt", header=None)
# plt.plot(df[1], color='r')
# plt.plot(df[3], color='b')
# plt.show()

#df = pd.read_csv("/home/justin/xout_file.csv")
#plt.plot(df['x'], color='b')
#plt.show()
