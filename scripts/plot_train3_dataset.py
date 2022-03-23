import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/Downloads/Train3/localization_ground_truth/0007_Tr_grass_GT.txt", header=None)
plt.plot(df[1][0:200], df[2][0:200], color='r')

df = pd.read_csv("/home/justin/xout_file.csv")
plt.plot(df['x'], df['y'], color='b')

df = pd.read_csv("/home/justin/xout_file2.csv")
plt.plot(df['x'], df['y'], color='b')

plt.xlabel('m')
plt.ylabel('m')
plt.legend(['Odometry', 'Simulation'])
plt.title('Simulation vs Dataset Odometry')
plt.show()




#df = pd.read_csv("/home/justin/Downloads/CV3/extracted_data/odometry/0050_odom_data.txt", header=None)
#plt.plot()
#plt.show()
