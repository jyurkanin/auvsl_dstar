import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D


skippers = [32,33,34,35,36,37,38, 78,79,80,81]

#for ii in skippers:
df = pd.read_csv("/home/justin/Downloads/CV3/localization_ground_truth/0081_CV_grass_GT.txt", header=None)
end_time = df[0][0] + 6
plt.plot(df[1][df[0] < end_time], df[2][df[0] < end_time])

df = pd.read_csv("/home/justin/xout_file.csv")
plt.plot(df['x'], df['y'], color='b')

#df = pd.read_csv("/home/justin/nn_xout.csv")
#plt.plot(df['x'], df['y'], color='b')

plt.xlabel('m')
plt.ylabel('m')
plt.legend(['Odometry', 'Simulation'])
plt.title('Simulation vs Dataset Odometry')
plt.show()

df = pd.read_csv("/home/justin/Downloads/CV3/extracted_data/odometry/0081_odom_data.txt", header=None)
plt.plot(df[1], color='r')
plt.plot(df[3], color='b')
plt.show()




#df = pd.read_csv("/home/justin/xout_file.csv")
#plt.plot(df['x'], color='b')
#plt.show()
