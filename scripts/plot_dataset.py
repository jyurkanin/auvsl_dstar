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

df = pd.read_csv("/home/justin/Downloads/LD3/localization_ground_truth/0001_LD_grass_GT.txt", header=None)
plt.plot(df[1], df[2], color='r')

df = pd.read_csv("/home/justin/xout_file.csv")
plt.scatter(df['x'], df['y'], color='b', s=1)

plt.xlabel('m')
plt.ylabel('m')
plt.legend(['Odometry', 'Simulation'])
plt.title('Rantoul Long Test Simulation vs Odometry')
plt.show()