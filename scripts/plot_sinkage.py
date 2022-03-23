import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
from mpl_toolkits.mplot3d import axes3d, Axes3D




df = pd.read_csv("/home/justin/sinkages.csv")

plt.plot(df['zr1'])
plt.plot(df['zr2'])
plt.plot(df['zr3'])
plt.plot(df['zr4'])
plt.show()



#df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_odom_2.csv")
#plt.plot(df['field.twist.twist.linear.x'])
#plt.plot(df['field.twist.twist.linear.y'])
#plt.show()
