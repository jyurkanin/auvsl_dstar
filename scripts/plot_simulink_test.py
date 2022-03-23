import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("../src/test/simulink_p_test.csv")
plt.plot(df['x'], df['y'], color='r')

df = pd.read_csv("/home/justin/xout_file.csv")
plt.plot(df['x'], df['y'], color='b')

dfe = pd.read_csv("/home/justin/xout_file_ref.csv")
plt.plot(dfe['x'], dfe['y'], color='g')

plt.xlabel('m')
plt.ylabel('m')
plt.legend(['Matlab', 'C++ rk4 qd=.5qw', 'C++ qn=axis-angle rotation'])
plt.title('Comparison of Quaternion Integration Methods \nEffect on simulated trajectory')
plt.show()

plt.plot(np.fabs(dfe['x'] - df['x']) + np.fabs(dfe['y'] - df['y']), color='r')
plt.show()
