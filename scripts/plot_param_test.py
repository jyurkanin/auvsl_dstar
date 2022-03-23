import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/param_graph.csv")
plt.plot(df['param'], df['lin_err'], color='r')

plt.show()
