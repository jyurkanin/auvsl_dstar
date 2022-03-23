import pandas as pd
import numpy as np
#import matplo


start_idx = 1


df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1_joints.csv")
df = df[["field.velocity0", "field.velocity1", "field.velocity2", "field.velocity3"]]



idx = 0
while True:
    vel_sum = abs(df["field.velocity0"][idx]) + abs(df["field.velocity1"][idx]) + abs(df["field.velocity2"][idx]) + abs(df["field.velocity3"][idx])
    if(vel_sum > .01):
        break
    idx += 1
print("first non zero idx", idx)

start_idx = idx

out_df = df.iloc[start_idx::1]
out_df.to_csv("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1_joints_50hz.csv");



df = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1_t265.csv")
df = df[["field.pose.pose.position.x", "field.pose.pose.position.y", "field.pose.pose.position.z", "field.pose.pose.orientation.x", "field.pose.pose.orientation.y", "field.pose.pose.orientation.z", "field.pose.pose.orientation.w"]]
out_df = df.iloc[start_idx::1]
out_df.to_csv("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1_t265_50hz.csv")

#import pdb
#pdb.set_trace()


