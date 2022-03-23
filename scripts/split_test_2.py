import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
#import matplo


def search_for_timestamp(dfj, time):
    time_col = dfj['%time'].to_numpy()
    time_col = np.abs(time_col - time)
    return np.argmin(time_col)


trial = 9
#these were determined manually.
start_idx = [0,     20000, 37500, 55500, 73500,  91000, 108000, 127000, 145000, 163000]
end_idx   = [17500, 35000, 53000, 71000, 88300, 106500, 125000, 142500, 160500, 178000]
for ii in range(len(start_idx)):
    start_idx[ii] = start_idx[ii] + 879
    end_idx[ii] = end_idx[ii] + 879


dfo = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1_t265.csv")
#plt.plot(df['field.pose.pose.position.x'][start_idx[trial]:end_idx[trial]], df['field.pose.pose.position.y'][start_idx[trial]:end_idx[trial]], color="red")
#plt.show()

dfj = pd.read_csv("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1_joints.csv")

for ii in range(len(start_idx)):
    out_df = dfo[["field.pose.pose.position.x", "field.pose.pose.position.y", "field.pose.pose.position.z", "field.pose.pose.orientation.x", "field.pose.pose.orientation.y", "field.pose.pose.orientation.z", "field.pose.pose.orientation.w"]][start_idx[ii]:end_idx[ii]]
    out_df.to_csv("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/" + str(ii) +"_odom_t265.csv")
    
    start_time = dfo["%time"][start_idx[ii]]
    joint_start_idx = search_for_timestamp(dfj, start_time)
    
    end_time = dfo["%time"][end_idx[ii]]
    joint_end_idx = search_for_timestamp(dfj, end_time)
    
    out_df = dfj[["field.velocity0", "field.velocity1", "field.velocity2", "field.velocity3"]][joint_start_idx:joint_end_idx]
    out_df.to_csv("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/" + str(ii) +"_joint_states.csv")
    




#dfj = df[["field.velocity0", "field.velocity1", "field.velocity2", "field.velocity3"]]
#out_df.to_csv("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1_joints_50hz.csv");




