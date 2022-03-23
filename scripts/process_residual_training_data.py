#Goal of this script is to process x,y,rad data from Train3 dataset
#and output approx vx,vy,rads and left/right velocities.
#problem is, x,y,rad data is sampled at a different rate from left/right velocity
#Need to compute vx,vy in robot body frame and also interpolate so that all input data is at the same rate.
#left/right hz is 20hz
#x,y,rads   hz is 30hz
#convert this to body frame velocity, then resmaple to 20hz

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from scipy import interpolate




def main():
    process_training_data()
    process_cv3_data()
    process_ld3_data()

#process cv3 data into a form usable by a VehicleNet
def process_cv3_data():
    odom_filename = "/home/justin/Downloads/CV3/extracted_data/odometry/{:04d}_odom_data.txt"
    gt_filename = "/home/justin/Downloads/CV3/localization_ground_truth/{:04d}_CV_grass_GT.txt"
    
    output_fn = "../data/residual_data/cv3/test_CV3_{:04d}.csv"
    
    for i in range(1, 145): #[1-144]
        print("File", i)
        data_x, data_y = group_samples(gt_filename.format(i), odom_filename.format(i))
        
        df = pd.read_csv(odom_filename.format(i), header=None)
        odom_data = df.to_numpy()
        
        train_data = np.concatenate((data_x, data_y), axis=1)
        np.savetxt(output_fn.format(i), train_data, comments='', delimiter=',', header='ts,vx,vy,wz,qd1,qd3,nvx,nvy,nwz')
        

def process_ld3_data():
    odom_filename = "/home/justin/Downloads/LD3/extracted_data/odometry/{:04d}_odom_data.txt"
    gt_filename = "/home/justin/Downloads/LD3/localization_ground_truth/{:04d}_LD_grass_GT.txt"
    
    output_fn = "../data/residual_data/ld3/test_LD3_{:04d}.csv"
    
    i = 1
    print("File", i)
    data_x, data_y = group_samples(gt_filename.format(i), odom_filename.format(i))
    
    df = pd.read_csv(odom_filename.format(i), header=None)
    odom_data = df.to_numpy()
    
    train_data = np.concatenate((data_x, data_y), axis=1)
    np.savetxt(output_fn.format(i), train_data, comments='', delimiter=',', header='ts,vx,vy,wz,qd1,qd3,nvx,nvy,nwz')

    
    
def process_training_data():
    concat_data_x = np.zeros((0,6))
    concat_data_y = np.zeros((0,3))
    
    odom_filename = "/home/justin/Downloads/Train3/extracted_data/odometry/{:04d}_odom_data.txt"
    gt_filename = "/home/justin/Downloads/Train3/localization_ground_truth/{:04d}_Tr_grass_GT.txt"
    
    output_fn = "../data/residual_data/train3/train.csv"
    
    for i in range(1, 18): #[1-17]
        print("File", i)
        data_x, data_y = group_samples(gt_filename.format(i), odom_filename.format(i))
        
        concat_data_x = np.concatenate((concat_data_x, data_x), axis=0)
        concat_data_y = np.concatenate((concat_data_y, data_y), axis=0)
        print("Accumulated data", concat_data_x.shape, concat_data_y.shape)
        
    #sanity checks
    #plt.plot(concat_data_x[:,1])
    #plt.plot(concat_data_y[:,0])
    #plt.show()
    
    train_data = np.concatenate((concat_data_x, concat_data_y), axis=1)
    
    np.savetxt(output_fn, train_data, comments='', delimiter=',', header='ts,vx,vy,wz,qd1,qd3,nvx,nvy,nwz')            
        

#pretty much do nothing. No grouping here. Unlike process_training_data.py
def group_samples(gt_filename, odom_filename):
    #[[timestamp,vx,vy,wz,qd1,qd3]...]
    #[[nvx,nvy,nwz]...]
    raw_features = process_file(gt_filename, odom_filename)
    data_x = raw_features[:,0:6]
    data_y = raw_features[:,6:9]
    return data_x, data_y
    
    
#Time,x,y,rad
#reads file, converts to velocity, then uses interpolation to convert to odometry rate.
def process_file(gt_filename, odom_filename):
    df = pd.read_csv(gt_filename, header=None) 
    gt_data = df.to_numpy()
    
    df = pd.read_csv(odom_filename, header=None)
    odom_data = df.to_numpy()
    
    vx = (gt_data[1:,1] - gt_data[:-1,1]) / (gt_data[1:,0] - gt_data[:-1,0]) #world frame velocity.
    vy = (gt_data[1:,2] - gt_data[:-1,2]) / (gt_data[1:,0] - gt_data[:-1,0])
    yaw_rate = (np.mod((gt_data[1:,3] - gt_data[:-1,3] + np.pi), 2*np.pi) - np.pi) / (gt_data[1:,0] - gt_data[:-1,0])

    world_vel = np.stack([vx,vy,yaw_rate], axis=0)
    body_vel = np.zeros(world_vel.shape)
    
    body_vel[2,:] = world_vel[2,:]
    #rotate velocities into body frame
    for i in range(world_vel.shape[1]):
        #rotation that transforms vectors in world frame to vectors in vehicle frame
        yaw_i = gt_data[i,3]
        rot = np.array([[np.cos(yaw_i), -np.sin(yaw_i)], [np.sin(yaw_i), np.cos(yaw_i)]]).T
        body_vel[0:2,i] = rot@world_vel[0:2,i]

    f_interp = interpolate.interp1d(gt_data[:-1,0], body_vel[0:3,:].T, axis=0, fill_value='extrapolate') #interpolate so that data is down sampled to odom rate. #'extrapolate'
    down_sampled_data = f_interp(odom_data[:,5])
    #timestamp, vx,vy,wz, qd1,qd3, nvx,nvy,nwz
    train_data = np.concatenate([odom_data[:-1,5].reshape(-1,1), down_sampled_data[:-1,:], odom_data[:-1,3].reshape(-1,1), odom_data[:-1,1].reshape(-1,1), down_sampled_data[1:,:]], axis=1)
    
    #plt.plot(gt_data[:,1], gt_data[:,2]) #plot path
    #plt.plot(train_data[1:,0] - train_data[:-1,0]) #plt body vel
    #plt.plot(train_data[:,0], train_data[:,1]) #plt body vel
    #plt.plot(train_data[:,0], train_data[:,2]) #plt body vel
    #plt.plot(train_data[:,1]) #plt body vel
    ##plt.plot(train_data[:,2]) #plt body vel
    ##plt.plot(train_data[:,3])
    #plt.plot(train_data[:,4])
    #plt.plot(train_data[:,5])
    #plt.show()

    return train_data


if __name__ == "__main__":
    main()
