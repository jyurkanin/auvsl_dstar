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


#dir_prefix = "/home/justin/JoydeepDataset"
dir_prefix = "/home/justin/Downloads"

def main():
    process_training_data()
    process_cv3_data()
    process_ld3_data()

#process cv3 data into a form usable by a VehicleNet
def process_cv3_data():
    odom_filename = dir_prefix + "/CV3/extracted_data/odometry/{:04d}_odom_data.txt"
    gt_filename = dir_prefix + "/CV3/localization_ground_truth/{:04d}_CV_grass_GT.txt"
    
    output_fn_x = "../data/joydeep_data/cv3/test_x_CV3_{:04d}.csv"
    output_fn_y = "../data/joydeep_data/cv3/test_y_CV3_{:04d}.csv"
    
    for i in range(1, 145): #[1-144]
        print("File", i)
        data_x, data_y = group_samples(gt_filename.format(i), odom_filename.format(i))
        
        df = pd.read_csv(odom_filename.format(i), header=None)
        odom_data = df.to_numpy()
        data_x = np.concatenate((odom_data[:,5].reshape(-1,1), data_x), axis=1)
        
        np.savetxt(output_fn_x.format(i), data_x, delimiter=',', comments='', header='ts,qd1_0,qd3_0,qd1_1,qd3_1,qd1_2,qd3_2,qd1_3,qd3_3,qd1_4,qd3_4,qd1_5,qd3_5,qd1_6,qd3_6,qd1_7,qd3_7')
        np.savetxt(output_fn_y.format(i), data_y, delimiter=',', comments='', header='vx,vy,wz')

def process_ld3_data():
    odom_filename = dir_prefix + "/LD3/extracted_data/odometry/{:04d}_odom_data.txt"
    gt_filename = dir_prefix + "/LD3/localization_ground_truth/{:04d}_LD_grass_GT.txt"
    
    output_fn_x = "../data/joydeep_data/ld3/test_x_LD3_{:04d}.csv"
    output_fn_y = "../data/joydeep_data/ld3/test_y_LD3_{:04d}.csv"
    
    i = 1
    print("File", i)
    data_x, data_y = group_samples(gt_filename.format(i), odom_filename.format(i))
    
    df = pd.read_csv(odom_filename.format(i), header=None)
    odom_data = df.to_numpy()
    data_x = np.concatenate((odom_data[:,5].reshape(-1,1), data_x), axis=1)
    
    np.savetxt(output_fn_x.format(i), data_x, comments='', delimiter=',', header='ts,qd1_0,qd3_0,qd1_1,qd3_1,qd1_2,qd3_2,qd1_3,qd3_3,qd1_4,qd3_4,qd1_5,qd3_5,qd1_6,qd3_6,qd1_7,qd3_7')
    np.savetxt(output_fn_y.format(i), data_y, comments='', delimiter=',', header='vx,vy,wz')

    
    
def process_training_data():
    concat_data_x = np.zeros((0,16))
    concat_data_y = np.zeros((0,3))
    
    odom_filename = dir_prefix + "/Train3/extracted_data/odometry/{:04d}_odom_data.txt"
    gt_filename = dir_prefix + "/Train3/localization_ground_truth/{:04d}_Tr_grass_GT.txt"
    
    for i in range(1, 18): #[1-17]
        print("File", i)
        data_x, data_y = group_samples(gt_filename.format(i), odom_filename.format(i))
        
        concat_data_x = np.concatenate((concat_data_x, data_x), axis=0)
        concat_data_y = np.concatenate((concat_data_y, data_y), axis=0)
        print("Accumulated data", concat_data_x.shape, concat_data_y.shape)
    
    #sanity checks
    #for i in range(0,16,2):
    #    plt.plot(concat_data_x[:,i])
    #plt.show()
    #
    #for i in range(1,16,2):
    #    plt.plot(concat_data_x[:,i])
    #plt.show()
    #
    #for i in range(0,3,1):
    #    plt.plot(concat_data_y[:,i])
    #plt.show()
    
    #plt.plot()
    #plt.show()

    np.savetxt("../data/joydeep_data/training_x.csv", concat_data_x, comments='', delimiter=',', header='qd1_0,qd3_0,qd1_1,qd3_1,qd1_2,qd3_2,qd1_3,qd3_3,qd1_4,qd3_4,qd1_5,qd3_5,qd1_6,qd3_6,qd1_7,qd3_7')
    np.savetxt("../data/joydeep_data/training_y.csv", concat_data_y, comments='', delimiter=',', header='vx,vy,wz')
            
        

#collect past 8 qd1,qd3 pairs to form input vector
def group_samples(gt_filename, odom_filename):
    #[[timestamp,vx,vy,wz,qd1,qd3]...]
    raw_features = process_file(gt_filename, odom_filename)
    data_x = np.array(raw_features[:,4:6])
    
    #data_z = np.zeros(data_x.shape)

    #import pdb; pdb.set_trace() #this is completely overpowered. Too useful.
    #temp = (data_x[:,0] + data_x[:][1])
    #data_z[:,0] = data_x[:,0] + data_x[:,1]
    #data_z[:,1] = data_x[:,0] - data_x[:,1]
    #data_x = data_z
    
    data_x_0 = np.concatenate([np.zeros([0,2]), data_x[:,:]], axis=0)   # data at t=0
    data_x_1 = np.concatenate([np.zeros([1,2]), data_x[:-1,:]], axis=0) # data at t=-1
    data_x_2 = np.concatenate([np.zeros([2,2]), data_x[:-2,:]], axis=0) # data at t=-2
    data_x_3 = np.concatenate([np.zeros([3,2]), data_x[:-3,:]], axis=0) # data at t=-3
    data_x_4 = np.concatenate([np.zeros([4,2]), data_x[:-4,:]], axis=0) # data at t=-4
    data_x_5 = np.concatenate([np.zeros([5,2]), data_x[:-5,:]], axis=0) # data at t=-5
    data_x_6 = np.concatenate([np.zeros([6,2]), data_x[:-6,:]], axis=0) # data at t=-6
    data_x_7 = np.concatenate([np.zeros([7,2]), data_x[:-7,:]], axis=0) # data at t=-7
    
    data_x = np.concatenate((data_x_0, data_x_1, data_x_2, data_x_3, data_x_4, data_x_5, data_x_6, data_x_7), axis=1)
    data_y = raw_features[:,1:4]
    
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
    
    #skip = 100
    #plt.plot(gt_data[:,1], gt_data[:,2])
    #plt.quiver(gt_data[::skip,1], gt_data[::skip,2], world_vel[0,::skip]*.1, world_vel[1,::skip]*.1,  scale=1, color='g', alpha=.5)
    #plt.show()
    
    f_interp = interpolate.interp1d(gt_data[:-1,0], body_vel[0:3,:].T, axis=0, fill_value='extrapolate') #interpolate so that data is down sampled to odom rate. #'extrapolate'
    down_sampled_data = f_interp(odom_data[:,5])
    #timestamp, vx,vy,wz, qd1,qd3
    train_data = np.concatenate([odom_data[:,5].reshape(-1,1), down_sampled_data, odom_data[:,1].reshape(-1,1), odom_data[:,3].reshape(-1,1)], axis=1)
    
    #plt.plot(gt_data[:,1], gt_data[:,2]) #plot path
    #plt.plot(train_data[1:,0] - train_data[:-1,0]) #plt body vel
    #plt.plot(train_data[:,0], train_data[:,1]) #plt body vel
    #plt.plot(train_data[:,0], train_data[:,2]) #plt body vel
    #plt.plot(train_data[:,1]) #plt body vel
    #plt.plot(train_data[:,2]) #plt body vel
    #plt.plot(train_data[:,3])
    #plt.plot(train_data[:,4])
    #plt.plot(train_data[:,5])
    #plt.show()

    return train_data


if __name__ == "__main__":
    main()
