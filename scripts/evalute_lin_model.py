import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.autograd as autograd


import pandas as pd
from sklearn.metrics import mean_squared_error
from sklearn.utils import shuffle
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import sys

from pickle import load
from pickle import dump

#import pdb; pdb.set_trace() #this is completely overpowered. Too useful.

dir_prefix = "/home/justin/Downloads"

#load test and training data
def load_training_data():
    in_features = ['qd1_0','qd3_0']
    out_features = ["vx","vy","wz"]
    
    df_x = pd.read_csv("../data/joydeep_data/training_x.csv")
    df_y = pd.read_csv("../data/joydeep_data/training_y.csv")
    
    skip_rows = 0
    data_x = np.array(df_x[in_features])    
    data_x = data_x[skip_rows:]
    
    data_y = np.array(df_y[out_features])
    data_y = data_y[skip_rows:]
        
    data_len = data_x.shape[0]
    
    train_data = data_x[:int(data_len),:]
    label_data = data_y[:int(data_len),:]
    
    train_data, label_data = shuffle(train_data, label_data, random_state=1)    
    
    return train_data, label_data
    
    
    
class VehicleNet(nn.Module):
  def __init__(self, mn):
    super(VehicleNet, self).__init__()
    
    self.in_size = 2
    self.out_size = 3
    
    self.layer = nn.Linear(self.in_size, self.out_size)
    
    self.loss_fn = torch.nn.MSELoss()
    self.opt = torch.optim.Adam(self.layer.parameters(), lr=1e-1)
    self.count = 0
    self.model_name = mn
    
    
  def forward(self, x):
    return self.layer.forward(x)

  def save(self):
    md = self.layer.state_dict()
    torch.save(md, self.model_name)
    return

  def load(self, name):
    md = torch.load(name)
    self.layer.load_state_dict(md)
    return
    

  def get_validation_loss(self):
    x = test_data
    y = test_labels
    
    self.opt.zero_grad()
    y_hat = self.forward(x)
    loss = self.loss_fn(y_hat, y)
    plt.scatter(self.count, loss.item(), color='r')
      
  def fit(self, lr, batch_size, epochs):
    for param_group in self.opt.param_groups:
      param_group['lr'] = lr
        
    for j in range(epochs):
      for i in range(0, train_data.shape[0]-batch_size, batch_size):
        x = train_data[i:i+batch_size, :]
        y = label_data[i:i+batch_size, :]
        
        self.opt.zero_grad()
        y_hat = self.forward(x)
        loss = self.loss_fn(y_hat, y)
        loss.backward()
        self.opt.step()

      self.get_validation_loss()
      plt.scatter(self.count, loss.item(), color='b')
      self.count += 1
      print("LOSS", loss)


    

print("going to load data")
train_data, label_data = load_training_data()
print("done loading")

train_data = torch.from_numpy(train_data).float()
label_data = torch.from_numpy(label_data).float()

print("train data size ", train_data.size())
print("label data size ", label_data.size())
print("has gpu? ", torch.cuda.is_available())

def network_forward(model, x):
    x_torch = torch.from_numpy(x).float()
    yhat = model.forward(x_torch).detach().numpy()
    return yhat

#returns linear and angular displacement
#expects gt_pos to be [[x,y,yaw], ...]
def get_path_displacement(gt_pos):
    diff = gt_pos[0,:] - gt_pos[-1,:]
    lin_disp = (diff[0]*diff[0]) + (diff[1]*diff[1]) #distance from start to end
    ang_disp = np.abs(np.mod((diff[2] + np.pi), 2*np.pi) - np.pi) #difference in rotation from start to end
    return lin_disp, ang_disp
    
def get_path_distance(gt_pos):
  diff = gt_pos[1:,:] - gt_pos[:-1,:]
  diff_sq = np.multiply(diff[:,0:2], diff[:,0:2]) #dx^2, dy^2
  dist = np.sqrt(np.sum(diff_sq, axis=1))         #sqrt(dx^2 + dy^2)
  path_len = np.sum(dist)                         #sum(length of segments)
  
  angle_diff = np.abs(np.mod((diff[:,2] + np.pi), 2*np.pi) - np.pi)
  yaw_displacement = np.sum(angle_diff)
  
  return path_len, yaw_displacement
  

#This function loads the data and tests the data over 6 second time horizons
def test_vehicle_network(model, gt_filename, features_filename):
    #First load all data
    in_features = ['ts','qd1_0','qd3_0','qd1_1','qd3_1','qd1_2','qd3_2','qd1_3','qd3_3','qd1_4','qd3_4','qd1_5','qd3_5','qd1_6','qd3_6','qd1_7','qd3_7']
    df_x = pd.read_csv(features_filename)
    test_features = np.array(df_x[in_features])
    
    df = pd.read_csv(gt_filename, header=None)
    gt_pos = df.to_numpy()
        
    sim_len = 6
    
    cur_test_features = test_features
    cur_gt_pos = gt_pos

    sum_lin_sre = 0
    sum_ang_sre = 0
    count = 0
    
    while(True):        
        #idx is 6 seconds ahead of the start_time
        idx = 0
        start_time = cur_test_features[0,0]
        for i in range(cur_test_features.shape[0]):
            if((cur_test_features[i,0] - start_time) > sim_len):
                idx = i
                break
        
        #this condition checks if we reached the end of the dataset.
        if(idx == 0):
            break
        
        #select features that are within a 6 second window.
        window_test_features = cur_test_features[:idx+1,:]
        stop_time = window_test_features[-1,0] #save the stop time
        window_test_features = window_test_features[:,1:]  #remove timestamp column
        
        cur_gt_pos = gt_pos #this is probably not necessary

        #print("time diff", cur_test_features[idx,0] - start_time)
        
        start_idx = 0
        #search for start index that most closely matches feature start idx
        for i in range(gt_pos.shape[0]):
            if(cur_gt_pos[i,0] > start_time):
                start_idx = i
                break
            
        #remove rows off the front so that cur_gt_pos lines up with cur_test_features
        cur_gt_pos = cur_gt_pos[start_idx:,:]
            
        if(start_idx == 0):
            print("Problemo", i)

        #import pdb; pdb.set_trace()
            
        #search for the index in cur_gt_pos that corresponds to the stop index in window_test_features
        stop_idx = 0
        for i in range(cur_gt_pos.shape[0]):
            if(cur_gt_pos[i,0] > stop_time):
                stop_idx = i
                break
        
        #this condition checks if we reached the end of the dataset.
        if(stop_idx == 0):
            break

        #print("time diff", cur_gt_pos[stop_idx,0] - start_time)
        
        #select 6 second window and select x,y,yaw columns
        window_gt_pos = cur_gt_pos[:stop_idx+1,:]
        window_gt_pos = window_gt_pos[:,1:4]
        
        #get relative error over 6 second prediction horizon
        lin_re, ang_re = test_time_horizon(window_gt_pos, window_test_features)
        
        sum_lin_sre += lin_re*lin_re
        sum_ang_sre += ang_re*lin_re
        count += 1
        
        
        
        #advance by one timestep
        cur_test_features = cur_test_features[idx:,:]
        break
        
    #plt.show()
    return sum_lin_sre,sum_ang_sre,count

#test vehicle over 6 second horizon
def test_time_horizon(gt_pos, test_features):
  #offset to start
  init_x = gt_pos[0,0]
  init_y = gt_pos[0,1]
  plt.plot(gt_pos[:,0], gt_pos[:,1], color='blue', alpha=.5)
  
  
  pos = np.array([0.0,0.0])
  features = np.zeros([2], dtype=float)
  
  yhat = np.array([0.0, 0, 0])
  predicted_path = np.zeros([test_features.shape[0]+1, 3])
  
  predicted_path[0][0] = gt_pos[0][0]
  predicted_path[0][1] = gt_pos[0][1]
  predicted_path[0][2] = gt_pos[0][2]
  
  ts = .05
  
  for i in range(test_features.shape[0]):    
    features[0] = test_features[i,0]
    features[1] = test_features[i,1]
    
    yhat = network_forward(model, features.reshape(1, -1)).reshape((3,))
        
    theta = predicted_path[i][2]
    rot = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
    vel = np.dot(rot, yhat[0:2])  #transform to world coordinates so we can integrate
    
    temp_int = predicted_path[i][:]
    #for k in range(50):
    temp_int[0] = temp_int[0] + vel[0]*ts
    temp_int[1] = temp_int[1] + vel[1]*ts
    temp_int[2] = temp_int[2] + yhat[2]*ts
    
    predicted_path[i+1][:] = temp_int
  
  #plt.plot(predicted_path[:,0], predicted_path[:,1], color='red', alpha=.5)
  
  #np.savetxt("example_cv3_55.csv", gt_pos, delimiter=",")
  #np.savetxt("example_cv3_55_linear.csv", predicted_path, delimiter=",")
  plt.show()
  
  #sys.exit()
  
  total_lin_disp, total_ang_disp = get_path_distance(gt_pos)
  
  
  #print("Displacement", total_lin_disp, total_ang_disp)
  dx = predicted_path[-1,0] - gt_pos[-1,0]
  dy = predicted_path[-1,1] - gt_pos[-1,1]
  dyaw = predicted_path[-1,2] - gt_pos[-1,2]
  dyaw = np.abs(np.mod((dyaw + np.pi), 2*np.pi) - np.pi)
  return (np.sqrt((dx*dx) + (dy*dy))) / total_lin_disp, dyaw / total_ang_disp


def evaluate_cv3_paths(model):
  gt_filename = dir_prefix + "/CV3/localization_ground_truth/{:04d}_CV_grass_GT.txt"
  features_filename = "../data/joydeep_data/cv3/test_x_CV3_{:04d}.csv"
  
  sum_lin_sre = 0
  sum_ang_sre = 0

  total_count = 0
  
  err_list = np.zeros((144,2))
  
  for i in range(1, 145):
    lin_sre, ang_sre, count = test_vehicle_network(model, gt_filename.format(i), features_filename.format(i))
    print("Trajectory", i, "Error", np.sqrt(lin_sre)/count, np.sqrt(ang_sre)/count, count)
    err_list[i-1][0] = np.sqrt(lin_sre)/count
    err_list[i-1][1] = np.sqrt(ang_sre)/count
    
    sum_lin_sre += lin_sre
    sum_ang_sre += ang_sre
    total_count += count

  #import pdb; pdb.set_trace()
    
  np.savetxt("err_lin_model.csv", err_list, delimiter=",") #save the errors and create a bar chart.
  #plt.bar(np.arange(0,144,1), err_list[:,0], width=1)
  plt.show()
  
  lin_rmsre = np.sqrt(sum_lin_sre)/(total_count)
  ang_rmsre = np.sqrt(sum_ang_sre)/(total_count)
  
  print("Total rmrse", lin_rmsre, ang_rmsre)
  
    



def evaluate_ld3_paths(model):
  gt_filename = dir_prefix + "/LD3/localization_ground_truth/0001_LD_grass_GT.txt"
  features_filename = "../data/joydeep_data/ld3/test_x_LD3_0001.csv"
  
  lin_sre, ang_sre, count = test_vehicle_network(model, gt_filename, features_filename)
  
  lin_mare = np.sqrt(lin_sre)/count
  ang_mare = np.sqrt(ang_sre)/count
  
  print("Total rmrse", lin_mare, ang_mare)





  
model_name = "../data/lin_vehicle.net"
model = VehicleNet(model_name)
model.load(model_name)

#evaluate_ld3_paths(model)
evaluate_cv3_paths(model)


#md = model.state_dict()

#import pdb; pdb.set_trace() #this is completely overpowered. Too useful.
#print_c_network(md, input_scaler, output_scaler)

