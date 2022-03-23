import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.autograd as autograd


import pandas as pd
from sklearn.preprocessing import MinMaxScaler, StandardScaler
from sklearn.metrics import mean_squared_error
from sklearn.utils import shuffle
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

from pickle import load
from pickle import dump

#import pdb; pdb.set_trace() #this is completely overpowered. Too useful.


dir_prefix = "/home/justin/Downloads"



#load test and training data
def load_data():
    #in_features =  ['qx', 'qy', 'qz', 'qw', 'vl','vr','zr1','zr2','zr3','zr4']
    #vel_features = ['vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'qd1', 'qd2', 'qd3', 'qd4']
    #in_features = ['vx','vy','vz', 'vl','vr','roll','pitch', 'qd1', 'qd2', 'qd3', 'qd4']
    in_features = ['qd1_0','qd3_0']
    out_features = ["vx","vy","wz"]
    
    df_x = pd.read_csv("../data/joydeep_data/training_x.csv")
    df_y = pd.read_csv("../data/joydeep_data/training_y.csv")

    #import pdb; pdb.set_trace()
    
    #print(df_x)
    
    skip_rows = 0
    data_x = np.array(df_x[in_features])    
    data_x = data_x[skip_rows:]
    
    data_y = np.array(df_y[out_features])
    data_y = data_y[skip_rows:]
    
    data_len = data_x.shape[0]
    
    train_data = data_x[:int(data_len),:]
    label_data = data_y[:int(data_len),:]
    
    train_data, label_data = shuffle(train_data, label_data, random_state=1)    
    
    df_x = pd.read_csv("../data/joydeep_data/cv3/test_x_CV3_0001.csv")
    df_y = pd.read_csv("../data/joydeep_data/cv3/test_y_CV3_0001.csv")
    test_data = np.array(df_x[in_features])
    test_labels = np.array(df_y[out_features])
    
    return train_data, label_data, test_data, test_labels
    
    
    
class VehicleNet(nn.Module):
  def __init__(self, mn):
    super(VehicleNet, self).__init__()
    
    self.in_size = 2
    self.out_size = 3
    
    self.last_layer = nn.Linear(self.in_size, self.out_size)
    
    self.loss_fn = torch.nn.MSELoss()
    self.opt = torch.optim.Adam(self.last_layer.parameters(), lr=1e-1)
    self.count = 0
    self.model_name = mn
        
  def save(self):
    md = self.last_layer.state_dict()
    torch.save(md, self.model_name)
    return

  def load(self, name):
    md = torch.load(name)
    self.last_layer.load_state_dict(md)
    return
    
  def forward_partial(self, x):
    out1 = self.last_layer(x)
    return out1

  def get_validation_loss(self):
    x = test_data
    y = test_labels
    
    self.opt.zero_grad()
    y_hat = self.forward_partial(x)
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
        y_hat = self.forward_partial(x)
        loss = self.loss_fn(y_hat, y)
        loss.backward()
        self.opt.step()

      self.get_validation_loss()
      plt.scatter(self.count, loss.item(), color='b')
      self.count += 1
      print("LOSS", loss)


    

print("going to load data")
train_data, label_data, test_data, test_labels = load_data()
print("done loading")

train_data = torch.from_numpy(train_data).float()
label_data = torch.from_numpy(label_data).float()

test_data = torch.from_numpy(test_data).float()
test_labels = torch.from_numpy(test_labels).float()

print("train data size ", train_data.size())
print("label data size ", label_data.size())
print("has gpu? ", torch.cuda.is_available())

#import pdb; pdb.set_trace() #this is completely overpowered. Too useful.

def network_forward(model, x):
  x_torch= torch.from_numpy(x).float()
  yhat = model.forward_partial(x_torch).detach().numpy()
  return yhat

def test_network(model):
    yhat = model.forward(test_data).detach().numpy()
    
    cpu_test_labels = test_labels.detach().numpy()
    
    print("Evaluation Loss", np.mean(np.abs(yhat.flatten() - cpu_test_labels.flatten())))
    
    idx = 2
    plt.plot(yhat[0:1000,idx])
    plt.plot(cpu_test_labels[0:1000,idx])
    plt.show()

def test_vehicle_network(model):
  df = pd.read_csv(dir_prefix + "/CV3/localization_ground_truth/0001_CV_grass_GT.txt", header=None)
  gt_pos = df.to_numpy()
  gt_pos = gt_pos[:,1:3]
  print("gt_pos.shape", gt_pos.shape)
  
  init_x = gt_pos[0,0]
  init_y = gt_pos[0,1]
  plt.plot(gt_pos[:,0] - init_x, gt_pos[:,1] - init_y, color='blue')
  #plt.show()
  
  pos = np.array([0.0,0.0])
  features = np.zeros([2], dtype=float)
  
  yhat = np.array([0.0, 0, 0])
  predicted_path = np.zeros([test_data.shape[0]+1, 3])
  predicted_path[0][2] = 0
  
  ts = .001
  
  for i in range(test_data.shape[0]):    
    features[0] = test_data[i,0]
    features[1] = test_data[i,1]
    
    yhat = network_forward(model, features.reshape(1, -1)).reshape((3,))
    
    temp_state = predicted_path[i][:]
    for j in range(50):
        rot = np.array([[np.cos(temp_state[2]), -np.sin(temp_state[2])],[np.sin(temp_state[2]), np.cos(temp_state[2])]])
        vel = np.dot(rot, yhat[0:2])  #transform to world coordinates so we can integrate
        
        temp_state[0] = temp_state[0] + vel[0]*ts
        temp_state[1] = temp_state[1] + vel[1]*ts
        temp_state[2] = temp_state[2] + yhat[2]*ts

    predicted_path[i+1][:] = temp_state
  
  plt.plot(predicted_path[:,0], predicted_path[:,1], color='red')
  plt.show()
  #plt.plot(predicted_path[:,2], color='blue')
  #plt.show()
  return



model_name = "../data/lin_vehicle.net"

model = VehicleNet(model_name)
#import pdb; pdb.set_trace()


model.load(model_name)
print("Training All Layers")
#for i in range(4):
#model.optimize_all()
#model.fit(1e-2, 1000, 100)
#model.fit(1e-4, 1000, 100)
# print("Training only last layer")
# model.optimize_last_layer()
# model.fit(1e-4, 100, 1000)
# print("Training All Layers")
# model.optimize_all()
# model.fit(1e-4, 100, 1000)
  
plt.show()

#model.save()

#test_features = np.array([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16])
#yhat = network_forward(model, test_features.reshape(1, -1)).reshape((3,))
#print("1 vector output: ", yhat)

test_vehicle_network(model)

md = model.state_dict()

#import pdb; pdb.set_trace() #this is completely overpowered. Too useful.



