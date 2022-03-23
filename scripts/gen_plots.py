import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def compare_nn_lin_cv3():
    nn_err = pd.read_csv("err_nn_model.csv", header=None).to_numpy()
    lin_err = pd.read_csv("err_lin_model.csv", header=None).to_numpy()
    
    print(nn_err.shape)

    plt.subplot(2,1,1)
    plt.title("Linear Error")
    plt.step(np.arange(0,144,1), lin_err[:,0])
    plt.step(np.arange(0,144,1), nn_err[:,0])
    plt.ylabel("Relative Mean Error (%)")
    plt.legend(["Linear","Neural Net"])
    
    plt.subplot(2,1,2)
    plt.title("Angular Yaw Error")
    plt.step(np.arange(0,144,1), lin_err[:,1])
    plt.step(np.arange(0,144,1), nn_err[:,1])
    plt.ylabel("Relative Mean Error (%)")
    plt.legend(["Linear","Neural Net"])
    
    plt.tight_layout()
    plt.show()

def create_example_plots():
    nn_pos = pd.read_csv("example_cv3_35_neural.csv", header=None).to_numpy()
    lin_pos = pd.read_csv("example_cv3_35_linear.csv", header=None).to_numpy()
    gt_pos = pd.read_csv("example_cv3_35.csv", header=None).to_numpy()

    plt.subplot(1,2,1)
    plt.title("Trajectory 35 from CV3 Dataset")
    plt.plot(gt_pos[:,0], gt_pos[:,1])
    plt.plot(lin_pos[:,0], lin_pos[:,1])
    plt.plot(nn_pos[:,0], nn_pos[:,1])
    plt.xlabel("m")
    plt.ylabel("m")
    plt.legend(["Ground Truth","Linear","Neural Net"])
    
    nn_pos = pd.read_csv("example_cv3_55_neural.csv", header=None).to_numpy()
    lin_pos = pd.read_csv("example_cv3_55_linear.csv", header=None).to_numpy()
    gt_pos = pd.read_csv("example_cv3_55.csv", header=None).to_numpy()
    
    plt.subplot(1,2,2)
    plt.title("Trajectory 55 from CV3 Dataset")
    plt.plot(gt_pos[:,0], gt_pos[:,1])
    plt.plot(lin_pos[:,0], lin_pos[:,1])
    plt.plot(nn_pos[:,0], nn_pos[:,1])
    plt.xlabel("m")
    plt.ylabel("m")
    #plt.legend(["Ground Truth","Linear","Neural Net"])
    plt.tight_layout()
    plt.show()    


def plot_tire_model():
    forces = pd.read_csv("/home/justin/force_log.csv")
    plt.scatter(forces['zr'], forces['bk_fz'], color='r', alpha=.4, s=.1)
    plt.scatter(forces['zr'], forces['nn_fz'], color='b', alpha=.4, s=.1)
    leg = plt.legend(['Bekker Model','Neural Network'], markerscale=10)
    for lh in leg.legendHandles: 
        lh.set_alpha(1)
    plt.title("Sinkage vs Normal Force")
    plt.xlabel('Sinkage (m)')
    plt.ylabel('Normal Force (N)')
    plt.show()

def plot_dynamic_model_examples():
    plt.subplot(1,2,1)
    plt.title("Trajectory 55 from CV3 Dataset")
    df = pd.read_csv("/home/justin/Downloads/CV3/localization_ground_truth/0055_CV_grass_GT.txt", header=None)
    end_time = df[0][0] + 6
    plt.plot(df[1][df[0] < end_time], df[2][df[0] < end_time])
    
    model_pos = pd.read_csv("xout_55.csv")
    plt.plot(model_pos['x'], model_pos['y'])
    
    plt.xlabel("m")
    plt.ylabel("m")
    plt.legend(["Ground Truth","Dynamic Model with\nTire Network"])
    
    plt.subplot(1,2,2)
    plt.title("Trajectory 132 from CV3 Dataset")
    df = pd.read_csv("/home/justin/Downloads/CV3/localization_ground_truth/0132_CV_grass_GT.txt", header=None)
    end_time = df[0][0] + 6
    plt.plot(df[1][df[0] < end_time], df[2][df[0] < end_time])

    model_pos = pd.read_csv("xout_132.csv")
    plt.plot(model_pos['x'], model_pos['y'])
    plt.xlabel("m")
    plt.ylabel("m")
    #plt.legend(["Ground Truth","Dynamic Model with Tire Network"])
    plt.tight_layout()
    plt.show()

    
plot_dynamic_model_examples()
#compare_nn_lin_cv3()
#create_example_plots()
#plot_tire_model()

