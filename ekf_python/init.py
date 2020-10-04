import numpy as np
import pandas as pd

def get_data():
    data = pd.read_csv("drone_dataset.csv")
    lin_acc = np.array([data["lin_acc_x"], data["lin_acc_y"], data["lin_acc_z"]]).T
    ang_vel = np.array([data["ang_vel_x"], data["ang_vel_y"], data["ang_vel_z"]]).T
    vicon_pose = np.array([data["w_pos_x"],data["w_pos_y"],data["w_pos_z"],data["w_ori_x"],data["w_ori_y"],data["w_ori_z"]]).T
    t = data["t"]
    return lin_acc, ang_vel, vicon_pose, t



# if __name__ == "__main__":
#     get_data()
