import numpy as np
import pandas as pd
from init import get_data
from ekf import EKF
import matplotlib.pyplot as plt
import time
lin_acc, ang_vel, vicon_pose, t = get_data()
u_prev = np.concatenate([vicon_pose[0], np.zeros(9)], axis=0)
u_prev = np.expand_dims(u_prev, axis=1)
covar_prev = np.eye(15)
saved_states = np.zeros([15, len(t)])
ekf = EKF()
curr_time = time.time()
for i in range(len(t)):
    if(i==0):
        dt = t[i]
        covar_est, u_est = ekf.predict(u_prev, covar_prev, ang_vel[i], lin_acc[i], dt)
        z_t = np.expand_dims(vicon_pose[i], axis=1)
        u_curr, covar_curr = ekf.update(z_t, covar_est, u_est)
        saved_states[:,i] = np.squeeze(u_curr, axis=1)
        out = u_curr
        u_prev, covar_prev = u_curr, covar_curr
    else:
        print(time.time() - curr_time)
        dt = t[i] - t[i-1]
        covar_est, u_est = ekf.predict(u_prev, covar_prev, ang_vel[i], lin_acc[i], dt)
        z_t = np.expand_dims(vicon_pose[i], axis=1)
        u_curr, covar_curr = ekf.update(z_t, covar_est, u_est)
        saved_states[:,i] = np.squeeze(u_curr, axis=1)
        out = np.concatenate([out, u_curr], axis = 1)
        u_prev, covar_prev = u_curr, covar_curr

plt.plot(t, saved_states[0,:])
plt.show()
