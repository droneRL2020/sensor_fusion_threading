from state import State
import numpy as np

class EKF(State):
    def __init__(self):
        super().__init__()

    def predict(self, u_prev, covar_prev, ang_vel, lin_acc, dt):
        state_dot, a_mat, u_mat = self.linearize()
        ang_vel = np.array(ang_vel)
        lin_acc = np.array(lin_acc)
        all_values = {"x":u_prev[0][0],"y":u_prev[1][0],"z":u_prev[2][0],
                      "q_x":u_prev[3][0],"q_y":u_prev[4][0],"q_z":u_prev[5][0],
                      "v_x":u_prev[6][0],"v_y":u_prev[7][0],"v_z":u_prev[8][0],
                      "bg_x":u_prev[9][0],"bg_y":u_prev[10][0],"bg_z":u_prev[11][0],
                      "ba_x":u_prev[12][0],"ba_y":u_prev[13][0],"ba_z":u_prev[14][0],
                      "w_x":ang_vel[0],"w_y":ang_vel[1],"w_z":ang_vel[2],
                      "a_x":lin_acc[0],"a_y":lin_acc[1],"a_z":lin_acc[2],
                      "ng_x":0,"ng_y":0,"ng_z":0,"na_x":0,"na_y":0,"na_z":0,
                      "nbg_x":0,"nbg_y":0,"nbg_z":0,"nba_x":0,"nba_y":0,"nba_z":0}
        
        state_dot_cal = np.array(state_dot.subs(all_values))
        a_mat_cal = np.array(a_mat.subs(all_values))
        u_mat_cal = np.array(u_mat.subs(all_values))

        f_mat = np.eye(15) + dt* a_mat_cal
        q_mat = (1e-3) * np.eye(12)
        
        u_est = u_prev + dt* state_dot_cal
        covar_est = np.matmul(np.matmul(f_mat, covar_prev), f_mat.T) + dt*np.matmul(np.matmul(u_mat_cal,  q_mat), u_mat_cal.T) 

        return covar_est, u_est

    def update(self, z_t, covar_est, u_est):
        c_mat = np.eye(15)[:6]
        r_mat = (1e-3)*np.eye(6)
        k_mat = np.matmul(np.matmul(covar_est, c_mat.T), np.linalg.inv(np.matmul(np.matmul(c_mat, np.float32(covar_est)), c_mat.T) + r_mat))
        
        u_curr = u_est + np.matmul(k_mat, (z_t - np.matmul(c_mat, u_est)))
        covar_curr = covar_est - (np.matmul(np.matmul(k_mat, c_mat), covar_est))
        return u_curr, covar_curr