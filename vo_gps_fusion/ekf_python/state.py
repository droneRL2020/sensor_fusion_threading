import sympy as sp
import numpy as np

class State(object):
    def __init__(self):
        self.x = sp.Symbol("x")
        self.y = sp.Symbol("y")
        self.z = sp.Symbol("z")
        self.q_x = sp.Symbol("q_x")
        self.q_y = sp.Symbol("q_y")
        self.q_z = sp.Symbol("q_z")
        self.v_x = sp.Symbol("v_x")
        self.v_y = sp.Symbol("v_y")
        self.v_z = sp.Symbol("v_z")
        self.bg_x = sp.Symbol("bg_x")
        self.bg_y = sp.Symbol("bg_y")
        self.bg_z = sp.Symbol("bg_z")
        self.ba_x = sp.Symbol("ba_x")
        self.ba_y = sp.Symbol("ba_y")
        self.ba_z = sp.Symbol("ba_z")
        self.w_x = sp.Symbol("w_x")
        self.w_y = sp.Symbol("w_y")
        self.w_z = sp.Symbol("w_z")
        self.a_x = sp.Symbol("w_x")
        self.a_y = sp.Symbol("w_y")
        self.a_z = sp.Symbol("w_z")
        self.ng_x = sp.Symbol("ng_x")
        self.ng_y = sp.Symbol("ng_y")
        self.ng_z = sp.Symbol("ng_z")
        self.na_x = sp.Symbol("na_x")
        self.na_y = sp.Symbol("na_y")
        self.na_z = sp.Symbol("na_z")
        self.nbg_x = sp.Symbol("nbg_x")
        self.nbg_y = sp.Symbol("nbg_y")
        self.nbg_z = sp.Symbol("nbg_z")
        self.nba_x = sp.Symbol("nba_x")
        self.nba_y = sp.Symbol("nba_y")
        self.nba_z = sp.Symbol("nba_z")
        self.state = sp.Matrix([[self.x],[self.y],[self.z],[self.q_x],[self.q_y],[self.q_z],[self.v_x],[self.v_y],[self.v_z],[self.bg_x],[self.bg_y],[self.bg_z],[self.ba_x],[self.ba_y],[self.ba_z]])
        self.noise = sp.Matrix([[self.ng_x],[self.ng_y],[self.ng_z],[self.na_x],[self.na_y],[self.na_z],[self.nba_x],[self.nba_y],[self.nba_z],[self.nbg_x],[self.nbg_y],[self.nbg_z]])
        # print(type(self.state))
        # print(type(self.noise))
        # self.jacobian(self.state, self.noise)

    def linearize(self):
        w_ang_vel = self.map_ang_vel_in_world()
        w_lin_acc = self.map_lin_acc_in_world()
        # print(w_lin_acc.T.shape)
        state_dot = sp.Matrix([[self.v_x],[self.v_y],[self.v_z],[w_ang_vel[0]],[w_ang_vel[1]],[w_ang_vel[2]],
                          [w_lin_acc[0]],[w_lin_acc[1]],[w_lin_acc[2]],[self.nbg_x],[self.nbg_y],[self.nbg_z], 
                          [self.nba_x],[self.nba_y],[self.nba_z]])
        # print(type(state_dot))
        a_mat = self.jacobian(state_dot, self.state)
        u_mat = self.jacobian(state_dot, self.noise)
        return state_dot, a_mat, u_mat


    def jacobian(self, f_x, x):
        return f_x.jacobian(x)

    def map_ang_vel_in_world(self):
        g_mat = sp.Matrix([[sp.cos(self.q_y), 0, -(sp.cos(self.q_x)*sp.sin(self.q_y))],
                            [0,          1,  sp.sin(self.q_x)               ],  
                            [sp.sin(self.q_y),0,  (sp.cos(self.q_x)*sp.cos(self.q_y))]])
        inverse_g = g_mat.T
        ang_vel_in_world = sp.MatMul(inverse_g, sp.Matrix([[self.w_x - self.bg_x - self.ng_x],
                                                           [self.w_y - self.bg_y - self.ng_y],
                                                           [self.w_z - self.bg_z - self.ng_z]]))
        ang_vel_in_world = sp.Matrix(ang_vel_in_world)
        # print(ang_vel_in_world.shape)
        # a = ang_vel_in_world[0,:]
        # b = ang_vel_in_world[1,:]
        # c = sp.Matrix([a,b])
        # print(a.shape, b.shape, c.shape)
        return ang_vel_in_world

    def map_lin_acc_in_world(self):
        r_mat = sp.Matrix([[(sp.cos(self.q_z)*sp.cos(self.q_y)) - (sp.sin(self.q_x)*sp.sin(self.q_z)*sp.sin(self.q_y)),
                  -(sp.cos(self.q_x)*sp.cos(self.q_z)), 
                  (sp.cos(self.q_z)*sp.sin(self.q_y)) + (sp.cos(self.q_y)*sp.sin(self.q_x)*sp.sin(self.q_z))],
                  [(sp.cos(self.q_y)*sp.sin(self.q_z)) - (sp.cos(self.q_z)*sp.sin(self.q_x)*sp.sin(self.q_y)),
                  (sp.cos(self.q_x)*sp.cos(self.q_z)), 
                  (sp.sin(self.q_z)*sp.sin(self.q_y)) + (sp.cos(self.q_z)*sp.cos(self.q_y)*sp.sin(self.q_x))],
                  [-sp.cos(self.q_x)*sp.sin(self.q_y),
                   sp.sin(self.q_x), 
                   sp.cos(self.q_x)*sp.cos(self.q_y)]])
        a = sp.MatMul(r_mat, sp.Matrix([[self.a_x - self.ba_x - self.na_x],
                                        [self.a_y - self.ba_y - self.na_y],
                                        [self.a_z - self.ba_z - self.na_z]]))
        lin_acc_in_world = sp.Matrix([[0],[0],[-9.81]]) + a
        return lin_acc_in_world


# if __name__ == "__main__":
#     state = State()
#     f1 = state.x**2
#     f2 = state.y**2
#     f3 = state.z**2
#     f4 = state.q_x**2

#     x = sp.Matrix([state.x,state.y,state.z, state.q_x])
#     f_x = sp.Matrix([f1,f2,f3, f4])
#     state.jacobian(f_x, x)