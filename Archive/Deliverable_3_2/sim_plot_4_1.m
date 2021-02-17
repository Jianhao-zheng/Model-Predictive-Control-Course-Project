clc;clear;close all;
% addpath('D:\Program Files\MATLAB\R2018b\toolbox\casadi-windows-matlabR2016a-v3.5.5')

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%%
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

%%
sim = quad.sim(mpc_x,mpc_y,mpc_z,mpc_yaw);
quad.plot(sim);