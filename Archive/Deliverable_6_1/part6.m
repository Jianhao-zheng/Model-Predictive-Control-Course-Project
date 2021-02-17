clear;close all;clc
addpath('D:\Program Files\MATLAB\R2018b\toolbox\casadi-windows-matlabR2016a-v3.5.5')

quad = Quad();
CTRL = ctrl_NMPC(quad);
sim = quad.sim(CTRL);
quad.plot(sim);