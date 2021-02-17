%% Part6 Nonlinear MPC
clear;close all;clc

%% simulation
quad = Quad();
CTRL = ctrl_NMPC(quad);
sim = quad.sim(CTRL);
quad.plot(sim);