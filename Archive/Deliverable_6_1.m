%% Part6 Nonlinear MPC
clc;clear;close all;
addpath("../Archive/Deliverable_6_1/");
addpath("../plotUtilis/");

%% simulation
quad = Quad();
CTRL = ctrl_NMPC(quad);
sim = quad.sim(CTRL);
% quad.plot(sim);
Deliverable="6_1"; Nplots=10; SaveFile = true;
plotRes(sim, Deliverable, Nplots, SaveFile);