%% Deliverable 3.2 Design MPC Tracking Controllers
clc;clear;close all;
addpath("../plotUtilis/");
titleFontSize = 12;
Deliverable = "3_2";

%% initialization
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

%% Plot for X starting stationary at the origin and tracking a reference to -2 meters from the origin 

% initial state
x0_X = [0;0;0;0];
x_pos_ref = -2;

X_X = x0_X;
U_X = 0;
while norm(X_X(:,end)-[0;0;0;x_pos_ref],2) > 1e-3
    uopt = mpc_x.get_u(X_X(:,end),x_pos_ref);
    U_X = [U_X uopt];
    X_X = [X_X mpc_x.A*X_X(:,end)+mpc_x.B*uopt];
end
U_X(:,1) = [];
% plot response
Var = "x";
drawOneState(X_X,U_X,Var,Deliverable);

%% Plot for Y starting stationary at the origin and tracking a reference to -2 meters from the origin 
x0_Y = [0;0;0;0];
y_pos_ref = -2;

X_Y = x0_Y;
U_Y = 0;
while norm(X_Y(:,end)-[0;0;0;y_pos_ref],2) > 1e-3
    uopt = mpc_y.get_u(X_Y(:,end),y_pos_ref);
    U_Y = [U_Y uopt];
    X_Y = [X_Y mpc_y.A*X_Y(:,end)+mpc_y.B*uopt];
end
U_Y(:,1) = [];
% plot response
Var = "y";
drawOneState(X_Y,U_Y,Var,Deliverable);

%% Plot for Z starting stationary at the origin and tracking a reference to ?2 meters from the origin 
x0_Z = [0;0];
z_pos_ref = -2;

X_Z = x0_Z;
U_Z = 0;
while norm(X_Z(:,end)-[0;z_pos_ref],2) > 1e-3
    uopt = mpc_z.get_u(X_Z(:,end),z_pos_ref);
    U_Z = [U_Z uopt];
    X_Z = [X_Z mpc_z.A*X_Z(:,end)+mpc_z.B*uopt];
end
U_Z(:,1) = [];
% plot response
Var = "z";
drawOneState(X_Z,U_Z,Var,Deliverable);

%% Plot for yaw starting stationary at the origin and tracking a reference to 45 degrees for yaw
x0_yaw = [0;0];
yaw_pos_ref = pi/4;

X_yaw = x0_yaw;
U_yaw = 0;
while norm(X_yaw(:,end)-[0;yaw_pos_ref],2) > 1e-3
    uopt = mpc_yaw.get_u(X_yaw(:,end),yaw_pos_ref);
    U_yaw = [U_yaw uopt];
    X_yaw = [X_yaw mpc_yaw.A*X_yaw(:,end)+mpc_yaw.B*uopt];
end
U_yaw(:,1) = [];
% plot response
Var = "yaw";
drawOneState(X_yaw,U_yaw,Var,Deliverable);