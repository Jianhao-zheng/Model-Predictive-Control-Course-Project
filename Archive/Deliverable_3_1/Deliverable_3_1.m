%% Deliverable 3.1 Design MPC Regulators
clc;clear;close all;
addpath("../plotUtilis/");
titleFontSize = 10;
Deliverable = "3_1";

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

%% plot terminal set
BaseName = "3_1_Terminal_set_";
% x
figureX_set = figure;
set(figureX_set,'position',[0 0 1000 230]);
for i =1:3
    subplot(1,3,i)
    mpc_x.Xf.projection(i:i+1).plot();
    title("Terminal set: x - projection("+num2str(i)+":"+num2str(i+1)+")","FontSize",titleFontSize);
end
tightfig;
print(BaseName+"x",'-dpng');
print(BaseName+"x",'-dpdf','-bestfit'); % '-bestfit' | '-fillpage'
% y
figureY_set = figure;
set(figureY_set,'position',[0 0 1000 230]);
for i = 1:3
    subplot(1,3,i)
    mpc_y.Xf.projection(i:i+1).plot();
    title("Terminal set: y - projection("+num2str(i)+":"+num2str(i+1)+")","FontSize",titleFontSize);
end
tightfig;
print(BaseName+"y",'-dpng');
print(BaseName+"y",'-dpdf','-bestfit'); % '-bestfit' | '-fillpage'
% z
figureZ_set = figure;
set(figureZ_set,'position',[0 0 400 300]);
mpc_z.Xf.plot();
title("Terminal set: z","FontSize",titleFontSize);
tightfig;
print(BaseName+"z",'-dpng');
print(BaseName+"z",'-dpdf','-bestfit'); % '-bestfit' | '-fillpage'
% yaw
figureYaw_set = figure;
set(figureYaw_set,'position',[0 0 400 300]);
mpc_yaw.Xf.plot();
title("Terminal set: yaw","FontSize",titleFontSize);
tightfig;
print(BaseName+"yaw",'-dpng');
print(BaseName+"yaw",'-dpdf','-bestfit'); % '-bestfit' | '-fillpage'

%% Plot for X starting stationary at two meters from the origin
% initial state
x0_X = [0;0;0;2];

X_X = x0_X;
U_X = 0;
while norm(X_X(:,end),2) > 1e-3
    uopt = mpc_x.get_u(X_X(:,end));
    U_X = [U_X uopt];
    X_X = [X_X mpc_x.A*X_X(:,end)+mpc_x.B*uopt];
end
U_X(:,1) = [];
% plot response
Var = "x";
drawOneState(X_X,U_X,Var,Deliverable);

%% Plot for Y starting stationary at two meters from the origin
x0_Y = [0;0;0;2];
X_Y = x0_Y;
U_Y = 0;
while norm(X_Y(:,end),2) > 1e-3
    uopt = mpc_y.get_u(X_Y(:,end));
    U_Y = [U_Y uopt];
    X_Y = [X_Y mpc_y.A*X_Y(:,end)+mpc_y.B*uopt];
end
U_Y(:,1) = [];
% plot response
Var = "y";
drawOneState(X_Y,U_Y,Var,Deliverable);

%% Plot for Z starting stationary at two meters from the origin
x0_Z = [0;2];

X_Z = x0_Z;
U_Z = 0;
while norm(X_Z(:,end),2) > 1e-3
    uopt = mpc_z.get_u(X_Z(:,end));
    U_Z = [U_Z uopt];
    X_Z = [X_Z mpc_z.A*X_Z(:,end)+mpc_z.B*uopt];
end
U_Z(:,1) = [];
% plot response
Var = "z";
drawOneState(X_Z,U_Z,Var,Deliverable);

%% Plot for yaw starting stationary at 45 degrees for yaw
x0_yaw = [0;pi/4];

X_yaw = x0_yaw;
U_yaw = 0;
while norm(X_yaw(:,end),2) > 1e-3
    uopt = mpc_yaw.get_u(X_yaw(:,end));
    U_yaw = [U_yaw uopt];
    X_yaw = [X_yaw mpc_yaw.A*X_yaw(:,end)+mpc_yaw.B*uopt];
end
U_yaw(:,1) = [];
% plot response
Var = "yaw";
drawOneState(X_yaw,U_yaw,Var,Deliverable);