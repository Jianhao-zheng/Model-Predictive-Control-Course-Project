clear;close all;clc
addpath('D:\Program Files\MATLAB\R2018b\toolbox\casadi-windows-matlabR2016a-v3.5.5')

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
for i =1:3
    figure(1)
    subplot(3,1,i)
    mpc_x.Xf.projection(i:i+1).plot();
    figure(2)
    subplot(3,1,i)
    mpc_y.Xf.projection(i:i+1).plot();
end
figure
mpc_z.Xf.plot();
figure
mpc_yaw.Xf.plot();

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

figure
hold on; grid on;
T = 0:Ts:Ts*(size(X_X,2)-1);
h1 = plot(T,X_X(1,:),'color','r');
h2 = plot(T,X_X(2,:),'color','b');
h3 = plot(T,X_X(3,:),'color','c');
h4 = plot(T,X_X(4,:),'color','m');
h5 = plot(T(1:end-1),U_X,'color','g');
legend([h1;h2;h3;h4;h5],{'vel\_pitch';'pitch';'vel\_x';'x';'M\_alpha'});
title('subsystem X');
xlabel('time(s)')



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

figure
hold on; grid on;
T = 0:Ts:Ts*(size(X_Y,2)-1);
h1 = plot(T,X_Y(1,:),'color','r');
h2 = plot(T,X_Y(2,:),'color','b');
h3 = plot(T,X_Y(3,:),'color','c');
h4 = plot(T,X_Y(4,:),'color','m');
h5 = plot(T(1:end-1),U_Y,'color','g');
legend([h1;h2;h3;h4;h5],{'vel\_row';'row';'vel\_y';'y';'M\_beta'});
title('subsystem Y');
xlabel('time(s)')

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

figure
hold on; grid on;
T = 0:Ts:Ts*(size(X_Z,2)-1);
h1 = plot(T,X_Z(1,:),'color','r');
h2 = plot(T,X_Z(2,:),'color','b');
h3 = plot(T(1:end-1),U_Z,'color','g');
legend([h1;h2;h3],{'vel\_z';'z';'F'});
title('subsystem Z');
xlabel('time(s)')

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

figure
hold on; grid on;
T = 0:Ts:Ts*(size(X_yaw,2)-1);
h1 = plot(T,X_yaw(1,:),'color','r');
h2 = plot(T,X_yaw(2,:),'color','b');
h3 = plot(T(1:end-1),U_yaw,'color','g');
legend([h1;h2;h3],{'vel\_yaw';'yaw';'M\_gamma'});
title('subsystem yaw');
xlabel('time(s)')

