function [ctrl, traj] = ctrl_NMPC(quad)

import casadi.*
f = @(x,u) quad.f(x,u);

opti = casadi.Opti(); % Optimization problem

N = 20; % MPC horizon 

% ---- decision variables ------------
X = opti.variable(12,N+1); % state trajectory variables
U = opti.variable(4, N); % control trajectory (throttle, brake)

X0 = opti.parameter(12,1); % initial state
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]

Xs = opti.variable(12,1);% steady state
Us = opti.variable(4, 1);

%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE HERE %%%%

% objective
cost=0;
Q = eye(12); Q(10,10) = 5; Q(11,11) = 5; Q(12,12) = 50; Q(6,6) = 5;

% more general solution
% Q(7,7) = 10; Q(8,8) = 10;Q(9,9) = 10;

R = eye(4);
for k = 1:N
    cost = cost + (U(:,k)-Us)'*R*(U(:,k)-Us);
    cost = cost + (X(:,k)-Xs)'*Q*(X(:,k)-Xs);
end
cost = cost + (X(:,end)-Xs)'*Q*(X(:,end)-Xs);

opti.minimize(cost)


% multiple shooting
for k = 1:N
    opti.subject_to(X(:,k+1) == RK4(X(:,k),U(:,k),f));
end

% constraints
opti.subject_to(-pi/6 <= X(4:5,:) <= pi/6);% change the constraint
opti.subject_to(0 <= U <= 1.5);

% initial state
opti.subject_to(X(:,1) == X0);

% constraints on steady state
opti.subject_to(-0.035 <= Xs(4) <= 0.035);
opti.subject_to(-0.035 <= Xs(5) <= 0.035);
opti.subject_to(0 <= Us <= 1.5);
opti.subject_to(Xs(6)==REF(4));
opti.subject_to(Xs(10)==REF(1));
opti.subject_to(Xs(11)==REF(2));
opti.subject_to(Xs(12)==REF(3));
opti.subject_to(Xs == RK4(Xs, Us,f));

%%%%%%%%%%%%%%%%%%%%%%%%

ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end

