function [x_next] = RK4(X,U,f)
%
% Inputs : 
%    X, U current state and input
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future

h = 1/5;

% Runge-Kutta 4 integration
k1 = f(X, U);
k2 = f(X + h/2 * k1, U);
k3 = f(X + h/2 * k2, U);
k4 = f(X + h * k3, U);
x_next = X + h/6 * (k1 + 2*k2 +2*k3 + k4);
    
end