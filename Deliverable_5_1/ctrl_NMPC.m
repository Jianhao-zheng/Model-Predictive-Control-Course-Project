function [ctrl, traj] = ctrl_NMPC(quad) 

import casadi.*

opti = casadi.Opti(); % Optimization problem 

N = 20; % MPC horizon

% ???? decision variables ?????????
X = opti.variable(12, N+1);
U = opti.variable(4, N);

X0 = opti.parameter(12, 1);  % initial state
REF = opti.parameter(4, 1); % reference position [x,y,z,yaw]

h = 0.2; % sampling time
f_discrete = @(x, u) RK4(x, u, h, @(X, U) quad.f(X, U));
% f_discrete = @(x, u) RK4(x, u, h, quad.f);

[~, us] = quad.trim();

% deltX = X;
% deltX(6, :) = deltX(6, :) - REF(4);
% deltX(10, :) = deltX(10, :) - REF(1);
% deltX(11, :) = deltX(11, :) - REF(2);
% deltX(12, :) = deltX(12, :) - REF(3);
% deltU = U;
obj = 0;

for i = 1:12
    if i == 6
        obj = obj + 0.05.*(X(6, :) - REF(4))*(X(6, :) - REF(4))';
    end
    
    if i == 10
        obj = obj + 0.1.*(X(10, :) - REF(1))*(X(10, :) - REF(1))';
    end
    
    if i == 11
        obj = obj + 0.1.*(X(11, :) - REF(2))*(X(11, :) - REF(2))';
    end
    
    if i == 12
        obj = obj + 1.0.*(X(12, :) - REF(3))*(X(12, :) - REF(3))';
    end
    
%     if i == 6 || i >= 10
%         obj = obj + 100.0*deltX(i, :)*deltX(i, :)';
%     end
    
    if i <= 4
%         deltU(i, :) = deltU(i, :) - us(i);
        obj = obj + 0.05.*(U(i, :) - us(i))*(U(i, :) - us(i))';
    end
end

opti.minimize(obj);

for k = 1:N
    opti.subject_to(X(:, k+1) == f_discrete(X(:, k), U(:, k)));
    opti.subject_to(0 <= U(:, k));
    opti.subject_to(U(:, k) <= 1.5);
end

opti.subject_to(X0 == X(:, 1))
% Tentative
% opti.subject_to(X(4, :) <= 0.035);
% opti.subject_to(-0.035 <= X(4, :));
% opti.subject_to(X(5, :) <= 0.035);
% opti.subject_to(-0.035 <= X(5, :));

ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);

end


function u = eval_ctrl(x, ref, opti, X0, REF, X, U) 
% ???? Set the initial state and reference ???? 
opti.set_value(X0, x);
opti.set_value(REF, ref);

% ???? Setup solver NLP    ??????
ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false); 
opti.solver('ipopt', ops);

% ???? Solve the optimization problem ????
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g)); 

end
