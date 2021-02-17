classdef MPC_Control_z < MPC_Control
  properties
    A_bar, B_bar, C_bar % Augmented system for disturbance rejection    
    L                   % Estimator gain for disturbance rejection
  end
  
  methods
    function mpc = MPC_Control_z(sys, Ts)
      mpc = mpc@MPC_Control(sys, Ts);
      
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
    end
    
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   d_est  - disturbance estimate
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.3)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);

      % SET THE HORIZON HERE
      N = 20;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
      
      % Tentative params
      Q = diag([1,5]);
      R = 1.0.*eye(m);
      
      % Terminal control law
      [P, ~, G] = dare(mpc.A, mpc.B, Q, R);
      K = -G;
      Qf = P;
      
%       % State constraints: Fx <= f
%       F = [0 1 0 0; 0 -1 0 0];
%       f = [1; 1].*0.035;
      
      % Input constraints: HKx <= h
      H = [1; -1];
      h = [0.3; 0.2];
      
      % Terminal invariant set
      Xf = Polyhedron(H*K, h);
      
      while true
          Xf_new = Polyhedron([Xf.A; Xf.A*(mpc.A+mpc.B*K)], [Xf.b; Xf.b]);  % Matrices should be updated
     
          if Xf_new == Xf
              break
          end
     
          Xf = Xf_new;
      end
      
      % Tracking
      deltx = x;
      deltu = u;
      
      for i = 1:n
          deltx(i, :) = x(i, :) - xs(i, 1);
      end
      
      for i = 1:m
          deltu(i, :) = u(i, :) - us(i, 1);
      end
      
      % Why this (F*x <= f M*u <= m) does not work...
%       Fx = F*x;
      Hu = H*deltu;
      Hus = H*us;
      con = [Hu(1, :) <= h(1)-Hus(1) Hu(2, :) <= h(2)-Hus(2) ...
%              Xf.A*deltx(:, N) <= Xf.b ...
             deltx(:, 2:N) == mpc.A*deltx(:, 1:N-1) + mpc.B*deltu(:, 1:N-1)];
      
      % Objective function
      xQx = diag(deltx(:, 1:N-1)'*Q*deltx(:, 1:N-1));
      uRu = diag(deltu'*R*deltu);
      
      obj = sum(xQx) + sum(uRu) + deltx(:, N)'*Qf*deltx(:, N);
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us, d_est}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      %   d_est  - disturbance estimate
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.3)
      ref = sdpvar;
            
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
      out_n = size(mpc.C, 1);
      m = size(mpc.B, 2);
      
      Rs = 1;
      
      % Input constraints: HKx <= h
      H = [1; -1];
      h = [0.3; 0.2];
      
%       con = [[mpc.A-eye(n) mpc.B; mpc.C 0]*[xs; us] == [zeros(n, 1); ref] ... 
%              H*us <= h];
      con = [[mpc.A-eye(n) mpc.B; mpc.C 0]*[xs; us] == [-mpc.B*d_est; ref - zeros(out_n, m)*d_est] ... 
             H*us <= h];
      
      obj = us'*Rs*us;

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
    end
    
    
    % Compute augmented system and estimator gain for input disturbance rejection
    function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
      
      %%% Design the matrices A_bar, B_bar, L, and C_bar
      %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
      %%% converges to the correct state and constant input disturbance
      %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
      [n,m] = size(mpc.B);
      out_n = size(mpc.C, 1);
      
      A_bar = [mpc.A mpc.B; zeros(m, n) eye(m)];
      B_bar = [mpc.B; zeros(m ,m)];
      C_bar = [mpc.C zeros(out_n, m)];
      
      F = rand(n+m, 1)./5 + 0.1;
      
      L = -place(A_bar', C_bar', F)';
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    
  end
end
