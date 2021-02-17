classdef MPC_Control_y < MPC_Control
  
  methods
    function mpc = MPC_Control_y(sys, Ts)
      mpc = mpc@MPC_Control(sys, Ts);
    end
    
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % SET THE HORIZON HERE
      N = 20;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      
      % Tentative params
      Q = 1.0.*eye(n);
      R = 1.0.*eye(m);
      
      % Terminal control law
      [P, ~, G] = dare(mpc.A, mpc.B, Q, R);
      K = -G;
      Qf = P;
      
      % State constraints: Fx <= f
      F = [0 1 0 0; 0 -1 0 0];
      f = [1; 1].*0.035;
      
      % Input constraints: HKx <= h
      H = [1; -1];
      h = [1; 1].*0.3;
      
      % Terminal invariant set
      Xf = Polyhedron([F; H*K], [f; h]);
      
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
      
      % Why this (F*x <= f H*u <= h) does not work...
      Fx = F*deltx;
      Fxs = F*xs;
      Hu = H*deltu;
      Hus = H*us;
      con = [Fx(1, :) <= f(1)-Fxs(1) Fx(2, :) <= f(2)-Fxs(2) ...
             Hu(1, :) <= h(1)-Hus(1) Hu(2, :) <= h(2)-Hus(2) ...
             Xf.A*deltx(:, N) <= Xf.b ...
             deltx(:, 2:N) == mpc.A*deltx(:, 1:N-1) + mpc.B*deltu(:, 1:N-1)];
      
      % Objective function
      xQx = diag(deltx(:, 1:N-1)'*Q*deltx(:, 1:N-1));
      uRu = diag(deltu'*R*deltu);
      
      obj = sum(xQx) + sum(uRu) + deltx(:, N)'*Qf*deltx(:, N);

      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
      Rs = 1;
      
      % State constraints: Fx <= f
      F = [0 1 0 0; 0 -1 0 0];
      f = [1; 1].*0.035;
      
      % Input constraints: HKx <= h
      H = [1; -1];
      h = [1; 1].*0.3;
      
      con = [[mpc.A-eye(n) mpc.B; mpc.C 0]*[xs; us] == [zeros(n, 1); ref] ... 
             F*xs <= f H*us <= h];
      
      obj = us'*Rs*us;
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
