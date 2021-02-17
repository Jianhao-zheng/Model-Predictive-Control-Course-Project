classdef MPC_Control
  properties
    ctrl_opt % YALMIP object to compute control law
    target_opt % YALMIP object to compute steady-state target
    
    % Discrete-time system matrices
    A, B, C, D
  end
  
  methods
    function mpc = MPC_Control(sys, Ts)
      
      % Discretize the system and extract the A,B,C,D matrices
      sys_d = c2d(sys, Ts);
      [mpc.A,mpc.B,mpc.C,mpc.D] = ssdata(sys_d);
      
      mpc.target_opt = mpc.setup_steady_state_target();
      mpc.ctrl_opt = mpc.setup_controller();
    end
    
    % Compute the MPC controller
    function u = get_u(mpc, x, ref)
      % Compute the target
      if nargin < 3
        ref_x = zeros(size(mpc.A,1),1);
        ref_u = 0;
      else
        if length(struct(mpc.target_opt).diminOrig) == 2
          if length(x) == 3
            d_est = x(end);
          else
            d_est = 0;
          end
          [target, isfeasible] = mpc.target_opt(ref, d_est);
        else
          [target, isfeasible] = mpc.target_opt(ref);
        end
        [ref_x, ref_u] = deal(target{:});
        assert(isfeasible==0, 'isfeasible in target computationn');
      end
      
      % Compute the control action
      if length(struct(mpc.ctrl_opt).diminOrig) == 4
        if length(x) == 3
          d_est = x(end);
          x = x(1:end-1);
        else
          d_est = 0;
        end
        [u, isfeasible] = mpc.ctrl_opt({x, ref_x, ref_u, d_est});        
      else
        [u, isfeasible] = mpc.ctrl_opt({x, ref_x, ref_u});
      end
      assert(isfeasible==0, 'isfeasible in control computationn');
    end
  end
  
  methods (Abstract)
    setup_controller(mpc)
    setup_steady_state_target(mpc)
  end
end
