classdef Quad
  
  properties (Hidden, Constant)
    % Physical properties and constraints
    thrustLimits = [0 0 0 0; 1 1 1 1] * 1.5;
    mass = 8;
    
    I = [10 0 0; 0 10 0; 0 0 15];
    K = [28 28 28 28; 0 5.6 0 -5.6; -5.6 0 5.6 0; 11 -11  11 -11];
    T = Quad.K;
    
    % Only used for plotting
    thrustDir = [0 0 0 0; 0 0 0 0; 1 1 1 1];
    L = [...
      0.2    0 -0.2    0;...
      0  0.2    0 -0.2;...
      0    0    0    0];
    
    nL = 0.2000;
    rad = 0.0400;
    bladeRad = 0.0800;
    
    % Indices into the states
    ind = struct('omega', [1:3], 'theta', [4:6], 'vel', [7:9], 'pos', [10:12]);
  end
  
  properties (Hidden)
    sys_discrete % Discrete time
    sys % Linearizations
    Ts % Sampling time
    xs, us % Trim point
  end
  
  methods
    
    function quad = Quad(Ts)
      if nargin < 1, Ts = 1/5; end
      
      try % Test installation
        import casadi.*
        x = casadi.SX.sym('x');
        
        x = sdpvar(2,1);
      catch
        error('Could not load casadi - check that it''s installed properly and on the path')
      end
      
      [quad.xs, quad.us] = quad.trim();
      quad.sys = quad.linearize(quad.xs, quad.us);
      quad.sys_discrete = c2d(quad.sys, Ts);
      quad.Ts = Ts;
    end
    
    %
    % Simulate the nonlinear quadcopter to track an MPC reference
    %
    function sim = sim(quad, ctrl_x, ctrl_y, ctrl_z, ctrl_yaw, input_bias)
      sim.t = 0;
      sim.x = zeros(12,1);
      sim.z_hat = zeros(3,1); % Offset free for z-dimension
      Ts = quad.Ts;
      Tf = 40;
      
      if nargin >= 3
        ctrl = quad.merge_controllers(ctrl_x, ctrl_y, ctrl_z, ctrl_yaw);
      else
        ctrl = ctrl_x;
        ctrl_z.L = [];
      end
      ref = @(t,x) quad.MPC_ref(t, Tf);
      
      if nargin < 6, input_bias = 0; end
      
      fprintf('Simulating...\n');
      tic
      for i = 1:ceil(Tf/Ts)
        if toc > 2
          tic
          fprintf('... %.2f of %.2f seconds\n', i * Ts, Tf);
        end
        
        % Compute reference
        sim(i).ref = ref(sim(i).t, sim(i).x);
        
        % Simulate forward in time
        sim(i+1).t = sim(i).t + Ts;
        
        % Compute control law
        if ~isempty(ctrl_z.L) % Doing offset free in the z-dimension
          
          % Compute control input for z-dimension
          z_input = ctrl_z.get_u(sim(i).z_hat, sim(i).ref(3));
          
          % Run the estimator
          sim(i+1).z_hat = ctrl_z.A_bar * sim(i).z_hat + ...
            ctrl_z.B_bar * z_input ...
            + ctrl_z.L * (ctrl_z.C_bar * sim(i).z_hat - sim(i).x(quad.ind.pos(3)));
          
          % Compute control input
          sim(i).u = ctrl(sim(i).x, sim(i).ref, sim(i).z_hat);
          
        else
          sim(i).u = ctrl(sim(i).x, sim(i).ref);
        end
        
        sim(i+1).x = quad.step(sim(i).x, sim(i).u + input_bias, Ts);
        
        [sim(i).omega, sim(i).theta, sim(i).vel, sim(i).pos] = ...
          quad.parse_state(sim(i).x);
      end
      sim(end) = [];
    end
    
    
    %
    % Trace out an MPC in ref_time seconds
    %
    function ref = MPC_ref(quad, t, ref_time)
      
      coords = [0 0; 0 2; 1 1; 2 2; 2 0; ... % 'M'
        3 0; 3 2; 4 2; 4 1; 3 1; 3 0; ... % 'P'
        7 0; 5 0; 5 2; 7 2]; % 'C'
      coords = coords / 2;
%       coords = [2 0; 2 0; 2 0; 2 0; 2 0; ... % 'M'
%         2 0; 2 0; 2 0; 2 0; 2 0; 2 0; ... % 'P'
%         2 0; 2 0; 2 0; 2 0]; % 'C'
      % coords = coords * 5;
      
      % Break the path into smaller steps
      % coords = [interp(coords(:,1), 5) interp(coords(:,2), 5)];
      
      for i = 1:size(coords,1)-1
        distance(i) = norm(coords(i,:) - coords(i+1,:));
      end
      
      distance = [0 (ref_time) * cumsum(distance) / sum(distance)];
      
      ind = min([sum(t >= distance) + 1, size(coords,1)]);
      x = coords(ind,:);
      x = [x t/ref_time]'; %2*sin(t/ref_time*2*pi)]';
      
      % % Rotate into a plane in R3
      % x = [coords(sum(t >= distance)+1,:) 0]';
      % u = [1;1;1]; u = u / norm(u);
      % th = 45/180*pi;
      % Rx = u*(u'*x) + cos(th)*cross(cross(u,x), u) + sin(th) * cross(u,x);
      
      ref = [x; 0*5/180*pi*sin(t/ref_time*2*pi)];
    end
    
    
    %
    % Compute trim point for flat and level flight
    %
    function [xs, us] = trim(quad)
      
      LB = [-inf*ones(12,1); quad.thrustLimits(1,:)'];
      UB = [ inf*ones(12,1); quad.thrustLimits(2,:)'];
      
      opt = optimoptions('fmincon','Algorithm','sqp');
      opt.Display = 'off';
      
      MSE = @(v) v'*v;
      [y, fval, exitflag] = fmincon(@(y) MSE(quad.f(y(1:12), y(13:16))), ...
        zeros(16, 1), ...
        [], [], [], [], LB, UB, [], opt);
      
      if exitflag < 0 || fval > 1e-3
        error('Could not find trim condition');
      end
      xs = y(1:12);
      us = y(13:16);
      
      xs(abs(xs) < 1e-6) = 0;
      us(abs(us) < 1e-6) = 0;
    end
    
    %
    % Simulate the system forward Ts seconds
    %
    function xp = step(quad, x, u, Ts)
      [~, yout] = ode45(@(t, x) quad.f(x, u), [0, Ts], x);
      xp = yout(end,:)';
      
    end
    
    %
    % Return a linearization of the quad around the
    % equilibrium point xs, us
    %
    function sys = linearize(quad, xs, us)
      if nargin < 2
        [xs,us] = quad.trim();
        fprintf('No equilibrium given... trimming\n');
      end
      
      x = casadi.SX.sym('x',12,1);
      u = casadi.SX.sym('u',4,1);
      f = quad.f(x,u);
      
      A = casadi.Function('A', {x,u}, {jacobian(f, x)});
      A = full(A(xs, us));
      B = casadi.Function('A', {x,u}, {jacobian(f, u)});
      B = full(B(xs, us));
      
      A(abs(A) < 1e-5) = 0;
      B(abs(B) < 1e-5) = 0;
      
      sys = ss(A,B,eye(12),zeros(12,4));
      sys.InputName = {'u1', 'u2', 'u3', 'u4'};
      sys.OutputName = {...
        'vel_roll', 'vel_pitch', 'vel_yaw', ...
        'roll', 'pitch', 'yaw',...
        'vel_x', 'vel_y', 'vel_z', ...
        'x','y','z'
        };
      sys.StateName = sys.OutputName;
    end
    
    %
    % Compute the quad dynamics
    %
    function dx = f(quad, x, u)
      
      if ~(isa(x,'casadi.SX') || isa(x,'casadi.MX'))
        for i = 1:4
          u(i) = max([quad.thrustLimits(1,i) ...
            min([u(i) quad.thrustLimits(2,i)])]);
        end
      end
      
      % u : four rotor speeds
      uTotal = quad.K(1,:) * u;
      uMoments = quad.K(2:end,:) * u;
      
      [omega, theta, xVel, xPos] = quad.parse_state(x);
      
      % Rotation from body to interial frame
      roll = theta(1); pitch = theta(2); yaw = theta(3);
      R = [1 0 0;0 cos(roll) -sin(roll);0 sin(roll) cos(roll)];
      R = R*[cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
      R = R*[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
      
      % Forces
      g = 9.81;
      vDot = -quad.mass*g*[0;0;1] + uTotal*R*[0;0;1];
      
      % Rotational derivative
      omegaDot = -cross(omega, quad.I*omega) + uMoments;
      
      % Compute state derivative
      dx = [quad.I \ omegaDot; omega; vDot / quad.mass; xVel];
    end
    
    %
    % Split the state into its parts
    %
    function [omega, theta, vel, pos] = parse_state(quad, x)
      if nargout >= 1, omega = x(quad.ind.omega, :); end
      if nargout >= 2, theta = x(quad.ind.theta, :); end
      if nargout >= 3, vel = x(quad.ind.vel, :); end
      if nargout >= 4, pos = x(quad.ind.pos, :); end
    end
    
    %
    % Decompose the quad copter into four systems around a hovering
    % equilibrium
    %
    function [sys_x, sys_y, sys_z, sys_yaw] = decompose(quad, sys, xs, us)
      
      if nargin < 2
        [xs,us] = quad.trim();
        sys = quad.linearize(xs, us);
      end
      [A,B,C,D] = ssdata(sys);
      
      % Split into four seperate systems
      ind = Quad.ind;
      stateNames = sys.StateName;
      
      I = [ind.omega(2) ind.theta(2) ind.vel(1) ind.pos(1)];
      T   = [-1 0 1 0];  % Pitch moment = x
      sys_x = ss(A(I,I), B(I,:)*T', C(ind.pos(1),I), 0);
      sys_x.StateName = {stateNames{I}};
      
      sys_x.UserData.states = I;
      sys_x.UserData.T      = T;
      sys_x.UserData.us     = T * us;
      
      I = [ind.omega(1) ind.theta(1) ind.vel(2) ind.pos(2)];
      T   = [0 1 0 -1];  % Roll moment = y
      sys_y = ss(A(I,I), B(I,:)*T', C(ind.pos(2),I), 0);
      sys_y.StateName = {stateNames{I}};
      
      sys_y.UserData.states = I;
      sys_y.UserData.T      = T;
      sys_y.UserData.us     = T * us;
      
      I = [ind.vel(3) ind.pos(3)];
      T   = [1 1 1 1];   % Total vertical force
      sys_z = ss(A(I,I), B(I,:)*T', C(ind.pos(3),I), 0);
      sys_z.StateName = {stateNames{I}};
      
      sys_z.UserData.states = I;
      sys_z.UserData.T      = T;
      sys_z.UserData.us     = T * us;
      
      I = [ind.omega(3) ind.theta(3)];
      T = [-1 1 -1 1]; % Yaw moment
      sys_yaw = ss(A(I,I), B(I,:)*T', C(ind.theta(3),I), 0);
      sys_yaw.StateName = {stateNames{I}};
      
      sys_yaw.UserData.states = I;
      sys_yaw.UserData.T      = T;
      sys_yaw.UserData.us     = T * us;
    end
    
    %
    % Combines the inputs from all four controllers
    %
    function ctrl = merge_controllers(quad, ctrl_x, ctrl_y, ctrl_z, ctrl_yaw)
      
      % Get the state indices
      [sys_x, sys_y, sys_z, sys_yaw] = quad.decompose();
      
      xI = sys_x.UserData.states;
      xT = sys_x.UserData.T;
      
      yI = sys_y.UserData.states;
      yT = sys_y.UserData.T;
      
      zI = sys_z.UserData.states;
      zT = sys_z.UserData.T;
      
      yawI = sys_yaw.UserData.states;
      yawT = sys_yaw.UserData.T;
      
      if ~isempty(ctrl_z.L) % Provide offset-free information in z
        fprintf('===> Detected offset-free z-controller\n');
        ctrl = @(x, ref, z_hat) quad.us + ...
          xT'*ctrl_x.get_u(x(xI), ref(1)) + ...
          yT'*ctrl_y.get_u(x(yI), ref(2)) + ...
          zT'*ctrl_z.get_u(z_hat, ref(3)) + ...
          yawT'*ctrl_yaw.get_u(x(yawI), ref(4));
      else % Not offset free
        fprintf('===> z-controller is not offset free\n');
        ctrl = @(x, ref) quad.us + ...
          xT'*ctrl_x.get_u(x(xI), ref(1)) + ...
          yT'*ctrl_y.get_u(x(yI), ref(2)) + ...
          zT'*ctrl_z.get_u([x(zI);0], ref(3)) + ...
          yawT'*ctrl_yaw.get_u(x(yawI), ref(4));
      end
    end
    
    %
    % Compute upper / lower bounds on the inputs of each sub-system
    %
    function dat = decomposition_bounds(quad, dat, us)
      T = [dat.x.T; dat.y.T; dat.z.T; dat.yaw.T];
      P = Polyhedron([T';-T'],[quad.thrustLimits(2,:)'-us; quad.thrustLimits(1,:)'+us]);
      
      V = Polyhedron('lb', -ones(4,1), 'ub', ones(4,1)).V;
      lb = sdpvar(4,1);
      ub = sdpvar(4,1);
      M = sdpvar(size(V,1), size(V,2));
      for i = 1:4
        M(V(:,i) < 0,i) = lb(i);
        M(V(:,i) > 0,i) = ub(i);
      end
      
      con = [P.A * M' <= repmat(P.b, 1, size(M,1))];
      con = con + [lb <= -0.2, ub >= 0.2];
      
      optimize(con, -(-sum(lb) + sum(ub)))
      
      dat.x.lb = double(lb(1)); dat.x.ub = double(ub(1));
      dat.y.lb = double(lb(2)); dat.y.ub = double(ub(2));
      dat.z.lb = double(lb(3)); dat.z.ub = double(ub(3));
      dat.yaw.lb = double(lb(4)); dat.yaw.ub = double(ub(4));
    end
    
    %
    % Plot the trajectory of the quad
    %
    function plot(quad, sim, Nplots)
      
      if ~isfield(sim, 't')
        % This is the plot of ode45 - convert it
        sim.t = sim.x;
        sim.x = sim.y;
        
        for i = 1:length(sim.t)
          [s(i).omega, s(i).theta, s(i).vel, s(i).pos] = quad.parse_state(sim.x(:,i));
          s(i).t = sim.t(i);
          s(i).u = Nplots;
          s(i).x = sim.x(:,i);
        end
        sim = s;
        Nplots = 10;
      end
      
      figure(1); clf; hold on;
      subplot(2,2,1);
      hold on; grid on
      plot([sim.t], [sim.theta]*180/pi, 'o-');
      title('Angles')
      legend('Roll', 'Pitch', 'Yaw');
      ylabel('Degrees');
      
      if isfield(sim, 'u')
        subplot(2,2,2);
        plot([sim.t], [sim.u], 's-');
        title('Thrust')
        legend('u1', 'u2', 'u3', 'u4');
      end
      
      subplot(2,2,3);
      hold on; grid on
      plot([sim.t], [sim.vel], 'o-');
      %       plot([0,max([sim.t])], [-0.25,-0.25], 'k-', 'linewidth', 2);
      %       plot([0,max([sim.t])], [ 0.25, 0.25], 'k-', 'linewidth', 2);
      title('Linear velocity')
      legend('Velocity x', 'Velocity y', 'Velocity z');
      
      subplot(2,2,4);
      hold on; grid on
      plot([sim.t], [sim.pos], 'o-');
      if isfield(sim, 'ref')
        plot([sim.t], [sim.ref], 'k-');
      end
      title('Position')
      if isfield(sim, 'ref')
        legend('x', 'y', 'z', ...
          'Reference x', 'Reference y', 'Reference z');
      else
        legend('x', 'y', 'z');
      end
      
      figure(2);
      clf; view(3);
      hold on; grid on;
      if nargin < 3, Nplots = 10; end
      
      pos = [sim.pos];
      plot3(pos(1,:), pos(2,:), pos(3,:), 'k', 'linewidth', 2);
      
      if isfield(sim, 'ref')
        ref = [sim.ref];
        plot3(ref(1,:), ref(2,:), ref(3,:), 'color', 0.7*[1 1 1], 'linewidth', 2);
      end
      
      % We plot Nplots quads along the path
      I = linspace(1, length(sim), Nplots);
      I = unique(ceil(I));
      
      for i = 1:length(I)
        quad.plot_point(sim(I(i)).x, sim(I(i)).u);
      end
      axis equal
      axis vis3d
    end
    
    
    % Plot the quad at a given state and input
    function plot_point(quad, x, u)
      
      [omega, theta, vel, pos] = quad.parse_state(x);
      
      % Rotation from body to interial frame
      roll = theta(1); pitch = theta(2); yaw = theta(3);
      R = [1 0 0;0 cos(roll) -sin(roll);0 sin(roll) cos(roll)];
      R = R*[cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
      R = R*[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
      
      [X,Y,Z] = sphere;
      X = quad.rad * X + pos(1);
      Y = quad.rad * Y + pos(2);
      Z = quad.rad * Z + pos(3);
      h=surf(X,Y,Z);
      shading interp
      set(h,'facecolor','b','linestyle','none');
      lighting gouraud
      hold on
      
      % Draw the blades
      L = R*quad.L;
      plot3(L(1,:)+pos(1),L(2,:)+pos(2),L(3,:)+pos(3),'.','markersize',30);
      for i = 1:4
        plot3([0;L(1,i)]+pos(1),...
          [0;L(2,i)]+pos(2),[0;L(3,i)]+pos(3),'k-','linewidth',3)
        
        th = linspace(-pi,pi,20);
        t = R*(quad.bladeRad*[sin(th);cos(th);0*th] + quad.L(:,i)*ones(1,20));
        t = t + pos*ones(1,20);
        plot3(t(1,:),t(2,:),t(3,:),'k');
      end
      
      % Plot the forces
      for i = 1:4
        thrustDir = quad.thrustDir(:,i);
        t = thrustDir / norm(thrustDir) * u(i) / quad.thrustLimits(2,i) * norm(quad.L(:,1));
        t = R*t;
        plot3([0;t(1)]+L(1,i)+pos(1),[0;t(2)]+L(2,i)+pos(2),...
          [0;t(3)]+L(3,i)+pos(3),'r-','linewidth',3);
      end
    end
  end
  
  methods (Static)
    function pos = pos(x)
      pos = x(Quad.ind.pos, :);
    end
    function vel = vel(x)
      vel = x(Quad.ind.vel, :);
    end
    function theta = theta(x)
      theta = x(Quad.ind.theta, :);
    end
    function omega = omega(x)
      omega = x(Quad.ind.omega, :);
    end
  end
  
end
