function drawState(sim, Deliverable, Nplots, SaveFile, QuadPlot)

fontname = 'Arial'; % 'Times New Roman' | 'Arial'
lcn = 'northwest'; % legend location: northwest | 'southoutside'

if ~isfield(sim, 't')
    % This is the plot of ode45 - convert it
    sim.t = sim.x;
    sim.x = sim.y;
    
    for i = 1:length(sim.t)
        [s(i).omega, s(i).theta, s(i).vel, s(i).pos] = parse_state(sim.x(:,i), QuadPlot);
        s(i).t = sim.t(i);
        s(i).u = Nplots;
        s(i).x = sim.x(:,i);
    end
    sim = s;
    Nplots = 10;
end

figurePath = figure;
set(figurePath,'position',[0 0 1200 500]);
clf; hold on;

%% I. Angles 
subplot(2,2,1);
hold on; grid on
plot([sim.t], [sim.theta]*180/pi, 'o-'); % ,'LineWidth',2
title('Angles')
legend('Roll', 'Pitch', 'Yaw', 'Location', lcn);
% ylabel('Degrees');
% newly added
ylabel('Degrees ({}^{\circ})');
xlabel('Time (s)');
set(gca,'FontName',fontname);

%% II. Thrust 
if isfield(sim, 'u')
    subplot(2,2,2);
    plot([sim.t], [sim.u], 's-');
    title('Thrust')
    legend('u1', 'u2', 'u3', 'u4', 'Location', lcn);
    % newly added
    ylabel('Force (N)');
    xlabel('Time (s)');
    set(gca,'FontName',fontname);
end

%% III. Linear velocity 
subplot(2,2,3);
hold on; grid on
plot([sim.t], [sim.vel], 'o-');
%       plot([0,max([sim.t])], [-0.25,-0.25], 'k-', 'linewidth', 2);
%       plot([0,max([sim.t])], [ 0.25, 0.25], 'k-', 'linewidth', 2);
title('Linear velocity')
legend('Velocity x', 'Velocity y', 'Velocity z' ,'Location', lcn);
% newly added
ylabel('Velocity (m/s)');
xlabel('Time (s)');
set(gca,'FontName',fontname);

%% IV. Position
subplot(2,2,4);
hold on; grid on
plot([sim.t], [sim.pos], 'o-');
if isfield(sim, 'ref')
    plot([sim.t], [sim.ref], 'k-');
end
% title('Position')
if isfield(sim, 'ref')
    legend('x', 'y', 'z', ...
        'Reference x', 'Reference y', 'Reference z' ,'Location', lcn);
    title('Quadcopter & Reference Position');
else
    legend('x', 'y', 'z' ,'Location', lcn);
    title('Quadcopter Position')
end
% newly added
ylabel('Position (m)');
xlabel('Time (s)');
set(gca,'FontName',fontname);

%% save as pdf or png for report
tightfig;

if SaveFile
    saveName = Deliverable + "_State";
    print(saveName,'-dpng');
    print(saveName,'-dpdf','-bestfit'); % '-bestfit' | '-fillpage'
    %print(saveName,'-dtiffn');
end

end

function [omega, theta, vel, pos] = parse_state(x,QuadPlot)
if nargout >= 1, omega = x(QuadPlot.ind.omega, :); end
if nargout >= 2, theta = x(QuadPlot.ind.theta, :); end
if nargout >= 3, vel = x(QuadPlot.ind.vel, :); end
if nargout >= 4, pos = x(QuadPlot.ind.pos, :); end
end