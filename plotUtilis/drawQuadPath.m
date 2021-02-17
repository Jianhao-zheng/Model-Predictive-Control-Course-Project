function drawQuadPath(sim, Deliverable, Nplots, SaveFile, QuadPlot)

% Plot the trajectory of the quad
figurePath = figure;
set(figurePath,'position',[0 0 1000 700]);
clf; view(3); % 2: top view | 3: orthographic view 
hold on; grid on;

%% plot trajectories and reference
pos = [sim.pos];
plot3(pos(1,:), pos(2,:), pos(3,:), 'k', 'linewidth', 2);

if isfield(sim, 'ref')
    ref = [sim.ref];
    plot3(ref(1,:), ref(2,:), ref(3,:), 'color', 0.7*[1 1 1], 'linewidth', 2);
end

%% plot Nplots quads along the path
I = linspace(1, length(sim), Nplots);
I = unique(ceil(I));

for i = 1:length(I)
    plot_point(sim(I(i)).x, sim(I(i)).u, QuadPlot);
    hold on
end
axis equal
% axis vis3d

%% save as pdf or png for report
if SaveFile
    saveName = Deliverable + "_QuadPath";
    print(saveName,'-dpng');
    print(saveName,'-dpdf','-bestfit'); % '-bestfit' | '-fillpage'
    %print(saveName,'-dtiffn');
end

end

% Plot the quad at a given state and input
function plot_point(x, u, QuadPlot)

[omega, theta, vel, pos] = parse_state(x, QuadPlot);

% Rotation from body to interial frame
roll = theta(1); pitch = theta(2); yaw = theta(3);
R = [1 0 0;0 cos(roll) -sin(roll);0 sin(roll) cos(roll)];
R = R*[cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
R = R*[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];

[X,Y,Z] = sphere;
X = QuadPlot.rad * X + pos(1);
Y = QuadPlot.rad * Y + pos(2);
Z = QuadPlot.rad * Z + pos(3);
h=surf(X,Y,Z);
shading interp
set(h,'facecolor','b','linestyle','none');
lighting gouraud
hold on

% Draw the blades
L = R*QuadPlot.L;
plot3(L(1,:)+pos(1),L(2,:)+pos(2),L(3,:)+pos(3),'.','markersize',30);
for i = 1:4
    plot3([0;L(1,i)]+pos(1),...
        [0;L(2,i)]+pos(2),[0;L(3,i)]+pos(3),'k-','linewidth',3)
    
    th = linspace(-pi,pi,20);
    t = R*(QuadPlot.bladeRad*[sin(th);cos(th);0*th] + QuadPlot.L(:,i)*ones(1,20));
    t = t + pos*ones(1,20);
    plot3(t(1,:),t(2,:),t(3,:),'k');
end

% Plot the forces
for i = 1:4
    thrustDir = QuadPlot.thrustDir(:,i);
    t = thrustDir / norm(thrustDir) * u(i) / QuadPlot.thrustLimits(2,i) * norm(QuadPlot.L(:,1));
    t = R*t;
    plot3([0;t(1)]+L(1,i)+pos(1),[0;t(2)]+L(2,i)+pos(2),...
        [0;t(3)]+L(3,i)+pos(3),'r-','linewidth',3);
end
end

function [omega, theta, vel, pos] = parse_state(x,QuadPlot)
if nargout >= 1, omega = x(QuadPlot.ind.omega, :); end
if nargout >= 2, theta = x(QuadPlot.ind.theta, :); end
if nargout >= 3, vel = x(QuadPlot.ind.vel, :); end
if nargout >= 4, pos = x(QuadPlot.ind.pos, :); end
end