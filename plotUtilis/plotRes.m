function plotRes(sim, Deliverable, Nplots, SaveFile)

if nargin < 3, Nplots = 10; end
if nargin < 4, SaveFile = false; end

% load parameters for plotting
QuadPlot = QuadPlotConst();

drawState(sim, Deliverable, Nplots, SaveFile, QuadPlot);
drawQuadPath(sim, Deliverable, Nplots, SaveFile, QuadPlot);
drawQuadPathTop(sim, Deliverable, Nplots, SaveFile, QuadPlot)

end

function QuadPlot = QuadPlotConst()
QuadPlot.rad = 0.0400;
QuadPlot.L   = [0.2    0 -0.2    0;...
                  0  0.2    0 -0.2;...
                  0    0    0    0];
QuadPlot.ind = struct('omega', [1:3], 'theta', [4:6], ...
                      'vel', [7:9], 'pos', [10:12]);
QuadPlot.bladeRad  = 0.0800;
QuadPlot.thrustDir = [0 0 0 0; 0 0 0 0; 1 1 1 1];
QuadPlot.thrustLimits = [0 0 0 0; 1 1 1 1] * 1.5;
end