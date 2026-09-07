%% Section 0: Header & Readme
% SYNTAX
%   Pause planTrajectory on the line AFTER addRouteSearchGeometry returns.
%   At the debug command prompt (K>>), run: plotRouteSearchGeometry
% PURPOSE
%   Visualize the geometry added by addRouteSearchGeometry without
%   running the trajectory planner or optimizer. Requires this repository.
% INPUTS
%   Existing planningContext in the currently selected debug workspace.
%   This script reads that record directly; it does not recreate it.
% OUTPUTS
%   Leaves geometryFigure and plot variables in the workspace.
%   Left: protected obstacle snapshots, colored by sample time.
%   Right: routeSearchGeometry.shape and its cached edges, start, and goal.
% UNITS
%   Azimuth/elevation are degrees; obstacle history times are seconds.
%
% This is route-search geometry, not a path or a safety certificate. The
% sampled union does not prove continuous collision freedom. Dense histories
% may instead use a conservative convex envelope per obstacle. Completed
% trajectories still require independent time-dependent validation.

%% Section 1: Controls
figureVisibility          = 'on'; % Use 'off' for headless inspection.
maximumDisplayedSnapshots = 25; % Display only; the context retains all samples.

% Make the package available when this script is run from another folder.
addpath(fileparts(mfilename('fullpath')));

%% Section 2: Require The Paused Planner Geometry
assert(exist('planningContext', 'var') == 1 && isstruct(planningContext) && isfield(planningContext, 'routeSearchGeometry'), 'plotRouteSearchGeometry:MissingPlanningContext', ['Pause after addRouteSearchGeometry returns, then run this script ' 'in the planTrajectory debug workspace.']);
routeSearchGeometry = planningContext.routeSearchGeometry;

%% Section 3: Plot Sampled Obstacles And Route-Search Boundary
geometryFigure = figure('Name', 'Route-search geometry', 'Color', 'w', 'Visible', figureVisibility);
geometryLayout = tiledlayout(geometryFigure, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
snapshotAxes   = nexttile(geometryLayout);
hold(snapshotAxes, 'on');
snapshotIndices = unique(round(linspace(1, numel(routeSearchGeometry.sampleTimes_s), min(maximumDisplayedSnapshots, numel(routeSearchGeometry.sampleTimes_s)))));
timeColors      = parula(256);
% Process each sample in temporal order and accumulate its result.
for sampleIndex = snapshotIndices
    sampleTime_s = routeSearchGeometry.sampleTimes_s(sampleIndex);
    timeFraction = (sampleTime_s - planningContext.startTime_s) / (planningContext.endTime_s - planningContext.startTime_s);
    colorIndex   = 1 + round(255 * timeFraction);
    % Evaluate each obstacle against the current geometry or motion.
    for obstacleIndex = 1:numel(planningContext.preparedObstacles)
        snapshotShape = obstacleAvoidance.obstacles.preparedShapeAtTime(planningContext.preparedObstacles(obstacleIndex), sampleTime_s);
        if ~isempty(snapshotShape.Vertices)
            plot(snapshotAxes, snapshotShape, 'FaceColor', timeColors(colorIndex, :), 'FaceAlpha', 0.08, 'EdgeColor', timeColors(colorIndex, :));
        end
    end
end
colormap(snapshotAxes, timeColors);
clim(snapshotAxes, [planningContext.startTime_s planningContext.endTime_s]);
timeColorbar = colorbar(snapshotAxes);
timeColorbar.Label.String = 'Sample time (s)';
title(snapshotAxes, sprintf('Protected snapshots: %d of %d times', numel(snapshotIndices), numel(routeSearchGeometry.sampleTimes_s)));

routeSearchAxes = nexttile(geometryLayout);
hold(routeSearchAxes, 'on');
if ~isempty(routeSearchGeometry.shape.Vertices)
    plot(routeSearchAxes, routeSearchGeometry.shape, 'FaceColor', [0.9 0.4 0.2], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end
% NaN separates cached edges so disjoint boundaries are not joined.
edgeX_deg = [routeSearchGeometry.edgeStart_deg(:, 1), routeSearchGeometry.edgeEnd_deg(:, 1), ...
    nan(size(routeSearchGeometry.edgeStart_deg, 1), 1)].';
edgeY_deg = [routeSearchGeometry.edgeStart_deg(:, 2), routeSearchGeometry.edgeEnd_deg(:, 2), ...
    nan(size(routeSearchGeometry.edgeStart_deg, 1), 1)].';
plot(routeSearchAxes, edgeX_deg(:), edgeY_deg(:), 'k.-', 'LineWidth', 1);
title(routeSearchAxes, routeSearchGeometry.representation, 'Interpreter', 'none');
% Process each plot axes needed to draw route search geometry.
for plotAxes = [snapshotAxes routeSearchAxes]
    plot(plotAxes, routeSearchGeometry.start_deg(1), routeSearchGeometry.start_deg(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    text(plotAxes, routeSearchGeometry.start_deg(1), routeSearchGeometry.start_deg(2), '  Start');
    plot(plotAxes, routeSearchGeometry.goal_deg(1), routeSearchGeometry.goal_deg(2), 'mp', 'MarkerFaceColor', 'm', 'MarkerSize', 10);
    text(plotAxes, routeSearchGeometry.goal_deg(1), routeSearchGeometry.goal_deg(2), '  Goal');
    axis(plotAxes, 'equal');
    grid(plotAxes, 'on');
    xlabel(plotAxes, 'Azimuth (deg)');
    ylabel(plotAxes, 'Elevation (deg)');
end
linkaxes([snapshotAxes routeSearchAxes], 'xy');
title(geometryLayout, 'Route-search geometry (not a validated trajectory)');

drawnow; % Render immediately while the planner remains paused.

%% Section 4: Report The Representation And Work Estimate
fprintf('Representation: %s\n', routeSearchGeometry.representation);
fprintf('Sample times: %d; displayed: %d; boundary edges: %d\n', numel(routeSearchGeometry.sampleTimes_s), numel(snapshotIndices), size(routeSearchGeometry.edgeStart_deg, 1));
fprintf('Estimated vertex work: %g; envelope threshold: %g\n', routeSearchGeometry.estimatedVertexWork, routeSearchGeometry.vertexWorkBudget);
disp(planningContext);
