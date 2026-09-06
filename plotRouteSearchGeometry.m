%% Section 0: Header & Readme
% SYNTAX
%   Pause planTrajectory on the line AFTER createRouteSearchGeometry returns.
%   At the debug command prompt (K>>), run: plotRouteSearchGeometry
% PURPOSE
%   Visualize the geometry returned by createRouteSearchGeometry without
%   running the trajectory planner or optimizer. Requires this repository.
% INPUTS
%   Existing proposal and scene in the currently selected debug workspace.
%   This script reads those values directly; it does not recreate them.
% OUTPUTS
%   Leaves geometryFigure and plot variables in the workspace.
%   Left: protected obstacle snapshots, colored by sample time.
%   Right: proposal.shape and its cached boundary edges, start, and goal.
% UNITS
%   Azimuth/elevation are degrees; obstacle history times are seconds.
%
% This is route-search geometry, not a path or a safety certificate. The
% sampled union does not prove continuous collision freedom. Dense histories
% may instead use a conservative convex envelope per obstacle. Completed
% trajectories still require independent time-dependent validation.

%% Section 1: Controls
figureVisibility          = 'on'; % Use 'off' for headless inspection.
maximumDisplayedSnapshots = 25; % Display only; proposal retains all samples.

% Make the package available when this script is run from another folder.
addpath(fileparts(mfilename('fullpath')));

%% Section 2: Require The Paused Planner Geometry
assert(exist('proposal', 'var') == 1 && isstruct(proposal) && isfield(proposal, 'shape'), 'plotRouteSearchGeometry:MissingProposal', ['Pause after createRouteSearchGeometry returns, then run this script ' 'in the planTrajectory debug workspace.']);
assert(exist('scene', 'var') == 1 && isstruct(scene) && isfield(scene, 'preparedObstacles'), 'plotRouteSearchGeometry:MissingScene', 'The prepared scene must be available in the same debug workspace.');

%% Section 3: Plot Sampled Obstacles And Proposal Boundary
geometryFigure = figure('Name', 'Route-search geometry', 'Color', 'w', 'Visible', figureVisibility);
geometryLayout = tiledlayout(geometryFigure, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
snapshotAxes   = nexttile(geometryLayout);
hold(snapshotAxes, 'on');
snapshotIndices = unique(round(linspace(1, numel(proposal.sampleTimes_s), min(maximumDisplayedSnapshots, numel(proposal.sampleTimes_s)))));
timeColors      = parula(256);
% Process each sample in temporal order and accumulate its result.
for sampleIndex = snapshotIndices
    sampleTime_s = proposal.sampleTimes_s(sampleIndex);
    timeFraction = (sampleTime_s - scene.startTime_s) / (scene.endTime_s - scene.startTime_s);
    colorIndex   = 1 + round(255 * timeFraction);
    % Evaluate each obstacle against the current geometry or motion.
    for obstacleIndex = 1:numel(scene.preparedObstacles)
        snapshotShape = obstacleAvoidance.obstacles.preparedShapeAtTime(scene.preparedObstacles(obstacleIndex), sampleTime_s);
        if ~isempty(snapshotShape.Vertices)
            plot(snapshotAxes, snapshotShape, 'FaceColor', timeColors(colorIndex, :), 'FaceAlpha', 0.08, 'EdgeColor', timeColors(colorIndex, :));
        end
    end
end
colormap(snapshotAxes, timeColors);
clim(snapshotAxes, [scene.startTime_s scene.endTime_s]);
timeColorbar = colorbar(snapshotAxes);
timeColorbar.Label.String = 'Sample time (s)';
title(snapshotAxes, sprintf('Protected snapshots: %d of %d times', numel(snapshotIndices), numel(proposal.sampleTimes_s)));

proposalAxes = nexttile(geometryLayout);
hold(proposalAxes, 'on');
if ~isempty(proposal.shape.Vertices)
    plot(proposalAxes, proposal.shape, 'FaceColor', [0.9 0.4 0.2], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end
% NaN separates cached edges so disjoint boundaries are not joined.
edgeX_deg = [proposal.edgeStart_deg(:, 1), proposal.edgeEnd_deg(:, 1), ...
    nan(size(proposal.edgeStart_deg, 1), 1)].';
edgeY_deg = [proposal.edgeStart_deg(:, 2), proposal.edgeEnd_deg(:, 2), ...
    nan(size(proposal.edgeStart_deg, 1), 1)].';
plot(proposalAxes, edgeX_deg(:), edgeY_deg(:), 'k.-', 'LineWidth', 1);
title(proposalAxes, proposal.representation, 'Interpreter', 'none');
% Process each plot axes needed to draw route search geometry.
for plotAxes = [snapshotAxes proposalAxes]
    plot(plotAxes, proposal.start_deg(1), proposal.start_deg(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    text(plotAxes, proposal.start_deg(1), proposal.start_deg(2), '  Start');
    plot(plotAxes, proposal.goal_deg(1), proposal.goal_deg(2), 'mp', 'MarkerFaceColor', 'm', 'MarkerSize', 10);
    text(plotAxes, proposal.goal_deg(1), proposal.goal_deg(2), '  Goal');
    axis(plotAxes, 'equal');
    grid(plotAxes, 'on');
    xlabel(plotAxes, 'Azimuth (deg)');
    ylabel(plotAxes, 'Elevation (deg)');
end
linkaxes([snapshotAxes proposalAxes], 'xy');
title(geometryLayout, 'Route-search proposal (not a validated trajectory)');

drawnow; % Render immediately while the planner remains paused.

%% Section 4: Report The Representation And Work Estimate
fprintf('Representation: %s\n', proposal.representation);
fprintf('Sample times: %d; displayed: %d; boundary edges: %d\n', numel(proposal.sampleTimes_s), numel(snapshotIndices), size(proposal.edgeStart_deg, 1));
fprintf('Estimated vertex work: %g; envelope threshold: %g\n', proposal.estimatedVertexWork, proposal.vertexWorkBudget);
disp(proposal);
