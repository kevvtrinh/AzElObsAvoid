function handles = plotTrajectory(result, axesHandle, diagnosis)
%% Section 0: Header & Readme
% SYNTAX
%   handles = obstacleAvoidance.plotting.plotTrajectory(result)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, axesHandle)
%
% PURPOSE
%   - Plot only the obstacle geometry, visibility graph, selected route, and
%     BMTP motion stored in an empty-core planner result.
%
% INPUTS
%   - result: output from planner.
%   - axesHandle (optional): target Cartesian axes.
%
% OUTPUTS
%   - handles: axes and graphics handles created by this function.
%
% UNITS
%   - Both plotted axes use coordinate units.

%% Section 1: Validate The Result And Axes

requiredFields = {'PreparedObstacles', 'VisibilityGraph', 'Route_units', ...
    'position_units', 'Inputs', 'Limits', 'Success', 'TerminationReason'};
if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, requiredFields))
    error("plotTrajectory:InvalidResult", "result must be a complete empty-core planner result.");
end
if nargin < 2 || isempty(axesHandle) || isstruct(axesHandle)
    visible = "on";
    if nargin >= 2 && isstruct(axesHandle) && isfield(axesHandle,'FigureVisible'), visible = axesHandle.FigureVisible; end
    figureHandle = figure("Name", "BMTP core", "Visible", visible);
    axesHandle = axes("Parent", figureHandle);
elseif ~isgraphics(axesHandle, "axes")
    error("plotTrajectory:InvalidAxes", "axesHandle must be Cartesian axes.");
end

%% Section 2: Draw Stored Geometry And Graph

cla(axesHandle);
hold(axesHandle, "on");
originalObstacle = gobjects(0, 1);
protectedObstacle = gobjects(0, 1);
for obstacleIndex = 1:numel(result.PreparedObstacles)
    prepared = result.PreparedObstacles(obstacleIndex);
    original = [prepared.originalX_units{1}, prepared.originalY_units{1}];
    original = [original; original(1,:)];
    [~,geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(prepared,result.Inputs.initialState.time_s,true);
    protected = [geometry.x_units,geometry.y_units];
    if isempty(protected), continue; end
    protected = [protected;protected(1,:)];
    protectedObstacle(end + 1, 1) = patch(axesHandle, protected(:, 1), protected(:, 2), [0.96 0.82 0.82], "EdgeColor", [0.70 0.18 0.18], "LineWidth", 1.25, "DisplayName", "protected obstacle"); %#ok<AGROW>
    originalObstacle(end + 1, 1) = plot(axesHandle, original(:, 1), original(:, 2), "-", "Color", [0.25 0.25 0.25], "LineWidth", 1.0, "DisplayName", "original obstacle"); %#ok<AGROW>
end
graphHandle = gobjects(0, 1);
nodes_units = result.VisibilityGraph.NodePosition_units;
edges = result.VisibilityGraph.AcceptedNodeIndex;
for edgeIndex = 1:size(edges, 1)
    points_units = nodes_units(edges(edgeIndex, :), :);
    graphHandle(end + 1, 1) = plot(axesHandle, points_units(:, 1), points_units(:, 2), "-", "Color", [0.72 0.82 0.92], "LineWidth", 0.75, "HandleVisibility", "off"); %#ok<AGROW>
end
nodeHandle = plot(axesHandle, nodes_units(:, 1), nodes_units(:, 2), ".", "Color", [0.18 0.42 0.67], "MarkerSize", 10, "DisplayName", "visibility node");

%% Section 3: Draw Stored Route And Motion

routeHandle = gobjects(0, 1);
if ~isempty(result.Route_units)
    routeHandle = plot(axesHandle, result.Route_units(:, 1), result.Route_units(:, 2), "--", "Color", [0.15 0.52 0.25], "LineWidth", 1.5, "DisplayName", "planner guide");
end
trajectoryHandle = gobjects(0, 1);
if ~isempty(result.position_units)
    trajectoryHandle = plot(axesHandle, result.position_units(:, 1), result.position_units(:, 2), "-", "Color", [0.05 0.22 0.55], "LineWidth", 2.25, "DisplayName", "BMTP trajectory");
end
endpoint_units = [result.Inputs.initialState.position_units; result.Inputs.goalState.position_units];
endpointHandle = plot(axesHandle, endpoint_units(:, 1), endpoint_units(:, 2), "o", "MarkerFaceColor", [1 1 1], "MarkerEdgeColor", [0.05 0.05 0.05], "LineWidth", 1.25, "DisplayName", "endpoints");
axis(axesHandle, "equal");
xlim(axesHandle, result.Limits.xInterval_units);
ylim(axesHandle, result.Limits.yInterval_units);
grid(axesHandle, "on");
xlabel(axesHandle, "x (units)");
ylabel(axesHandle, "y (units)");
title(axesHandle, "BMTP: " + string(result.TerminationReason), "Interpreter", "none");

%% Section 4: Return The Created Handles

handles = struct();
handles.Axes = axesHandle;
handles.OriginalObstacle = originalObstacle;
handles.ProtectedObstacle = protectedObstacle;
handles.VisibilityEdge = graphHandle;
handles.VisibilityNode = nodeHandle;
handles.Route = routeHandle;
handles.Trajectory = trajectoryHandle;
handles.Endpoint = endpointHandle;
end
