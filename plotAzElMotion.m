function handles = plotAzElMotion(result, diagnostics, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = plotAzElMotion()
%   handles = plotAzElMotion(result, diagnostics)
%   handles = plotAzElMotion(result, diagnostics, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot obstacle geometry, candidate routes, the selected motion,
%     time-stacked visibility graphs, and kinematic histories.
%**************************************************************************
% INPUTS
%   - result (scalar planner-result struct)
%       Compact first output from planAzElMotion.
%   - diagnostics (scalar planner-diagnostics struct)
%       Optional second output from planAzElMotion or an intercept wrapper.
%   - optionOverrides (scalar struct, optional; default struct())
%       FigureVisible, Title, ShowAnimation, FrameStride, Pause_s,
%       ShowVisibilityGraphs, ShowSweptSurfaces, and display limits.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Workspace, graph-inspector, kinematic, and animation handles.
%       A zero-input call returns plot defaults.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************
defaults = struct("FigureVisible", "on", "Title", "Az/El motion plan", ...
    "ShowAnimation", true, "ShowKinematics", true, ...
    "ShowVisibilityGraphs", true, ...
    "FrameStride", 10, "Pause_s", 0.001, ...
    "ShowSweptSurfaces", true, ...
    "MaximumDisplayedSlicesPerObstacle", 30, ...
    "MaximumDisplayedVisibilitySnapshots", 30);
if nargin == 0
    handles = defaults;
    return;
end
if nargin < 2 || ~isstruct(diagnostics) || ~isscalar(diagnostics)
    error("plotAzElMotion:InvalidDiagnostics", ...
        "diagnostics must be the second planner output.");
end
if nargin < 3 || isempty(optionOverrides)
    optionOverrides = struct();
end
options = resolvePlotOptions(defaults, optionOverrides);
required = ["Success" "selectedRoute_deg" "initialState" "goalState"];
if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, required))
    error("plotAzElMotion:InvalidResult", ...
        "result must be the scalar output from planAzElMotion.");
end
[plannerDiagnostics, targetTrack] = ...
    plotDiagnosticViews(diagnostics);
requiredDiagnostics = ["OriginalObstacleData" "ProtectedObstacleData" ...
    "ObstacleField" "Search" "TimedPath"];
if ~all(isfield(plannerDiagnostics, requiredDiagnostics))
    error("plotAzElMotion:InvalidDiagnostics", ...
        "diagnostics does not contain the required planner records.");
end

%% Section 1: Plot Workspace & Routes

workspaceFigure = figure("Visible", options.FigureVisible, ...
    "Name", options.Title);
workspaceAxes = axes(workspaceFigure);
hold(workspaceAxes, "on");
plotObstacleHistory(workspaceAxes, ...
    plannerDiagnostics.OriginalObstacleData, ...
    [0.75 0.15 0.15], "-", "Original obstacle");
plotObstacleHistory(workspaceAxes, ...
    plannerDiagnostics.ProtectedObstacleData, ...
    [0.90 0.45 0.05], "--", "Protected obstacle");
candidateSeeds = plannerDiagnostics.Search.CandidateSeeds;
for routeIndex = 1:numel(candidateSeeds)
    route_deg = candidateSeeds(routeIndex).Route_deg;
    plot(workspaceAxes, route_deg(:, 1), route_deg(:, 2), ...
        "Color", [0.75 0.75 0.75], "HandleVisibility", "off");
end
plot(workspaceAxes, result.selectedRoute_deg(:, 1), ...
    result.selectedRoute_deg(:, 2), "--", "Color", [0.15 0.35 0.8], ...
    "LineWidth", 1.2, "DisplayName", "Selected geometric route");
timedPath = plannerDiagnostics.TimedPath;
if ~isempty(timedPath.position_deg)
    plot(workspaceAxes, timedPath.position_deg(:, 1), ...
        timedPath.position_deg(:, 2), "k-", "LineWidth", 2, ...
        "DisplayName", "Timed motion");
end
if ~isempty(targetTrack.position_deg)
    plot(workspaceAxes, targetTrack.position_deg(:, 1), ...
        targetTrack.position_deg(:, 2), "-.", ...
        "Color", [0.65 0.10 0.65], "LineWidth", 1.4, ...
        "DisplayName", "Target track");
end
plot(workspaceAxes, result.initialState.position_deg(1), ...
    result.initialState.position_deg(2), "go", "MarkerFaceColor", "g", ...
    "DisplayName", "Start");
plot(workspaceAxes, result.goalState.position_deg(1), ...
    result.goalState.position_deg(2), "ro", "MarkerFaceColor", "r", ...
    "DisplayName", "Goal");
xlabel(workspaceAxes, "Azimuth (deg)");
ylabel(workspaceAxes, "Elevation (deg)");
title(workspaceAxes, options.Title + " - " + result.TerminationReason);
axis(workspaceAxes, "equal");
grid(workspaceAxes, "on");
box(workspaceAxes, "on");
legend(workspaceAxes, "Location", "best");

%% Section 2: Plot Time-Stacked Visibility Graphs

visibilityGraphs = struct();
if options.ShowVisibilityGraphs
    visibilityGraphs = createVisibilityGraphInspector( ...
        plannerDiagnostics, options);
end

%% Section 3: Plot Kinematics

kinematicsFigure = gobjects(0, 1);
kinematicAxes = gobjects(4, 1);
if options.ShowKinematics
    kinematicsFigure = figure("Visible", options.FigureVisible, ...
        "Name", options.Title + " kinematics");
    layout = tiledlayout(kinematicsFigure, 4, 1, ...
        "TileSpacing", "compact", "Padding", "compact");
    quantities = {"position_deg", "velocity_deg_s", ...
        "acceleration_deg_s2", "jerk_deg_s3"};
    labels = ["Position (deg)" "Velocity (deg/s)" ...
        "Acceleration (deg/s^2)" "Jerk (deg/s^3)"];
    for quantityIndex = 1:4
        kinematicAxes(quantityIndex) = nexttile(layout);
        values = timedPath.(quantities{quantityIndex});
        hasFiniteValue = ~isempty(values) && any(isfinite(values), "all");
        if ~isempty(timedPath.time_s) && hasFiniteValue
            plot(kinematicAxes(quantityIndex), timedPath.time_s, values, ...
                "LineWidth", 1.2);
        elseif ~isempty(timedPath.time_s)
            xlim(kinematicAxes(quantityIndex), ...
                [timedPath.time_s(1), timedPath.time_s(end)]);
            text(kinematicAxes(quantityIndex), 0.5, 0.5, ...
                "Unconstrained / not reported", "Units", "normalized", ...
                "HorizontalAlignment", "center", "Color", [0.35 0.35 0.35]);
        end
        ylabel(kinematicAxes(quantityIndex), labels(quantityIndex));
        grid(kinematicAxes(quantityIndex), "on");
        box(kinematicAxes(quantityIndex), "on");
    end
    xlabel(kinematicAxes(end), "Time (s)");
    hasPositionLines = ~isempty(timedPath.time_s) && ...
        size(timedPath.position_deg, 2) == 2 && ...
        any(isfinite(timedPath.position_deg), "all");
    if hasPositionLines
        legend(kinematicAxes(1), ["Azimuth" "Elevation"], ...
            "Location", "best");
    end
    title(layout, options.Title);
end
animation = struct();
if result.Success && options.ShowAnimation
    animationPause_s = options.Pause_s;
    if options.FigureVisible == "off"
        animationPause_s = 0;
    end
    animationOptions = struct( ...
        "FigureVisible", options.FigureVisible, ...
        "FrameStride", options.FrameStride, ...
        "Pause_s", animationPause_s, ...
        "ShowSweptSurfaces", options.ShowSweptSurfaces, ...
        "MaximumDisplayedSlicesPerObstacle", ...
            options.MaximumDisplayedSlicesPerObstacle, ...
        "Title", options.Title + " animation");
    if ~isempty(targetTrack.time_s)
        animationOptions.TargetTime_s = targetTrack.time_s;
        animationOptions.TargetPosition_deg = targetTrack.position_deg;
        animationOptions.TargetLabel = "Moving target";
    end
    animation = animateAzElTimedSlopePath( ...
        timedPath, plannerDiagnostics.ObstacleField, animationOptions);
end
handles = struct("WorkspaceFigure", workspaceFigure, ...
    "WorkspaceAxes", workspaceAxes, ...
    "VisibilityGraphs", visibilityGraphs, ...
    "KinematicsFigure", kinematicsFigure, ...
    "KinematicsAxes", kinematicAxes, ...
    "Animation", animation);
end

%% Section 4: Local Functions

function options = resolvePlotOptions(defaults, overrides)
% PURPOSE
%   - Merge and validate presentation and animation controls.
if ~isstruct(overrides) || ~isscalar(overrides)
    error("plotAzElMotion:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
[options, unknown] = azElInternal.resolveOptions(defaults, overrides);
if ~isempty(unknown)
    warning("plotAzElMotion:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknown, ", "));
end
options.FigureVisible = lower(string(options.FigureVisible));
options.Title = string(options.Title);
if ~isscalar(options.FigureVisible) || ...
        ~any(options.FigureVisible == ["on" "off"])
    error("plotAzElMotion:InvalidFigureVisible", ...
        "FigureVisible must be on or off.");
end
if ~isscalar(options.Title)
    error("plotAzElMotion:InvalidTitle", "Title must be scalar text.");
end
logicalOptionNames = ["ShowAnimation" "ShowKinematics" ...
    "ShowVisibilityGraphs" "ShowSweptSurfaces"];
for optionIndex = 1:numel(logicalOptionNames)
    optionName = logicalOptionNames(optionIndex);
    options.(optionName) = azElInternal.normalizeLogicalScalar( ...
        options.(optionName), optionName, ...
        "plotAzElMotion:InvalidLogicalOption");
end
validateattributes(options.FrameStride, {'numeric'}, ...
    {'scalar','integer','positive'});
validateattributes(options.Pause_s, {'numeric'}, ...
    {'scalar','real','finite','nonnegative'});
validateattributes(options.MaximumDisplayedSlicesPerObstacle, {'numeric'}, ...
    {'scalar','integer','positive'});
validateattributes(options.MaximumDisplayedVisibilitySnapshots, ...
    {'numeric'}, {'scalar','integer','positive'});
end

function inspector = createVisibilityGraphInspector(diagnostics, options)
% PURPOSE
%   - Show retained visibility graphs in azimuth/elevation/time space.
%   - Provide controls for stepping graphs or selecting the nearest graph
%     to an arbitrary requested time without rerunning planning.
graphs = diagnostics.Search.VisibilityGraphs;
spaceTimeGraph = diagnostics.Search.SpaceTimeVisibilityGraph;
figureHandle = figure( ...
    "Visible", options.FigureVisible, ...
    "Name", options.Title + " visibility graphs", ...
    "Position", [80 80 1320 720]);
overviewAxes = axes(figureHandle, ...
    "Position", [0.055 0.18 0.42 0.75]);
snapshotAxes = axes(figureHandle, ...
    "Position", [0.55 0.18 0.42 0.75]);

previousButton = uicontrol(figureHandle, "Style", "pushbutton", ...
    "Units", "normalized", "Position", [0.07 0.055 0.08 0.05], ...
    "String", "Previous", "Callback", ...
    @(source, event) visibilityGraphControlCallback( ...
    source, event, "previous"));
nextButton = uicontrol(figureHandle, "Style", "pushbutton", ...
    "Units", "normalized", "Position", [0.16 0.055 0.08 0.05], ...
    "String", "Next", "Callback", ...
    @(source, event) visibilityGraphControlCallback( ...
    source, event, "next"));
slider = uicontrol(figureHandle, "Style", "slider", ...
    "Units", "normalized", "Position", [0.27 0.065 0.32 0.03], ...
    "Callback", @(source, event) visibilityGraphControlCallback( ...
    source, event, "slider"));
uicontrol(figureHandle, "Style", "text", "Units", "normalized", ...
    "Position", [0.61 0.057 0.08 0.035], ...
    "HorizontalAlignment", "right", "String", "Time (s):");
timeEdit = uicontrol(figureHandle, "Style", "edit", ...
    "Units", "normalized", "Position", [0.70 0.055 0.10 0.05], ...
    "String", "", "Callback", ...
    @(source, event) visibilityGraphControlCallback( ...
    source, event, "time"));
goButton = uicontrol(figureHandle, "Style", "pushbutton", ...
    "Units", "normalized", "Position", [0.81 0.055 0.06 0.05], ...
    "String", "Go", "Callback", ...
    @(source, event) visibilityGraphControlCallback( ...
    source, event, "time"));
statusText = uicontrol(figureHandle, "Style", "text", ...
    "Units", "normalized", "Position", [0.28 0.01 0.58 0.035], ...
    "HorizontalAlignment", "center", "String", "");

inspector = struct( ...
    "Figure", figureHandle, ...
    "OverviewAxes", overviewAxes, ...
    "SnapshotAxes", snapshotAxes, ...
    "PreviousButton", previousButton, ...
    "NextButton", nextButton, ...
    "Slider", slider, ...
    "TimeEdit", timeEdit, ...
    "GoButton", goButton, ...
    "StatusText", statusText);

if isempty(graphs)
    if ~isempty(spaceTimeGraph.NodeAzElTime)
        hold(overviewAxes, "on");
        drawSpaceTimeVisibilityGraph(overviewAxes, spaceTimeGraph);
        xlabel(overviewAxes, "Azimuth (deg)");
        ylabel(overviewAxes, "Elevation (deg)");
        zlabel(overviewAxes, "Time (s)");
        grid(overviewAxes, "on");
        box(overviewAxes, "on");
        view(overviewAxes, 3);
        addVisibilityGraphLegend(overviewAxes, true);
    else
        axis(overviewAxes, "off");
        text(overviewAxes, 0.5, 0.5, ...
            "No retained visibility graphs are available.", ...
            "Units", "normalized", "HorizontalAlignment", "center");
    end
    axis(snapshotAxes, "off");
    set([previousButton nextButton slider timeEdit goButton], ...
        "Enable", "off");
    set(statusText, "String", "No graph snapshots returned.");
    return;
end

snapshotCount = numel(graphs);
set(slider, "Min", 1, "Max", max(1, snapshotCount), "Value", 1);
if snapshotCount > 1
    set(slider, "SliderStep", ...
        [1 / (snapshotCount - 1), min(1, 10 / (snapshotCount - 1))]);
else
    set([previousButton nextButton slider], "Enable", "off");
end

drawVisibilityGraphOverview(overviewAxes, graphs, ...
    diagnostics.ProtectedObstacleData, ...
    options.MaximumDisplayedVisibilitySnapshots, options.Title);
drawSpaceTimeVisibilityGraph(overviewAxes, spaceTimeGraph);
state = struct( ...
    "Graphs", graphs, ...
    "Obstacles", diagnostics.ProtectedObstacleData, ...
    "SnapshotAxes", snapshotAxes, ...
    "Slider", slider, ...
    "TimeEdit", timeEdit, ...
    "StatusText", statusText, ...
    "SelectedIndex", 1);
guidata(figureHandle, state);
redrawVisibilityGraphSnapshot(figureHandle, 1, NaN);
end

function [plannerDiagnostics, targetTrack] = ...
        plotDiagnosticViews(diagnostics)
% PURPOSE
%   - Select planner data and optional intercept target-track data.
plannerDiagnostics = diagnostics;
if isfield(diagnostics, "Planner")
    plannerDiagnostics = diagnostics.Planner;
end
targetTrack = struct( ...
    "time_s", zeros(0, 1), ...
    "position_deg", zeros(0, 2));
if isfield(diagnostics, "TargetTrack")
    targetTrack = diagnostics.TargetTrack;
end
end

function visibilityGraphControlCallback(source, ~, action)
% PURPOSE
%   - Resolve one inspector control event to a retained snapshot index.
figureHandle = ancestor(source, "figure");
state = guidata(figureHandle);
if isempty(state) || ~isfield(state, "Graphs") || isempty(state.Graphs)
    return;
end
snapshotCount = numel(state.Graphs);
selectedIndex = state.SelectedIndex;
requestedTime_s = NaN;
switch action
    case "previous"
        selectedIndex = max(1, selectedIndex - 1);
    case "next"
        selectedIndex = min(snapshotCount, selectedIndex + 1);
    case "slider"
        selectedIndex = min(snapshotCount, max(1, round(source.Value)));
    case "time"
        requestedTime_s = str2double(string(state.TimeEdit.String));
        if ~isfinite(requestedTime_s)
            set(state.StatusText, "String", ...
                "Enter a finite time in seconds.");
            return;
        end
        graphTime_s = [state.Graphs.Time_s].';
        [~, selectedIndex] = min(abs(graphTime_s - requestedTime_s));
end
redrawVisibilityGraphSnapshot( ...
    figureHandle, selectedIndex, requestedTime_s);
end

function redrawVisibilityGraphSnapshot( ...
        figureHandle, selectedIndex, requestedTime_s)
% PURPOSE
%   - Redraw one exact retained graph selected by index or nearest time.
state = guidata(figureHandle);
graph = state.Graphs(selectedIndex);
axesHandle = state.SnapshotAxes;
% Graph artists deliberately hide their handles to keep the legend stable.
% Plain cla preserves those hidden children, so stepping snapshots would
% stack every previous graph. The reset form deletes all children while
% preserving the axes position used by the inspector layout.
cla(axesHandle, "reset");
hold(axesHandle, "on");
plotObstacleSnapshot(axesHandle, state.Obstacles, graph.Time_s, false);
drawVisibilityGraphSnapshot(axesHandle, graph, false);
xlabel(axesHandle, "Azimuth (deg)");
ylabel(axesHandle, "Elevation (deg)");
axis(axesHandle, "equal");
grid(axesHandle, "on");
box(axesHandle, "on");
testedCount = nnz(triu(graph.VisibilityTestedMask, 1));
blockedCount = nnz(triu(graph.VisibilityBlockedMask, 1));
title(axesHandle, sprintf( ...
    "Snapshot %d/%d, t = %.3f s | tested %d, blocked %d", ...
    selectedIndex, numel(state.Graphs), graph.Time_s, ...
    testedCount, blockedCount));
addVisibilityGraphLegend(axesHandle, false);

state.SelectedIndex = selectedIndex;
set(state.Slider, "Value", selectedIndex);
set(state.TimeEdit, "String", sprintf("%.6g", graph.Time_s));
if isfinite(requestedTime_s)
    status = sprintf( ...
        "Requested %.3f s; showing nearest retained graph at %.3f s.", ...
        requestedTime_s, graph.Time_s);
else
    status = sprintf("Showing retained graph at %.3f s.", graph.Time_s);
end
set(state.StatusText, "String", status);
setappdata(figureHandle, "SelectedSnapshotIndex", selectedIndex);
setappdata(figureHandle, "SelectedSnapshotTime_s", graph.Time_s);
guidata(figureHandle, state);
drawnow limitrate;
end

function drawVisibilityGraphOverview( ...
        axesHandle, graphs, obstacles, maximumSnapshotCount, titleText)
% PURPOSE
%   - Stack representative returned graphs in three-dimensional time space.
snapshotCount = numel(graphs);
displayIndex = unique(round(linspace( ...
    1, snapshotCount, min(snapshotCount, maximumSnapshotCount))));
hold(axesHandle, "on");
for graphIndex = reshape(displayIndex, 1, [])
    graph = graphs(graphIndex);
    plotObstacleSnapshot(axesHandle, obstacles, graph.Time_s, true);
    drawVisibilityGraphSnapshot(axesHandle, graph, true);
end
xlabel(axesHandle, "Azimuth (deg)");
ylabel(axesHandle, "Elevation (deg)");
zlabel(axesHandle, "Time (s)");
title(axesHandle, sprintf("%s - %d of %d retained graphs", ...
    titleText, numel(displayIndex), snapshotCount));
grid(axesHandle, "on");
box(axesHandle, "on");
view(axesHandle, 3);
addVisibilityGraphLegend(axesHandle, true);
end

function drawVisibilityGraphSnapshot(axesHandle, graph, useTimeAxis)
% PURPOSE
%   - Draw accepted, blocked, boundary, selected, and pruned graph data.
nodePosition_deg = graph.NodePosition_deg;
blockedMask = graph.VisibilityBlockedMask;
clearMask = graph.EdgeType == "visibility";
boundaryMask = graph.EdgeType == "boundary";
[blockedAzimuth_deg, blockedElevation_deg] = ...
    straightEdgeCoordinates(nodePosition_deg, blockedMask);
[clearAzimuth_deg, clearElevation_deg] = ...
    straightEdgeCoordinates(nodePosition_deg, clearMask);
[boundaryAzimuth_deg, boundaryElevation_deg] = ...
    routedEdgeCoordinates(graph.EdgeRoute_deg, boundaryMask);

plotGraphCoordinates(axesHandle, blockedAzimuth_deg, ...
    blockedElevation_deg, graph.Time_s, useTimeAxis, ...
    ":", [0.85 0.20 0.20], 0.8);
plotGraphCoordinates(axesHandle, clearAzimuth_deg, ...
    clearElevation_deg, graph.Time_s, useTimeAxis, ...
    "-", [0.20 0.65 0.25], 0.8);
plotGraphCoordinates(axesHandle, boundaryAzimuth_deg, ...
    boundaryElevation_deg, graph.Time_s, useTimeAxis, ...
    "-", [0.90 0.55 0.10], 1.0);
plotGraphCoordinates(axesHandle, graph.PathPosition_deg(:, 1), ...
    graph.PathPosition_deg(:, 2), graph.Time_s, useTimeAxis, ...
    "-", [0.10 0.30 0.90], 2.4);

candidatePosition_deg = nodePosition_deg(3:end, :);
activeMask = graph.CandidateActiveMask;
plotGraphPoints(axesHandle, candidatePosition_deg(activeMask, :), ...
    graph.Time_s, useTimeAxis, ".", [0.10 0.10 0.10], 10);
plotGraphPoints(axesHandle, candidatePosition_deg(~activeMask, :), ...
    graph.Time_s, useTimeAxis, "x", [0.60 0.60 0.60], 6);
selectedNodeIndex = graph.PathNodeIndex;
selectedNodeIndex = selectedNodeIndex(selectedNodeIndex > 2);
plotGraphPoints(axesHandle, nodePosition_deg(selectedNodeIndex, :), ...
    graph.Time_s, useTimeAxis, "s", [0.10 0.30 0.90], 7);
if size(nodePosition_deg, 1) >= 2
    plotGraphPoints(axesHandle, nodePosition_deg(1, :), ...
        graph.Time_s, useTimeAxis, "o", [0.10 0.65 0.10], 7);
    plotGraphPoints(axesHandle, nodePosition_deg(2, :), ...
        graph.Time_s, useTimeAxis, "o", [0.85 0.10 0.10], 7);
end
end

function drawSpaceTimeVisibilityGraph(axesHandle, graph)
% PURPOSE
%   - Plot retained directed edges and the selected timed route in 3-D.
if isempty(graph.NodeAzElTime)
    return;
end
[acceptedAzimuth_deg, acceptedElevation_deg, acceptedTime_s] = ...
    directedEdgeCoordinates(graph.NodeAzElTime, ...
    graph.AcceptedEdgeSourceNodeIndex, ...
    graph.AcceptedEdgeTargetNodeIndex);
collisionRejected = graph.RejectedEdgeReason == "collision";
[collisionAzimuth_deg, collisionElevation_deg, collisionTime_s] = ...
    directedEdgeCoordinates(graph.NodeAzElTime, ...
    graph.RejectedEdgeSourceNodeIndex(collisionRejected), ...
    graph.RejectedEdgeTargetNodeIndex(collisionRejected));
dynamicsRejected = graph.RejectedEdgeReason == "velocity";
[dynamicsAzimuth_deg, dynamicsElevation_deg, dynamicsTime_s] = ...
    directedEdgeCoordinates(graph.NodeAzElTime, ...
    graph.RejectedEdgeSourceNodeIndex(dynamicsRejected), ...
    graph.RejectedEdgeTargetNodeIndex(dynamicsRejected));
plot3(axesHandle, acceptedAzimuth_deg, acceptedElevation_deg, ...
    acceptedTime_s, "-", "Color", [0.10 0.65 0.70], ...
    "LineWidth", 0.5, "HandleVisibility", "off");
plot3(axesHandle, collisionAzimuth_deg, collisionElevation_deg, ...
    collisionTime_s, ":", "Color", [0.85 0.20 0.20], ...
    "LineWidth", 0.5, "HandleVisibility", "off");
plot3(axesHandle, dynamicsAzimuth_deg, dynamicsElevation_deg, ...
    dynamicsTime_s, ":", "Color", [0.55 0.55 0.55], ...
    "LineWidth", 0.5, "HandleVisibility", "off");
if ~isempty(graph.PathTime_s)
    plot3(axesHandle, graph.PathPosition_deg(:, 1), ...
        graph.PathPosition_deg(:, 2), graph.PathTime_s, "-", ...
        "Color", [0.75 0.10 0.75], "LineWidth", 3.0, ...
        "HandleVisibility", "off");
end
end

function [azimuth_deg, elevation_deg, time_s] = ...
        directedEdgeCoordinates(nodeAzElTime, sourceIndex, targetIndex)
% PURPOSE
%   - Batch directed 3-D edges into NaN-separated plot vectors.
edgeCount = numel(sourceIndex);
azimuth_deg = nan(3 * edgeCount, 1);
elevation_deg = nan(3 * edgeCount, 1);
time_s = nan(3 * edgeCount, 1);
firstRow = (1:3:3 * edgeCount).';
secondRow = firstRow + 1;
azimuth_deg(firstRow) = nodeAzElTime(sourceIndex, 1);
azimuth_deg(secondRow) = nodeAzElTime(targetIndex, 1);
elevation_deg(firstRow) = nodeAzElTime(sourceIndex, 2);
elevation_deg(secondRow) = nodeAzElTime(targetIndex, 2);
time_s(firstRow) = nodeAzElTime(sourceIndex, 3);
time_s(secondRow) = nodeAzElTime(targetIndex, 3);
end

function [azimuth_deg, elevation_deg] = straightEdgeCoordinates( ...
        nodePosition_deg, edgeMask)
% PURPOSE
%   - Batch undirected straight edges into NaN-separated plot vectors.
[firstNodeIndex, secondNodeIndex] = find(triu(edgeMask, 1));
edgeCount = numel(firstNodeIndex);
azimuth_deg = nan(3 * edgeCount, 1);
elevation_deg = nan(3 * edgeCount, 1);
firstRow = (1:3:3 * edgeCount).';
secondRow = firstRow + 1;
azimuth_deg(firstRow) = nodePosition_deg(firstNodeIndex, 1);
azimuth_deg(secondRow) = nodePosition_deg(secondNodeIndex, 1);
elevation_deg(firstRow) = nodePosition_deg(firstNodeIndex, 2);
elevation_deg(secondRow) = nodePosition_deg(secondNodeIndex, 2);
end

function [azimuth_deg, elevation_deg] = routedEdgeCoordinates( ...
        edgeRoutes_deg, edgeMask)
% PURPOSE
%   - Batch undirected boundary routes into NaN-separated plot vectors.
[firstNodeIndex, secondNodeIndex] = find(triu(edgeMask, 1));
routeCount = numel(firstNodeIndex);
azimuthByRoute_deg = cell(routeCount, 1);
elevationByRoute_deg = cell(routeCount, 1);
for routeIndex = 1:routeCount
    route_deg = edgeRoutes_deg{ ...
        firstNodeIndex(routeIndex), secondNodeIndex(routeIndex)};
    azimuthByRoute_deg{routeIndex} = [route_deg(:, 1); NaN];
    elevationByRoute_deg{routeIndex} = [route_deg(:, 2); NaN];
end
azimuth_deg = vertcat(azimuthByRoute_deg{:});
elevation_deg = vertcat(elevationByRoute_deg{:});
end

function plotGraphCoordinates(axesHandle, azimuth_deg, elevation_deg, ...
        time_s, useTimeAxis, lineStyle, color, lineWidth)
% PURPOSE
%   - Plot one batched graph category in two or three dimensions.
if isempty(azimuth_deg)
    return;
end
if useTimeAxis
    plot3(axesHandle, azimuth_deg, elevation_deg, ...
        repmat(time_s, size(azimuth_deg)), ...
        "LineStyle", lineStyle, "Color", color, ...
        "LineWidth", lineWidth, "HandleVisibility", "off");
else
    plot(axesHandle, azimuth_deg, elevation_deg, ...
        "LineStyle", lineStyle, "Color", color, ...
        "LineWidth", lineWidth, "HandleVisibility", "off");
end
end

function plotGraphPoints(axesHandle, position_deg, time_s, ...
        useTimeAxis, marker, color, markerSize)
% PURPOSE
%   - Plot one node category in two or three dimensions.
if isempty(position_deg)
    return;
end
if useTimeAxis
    plot3(axesHandle, position_deg(:, 1), position_deg(:, 2), ...
        repmat(time_s, size(position_deg, 1), 1), marker, ...
        "Color", color, "MarkerSize", markerSize, ...
        "HandleVisibility", "off");
else
    plot(axesHandle, position_deg(:, 1), position_deg(:, 2), marker, ...
        "Color", color, "MarkerSize", markerSize, ...
        "HandleVisibility", "off");
end
end

function plotObstacleSnapshot(axesHandle, obstacles, time_s, useTimeAxis)
% PURPOSE
%   - Plot the protected obstacle samples used by one retained graph.
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    [~, sampleIndex] = min(abs(obstacle.time_s - time_s));
    azimuth_deg = obstacle.az_deg{sampleIndex};
    elevation_deg = obstacle.el_deg{sampleIndex};
    plotGraphCoordinates(axesHandle, azimuth_deg, elevation_deg, ...
        time_s, useTimeAxis, "-", [0.25 0.25 0.25], 1.2);
end
end

function addVisibilityGraphLegend(axesHandle, useTimeAxis)
% PURPOSE
%   - Add one compact, stable legend for graph diagnostic categories.
categories = { ...
    "Protected obstacle", "Clear visibility", "Blocked visibility", ...
    "Boundary edge", "Selected graph path", "Active candidate", ...
    "Pruned candidate", "Selected candidate", "Start", "Goal", ...
    "Space-time edge", "Selected space-time path"};
styles = {"-", "-", ":", "-", "-", ".", "x", "s", "o", "o", ...
    "-", "-"};
colors = [0.25 0.25 0.25; 0.20 0.65 0.25; 0.85 0.20 0.20; ...
    0.90 0.55 0.10; 0.10 0.30 0.90; 0.10 0.10 0.10; ...
    0.60 0.60 0.60; 0.10 0.30 0.90; 0.10 0.65 0.10; ...
    0.85 0.10 0.10; 0.10 0.65 0.70; 0.75 0.10 0.75];
for categoryIndex = 1:numel(categories)
    if useTimeAxis
        plot3(axesHandle, NaN, NaN, NaN, styles{categoryIndex}, ...
            "Color", colors(categoryIndex, :), ...
            "DisplayName", categories{categoryIndex});
    else
        plot(axesHandle, NaN, NaN, styles{categoryIndex}, ...
            "Color", colors(categoryIndex, :), ...
            "DisplayName", categories{categoryIndex});
    end
end
legend(axesHandle, "Location", "best");
end

function plotObstacleHistory( ...
        axesHandle, obstacles, color, lineStyle, displayName)
% PURPOSE
%   - Plot representative full-resolution obstacle-history slices.
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    sampleCount = numel(obstacle.time_s);
    sampleIndices = unique(round(linspace(1, sampleCount, min(3, sampleCount))));
    for sampleIndex = reshape(sampleIndices, 1, [])
        azimuth_deg = obstacle.az_deg{sampleIndex};
        elevation_deg = obstacle.el_deg{sampleIndex};
        plot(axesHandle, azimuth_deg, elevation_deg, ...
            "Color", color, "LineStyle", lineStyle, ...
            "LineWidth", 1.2, ...
            "HandleVisibility", "off");
    end
end
if ~isempty(obstacles)
    plot(axesHandle, NaN, NaN, "Color", color, ...
        "LineStyle", lineStyle, "LineWidth", 1.2, ...
        "DisplayName", displayName);
end
end
