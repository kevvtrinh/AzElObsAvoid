function handles = plotAzElMotion(result, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = plotAzElMotion()
%   handles = plotAzElMotion(result)
%   handles = plotAzElMotion(result, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot obstacle geometry, candidate routes, the selected motion,
%     time-stacked visibility graphs, and kinematic histories.
%**************************************************************************
% INPUTS
%   - result (scalar planner-result struct)
%       Output from planAzElMotion.
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
if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end
options = resolvePlotOptions(defaults, optionOverrides);
required = ["Success" "originalAzElData" "azElData" "candidateRoutes_deg" ...
    "selectedRoute_deg" "timedSlopePath" "initialState" "goalState"];
if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, required))
    error("plotAzElMotion:InvalidResult", ...
        "result must be the scalar output from planAzElMotion.");
end

%% Section 1: Plot Workspace & Routes

workspaceFigure = figure("Visible", options.FigureVisible, ...
    "Name", options.Title);
workspaceAxes = axes(workspaceFigure);
hold(workspaceAxes, "on");
plotObstacleHistory(workspaceAxes, result.originalAzElData, ...
    [0.75 0.15 0.15], "-", "Original obstacle");
plotObstacleHistory(workspaceAxes, result.azElData, ...
    [0.90 0.45 0.05], "--", "Protected obstacle");
for routeIndex = 1:numel(result.candidateRoutes_deg)
    route_deg = result.candidateRoutes_deg{routeIndex};
    plot(workspaceAxes, route_deg(:, 1), route_deg(:, 2), ...
        "Color", [0.75 0.75 0.75], "HandleVisibility", "off");
end
plot(workspaceAxes, result.selectedRoute_deg(:, 1), ...
    result.selectedRoute_deg(:, 2), "--", "Color", [0.15 0.35 0.8], ...
    "LineWidth", 1.2, "DisplayName", "Selected geometric route");
timedPath = result.timedSlopePath;
if ~isempty(timedPath.position_deg)
    plot(workspaceAxes, timedPath.position_deg(:, 1), ...
        timedPath.position_deg(:, 2), "k-", "LineWidth", 2, ...
        "DisplayName", "Timed motion");
end
if isfield(result, "TargetTrackPosition_deg") && ...
        ~isempty(result.TargetTrackPosition_deg)
    plot(workspaceAxes, result.TargetTrackPosition_deg(:, 1), ...
        result.TargetTrackPosition_deg(:, 2), "-.", ...
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
    visibilityGraphs = createVisibilityGraphInspector(result, options);
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
    legend(kinematicAxes(1), ["Azimuth" "Elevation"], ...
        "Location", "best");
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
    if isfield(result, "TargetTrackTime_s") && ...
            isfield(result, "TargetTrackPosition_deg")
        animationOptions.TargetTime_s = result.TargetTrackTime_s;
        animationOptions.TargetPosition_deg = ...
            result.TargetTrackPosition_deg;
        animationOptions.TargetLabel = "Moving target";
    end
    animation = animateAzElTimedSlopePath( ...
        timedPath, result.obstacleField, animationOptions);
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
%% Section 0: Header & Readme
% SYNTAX
%   options = resolvePlotOptions(defaults, overrides)
%**************************************************************************
% PURPOSE
%   - Merge and validate presentation and animation controls.
%**************************************************************************
% INPUTS
%   - defaults (scalar struct)
%   - overrides (scalar struct)
%       Partial controls; unknown fields are ignored with one warning.
%**************************************************************************
% OUTPUTS
%   - options (scalar struct)
%       Fully resolved and normalized plot controls.
%**************************************************************************
% UNITS
%   - Pause_s is seconds; counts and logical controls are dimensionless.
%**************************************************************************
if ~isstruct(overrides) || ~isscalar(overrides)
    error("plotAzElMotion:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
unknown = setdiff(fieldnames(overrides), fieldnames(defaults), "stable");
if ~isempty(unknown)
    warning("plotAzElMotion:UnknownOptions", ...
        "Ignored unknown fields: %s.", strjoin(string(unknown), ", "));
    overrides = rmfield(overrides, unknown);
end
options = defaults;
names = fieldnames(overrides);
for nameIndex = 1:numel(names)
    if ~isempty(overrides.(names{nameIndex}))
        options.(names{nameIndex}) = overrides.(names{nameIndex});
    end
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
options.ShowAnimation = logicalScalar(options.ShowAnimation, ...
    "ShowAnimation");
options.ShowKinematics = logicalScalar(options.ShowKinematics, ...
    "ShowKinematics");
options.ShowVisibilityGraphs = logicalScalar( ...
    options.ShowVisibilityGraphs, "ShowVisibilityGraphs");
options.ShowSweptSurfaces = logicalScalar(options.ShowSweptSurfaces, ...
    "ShowSweptSurfaces");
validateattributes(options.FrameStride, {'numeric'}, ...
    {'scalar','integer','positive'});
validateattributes(options.Pause_s, {'numeric'}, ...
    {'scalar','real','finite','nonnegative'});
validateattributes(options.MaximumDisplayedSlicesPerObstacle, {'numeric'}, ...
    {'scalar','integer','positive'});
validateattributes(options.MaximumDisplayedVisibilitySnapshots, ...
    {'numeric'}, {'scalar','integer','positive'});
end

function value = logicalScalar(value, name)
%% Section 0: Header & Readme
% SYNTAX
%   value = logicalScalar(value, name)
%**************************************************************************
% PURPOSE
%   - Normalize one logical presentation control.
%**************************************************************************
% INPUTS
%   - value (scalar logical or binary numeric value)
%   - name (scalar text)
%       Option name used in diagnostics.
%**************************************************************************
% OUTPUTS
%   - value (logical scalar)
%**************************************************************************
% UNITS
%   - Values are dimensionless.
%**************************************************************************
if ~(islogical(value) && isscalar(value)) && ...
        ~(isnumeric(value) && isscalar(value) && ...
        isfinite(value) && any(value == [0 1]))
    error("plotAzElMotion:InvalidLogicalOption", ...
        "%s must be scalar logical or binary numeric.", name);
end
value = logical(value);
end

function inspector = createVisibilityGraphInspector(result, options)
%% Section 0: Header & Readme
% SYNTAX
%   inspector = createVisibilityGraphInspector(result, options)
%**************************************************************************
% PURPOSE
%   - Show retained visibility graphs in azimuth/elevation/time space.
%   - Provide controls for stepping graphs or selecting the nearest graph
%     to an arbitrary requested time without rerunning planning.
%**************************************************************************
% INPUTS
%   - result (scalar planner-result struct)
%   - options (scalar resolved plotting-options struct)
%**************************************************************************
% OUTPUTS
%   - inspector (scalar struct)
%       Figure, axes, slider, time edit, buttons, and status handles.
%**************************************************************************
% UNITS
%   - Position is degrees and graph time is seconds.
%**************************************************************************
graphs = visibilityGraphsFromResult(result);
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
    axis(overviewAxes, "off");
    axis(snapshotAxes, "off");
    text(overviewAxes, 0.5, 0.5, ...
        "No retained visibility graphs are available.", ...
        "Units", "normalized", "HorizontalAlignment", "center");
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

drawVisibilityGraphOverview(overviewAxes, graphs, result.azElData, ...
    options.MaximumDisplayedVisibilitySnapshots, options.Title);
state = struct( ...
    "Graphs", graphs, ...
    "Obstacles", result.azElData, ...
    "SnapshotAxes", snapshotAxes, ...
    "Slider", slider, ...
    "TimeEdit", timeEdit, ...
    "StatusText", statusText, ...
    "SelectedIndex", 1);
guidata(figureHandle, state);
redrawVisibilityGraphSnapshot(figureHandle, 1, NaN);
end

function graphs = visibilityGraphsFromResult(result)
%% Section 0: Header & Readme
% SYNTAX
%   graphs = visibilityGraphsFromResult(result)
%**************************************************************************
% PURPOSE
%   - Read direct-planner or intercept-wrapper graph diagnostics.
%**************************************************************************
% INPUTS
%   - result (scalar planner-result struct)
%**************************************************************************
% OUTPUTS
%   - graphs (visibility-graph structure array, possibly empty)
%**************************************************************************
% UNITS
%   - Graph positions are degrees and Time_s is seconds.
%**************************************************************************
graphs = struct([]);
if isfield(result, "PlannerSearchDiagnostics") && ...
        isfield(result.PlannerSearchDiagnostics, "VisibilityGraphs")
    graphs = result.PlannerSearchDiagnostics.VisibilityGraphs;
elseif isfield(result, "SearchDiagnostics") && ...
        isfield(result.SearchDiagnostics, "VisibilityGraphs")
    graphs = result.SearchDiagnostics.VisibilityGraphs;
end
end

function visibilityGraphControlCallback(source, ~, action)
%% Section 0: Header & Readme
% SYNTAX
%   visibilityGraphControlCallback(source, event, action)
%**************************************************************************
% PURPOSE
%   - Resolve one inspector control event to a retained snapshot index.
%**************************************************************************
% INPUTS
%   - source (graphics handle), event (unused event record)
%   - action (previous, next, slider, or time)
%**************************************************************************
% OUTPUTS
%   - None. The inspector figure and stored state are updated.
%**************************************************************************
% UNITS
%   - The time-edit value is seconds; indices are dimensionless.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   redrawVisibilityGraphSnapshot( ...
%       figureHandle, selectedIndex, requestedTime_s)
%**************************************************************************
% PURPOSE
%   - Redraw one exact retained graph selected by index or nearest time.
%**************************************************************************
% INPUTS
%   - figureHandle (scalar figure handle)
%   - selectedIndex (positive graph index)
%   - requestedTime_s (finite scalar or NaN when stepping by index)
%**************************************************************************
% OUTPUTS
%   - None. Graphics, controls, and figure application data are updated.
%**************************************************************************
% UNITS
%   - requestedTime_s and graph Time_s are seconds.
%**************************************************************************
state = guidata(figureHandle);
graph = state.Graphs(selectedIndex);
axesHandle = state.SnapshotAxes;
cla(axesHandle);
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
%% Section 0: Header & Readme
% SYNTAX
%   drawVisibilityGraphOverview( ...
%       axesHandle, graphs, obstacles, maximumSnapshotCount, titleText)
%**************************************************************************
% PURPOSE
%   - Stack representative returned graphs in three-dimensional time space.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes handle)
%   - graphs (visibility-graph structure array)
%   - obstacles (canonical obstacle structure array)
%   - maximumSnapshotCount (positive integer)
%   - titleText (scalar text)
%**************************************************************************
% OUTPUTS
%   - None. Graphics are added to axesHandle.
%**************************************************************************
% UNITS
%   - Axes are azimuth degrees, elevation degrees, and time seconds.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   drawVisibilityGraphSnapshot(axesHandle, graph, useTimeAxis)
%**************************************************************************
% PURPOSE
%   - Draw accepted, blocked, boundary, selected, and pruned graph data.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes handle)
%   - graph (scalar visibility-graph struct)
%   - useTimeAxis (logical scalar)
%**************************************************************************
% OUTPUTS
%   - None. Graphics are added to axesHandle.
%**************************************************************************
% UNITS
%   - Position is degrees and the optional third axis is seconds.
%**************************************************************************
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

function [azimuth_deg, elevation_deg] = straightEdgeCoordinates( ...
        nodePosition_deg, edgeMask)
%% Section 0: Header & Readme
% SYNTAX
%   [azimuth_deg, elevation_deg] = straightEdgeCoordinates( ...
%       nodePosition_deg, edgeMask)
%**************************************************************************
% PURPOSE
%   - Batch undirected straight edges into NaN-separated plot vectors.
%**************************************************************************
% INPUTS
%   - nodePosition_deg (N-by-2 numeric matrix)
%   - edgeMask (N-by-N logical matrix)
%**************************************************************************
% OUTPUTS
%   - azimuth_deg, elevation_deg (NaN-separated numeric columns)
%**************************************************************************
% UNITS
%   - Coordinates are degrees.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   [azimuth_deg, elevation_deg] = routedEdgeCoordinates( ...
%       edgeRoutes_deg, edgeMask)
%**************************************************************************
% PURPOSE
%   - Batch undirected boundary routes into NaN-separated plot vectors.
%**************************************************************************
% INPUTS
%   - edgeRoutes_deg (N-by-N cell array of route matrices)
%   - edgeMask (N-by-N logical matrix)
%**************************************************************************
% OUTPUTS
%   - azimuth_deg, elevation_deg (NaN-separated numeric columns)
%**************************************************************************
% UNITS
%   - Coordinates are degrees.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   plotGraphCoordinates(axesHandle, azimuth_deg, elevation_deg, ...
%       time_s, useTimeAxis, lineStyle, color, lineWidth)
%**************************************************************************
% PURPOSE
%   - Plot one batched graph category in two or three dimensions.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes handle)
%   - azimuth_deg, elevation_deg (matching numeric vectors)
%   - time_s (finite scalar), useTimeAxis (logical scalar)
%   - lineStyle (scalar text), color (RGB row), lineWidth (positive scalar)
%**************************************************************************
% OUTPUTS
%   - None. Graphics are added to axesHandle.
%**************************************************************************
% UNITS
%   - Position is degrees, time is seconds, and width is points.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   plotGraphPoints(axesHandle, position_deg, time_s, ...
%       useTimeAxis, marker, color, markerSize)
%**************************************************************************
% PURPOSE
%   - Plot one node category in two or three dimensions.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes handle)
%   - position_deg (N-by-2 numeric matrix), time_s (finite scalar)
%   - useTimeAxis (logical scalar), marker (scalar text)
%   - color (RGB row), markerSize (positive scalar)
%**************************************************************************
% OUTPUTS
%   - None. Graphics are added to axesHandle.
%**************************************************************************
% UNITS
%   - Position is degrees, time is seconds, and marker size is points.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   plotObstacleSnapshot(axesHandle, obstacles, time_s, useTimeAxis)
%**************************************************************************
% PURPOSE
%   - Plot the protected obstacle samples used by one retained graph.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes handle)
%   - obstacles (canonical obstacle structure array)
%   - time_s (finite scalar), useTimeAxis (logical scalar)
%**************************************************************************
% OUTPUTS
%   - None. Graphics are added to axesHandle.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   addVisibilityGraphLegend(axesHandle, useTimeAxis)
%**************************************************************************
% PURPOSE
%   - Add one compact, stable legend for graph diagnostic categories.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes handle)
%   - useTimeAxis (logical scalar)
%**************************************************************************
% OUTPUTS
%   - None. Legend prototypes are added to axesHandle.
%**************************************************************************
% UNITS
%   - Values are dimensionless.
%**************************************************************************
categories = { ...
    "Protected obstacle", "Clear visibility", "Blocked visibility", ...
    "Boundary edge", "Selected graph path", "Active candidate", ...
    "Pruned candidate", "Selected candidate", "Start", "Goal"};
styles = {"-", "-", ":", "-", "-", ".", "x", "s", "o", "o"};
colors = [0.25 0.25 0.25; 0.20 0.65 0.25; 0.85 0.20 0.20; ...
    0.90 0.55 0.10; 0.10 0.30 0.90; 0.10 0.10 0.10; ...
    0.60 0.60 0.60; 0.10 0.30 0.90; 0.10 0.65 0.10; ...
    0.85 0.10 0.10];
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
%% Section 0: Header & Readme
% SYNTAX
%   plotObstacleHistory(axesHandle, obstacles, color, ...
%       lineStyle, displayName)
%**************************************************************************
% PURPOSE
%   - Plot representative full-resolution obstacle-history slices.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes handle)
%   - obstacles (structure array)
%       Canonical obstacle records with time_s, az_deg, and el_deg.
%   - color (1-by-3 numeric RGB row)
%   - lineStyle, displayName (scalar text)
%**************************************************************************
% OUTPUTS
%   - None. Graphics are added to axesHandle.
%**************************************************************************
% UNITS
%   - Azimuth and elevation are degrees.
%**************************************************************************
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
