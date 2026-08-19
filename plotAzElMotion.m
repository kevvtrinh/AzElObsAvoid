function handles = plotAzElMotion(result, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = plotAzElMotion()
%   handles = plotAzElMotion(result)
%   handles = plotAzElMotion(result, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot returned HS3 motion, search diagnostics, and physical limits.
%   - Animate returned motion without rerunning planning or collision logic.
%**************************************************************************
% INPUTS
%   - result (scalar planAzElMotion result)
%   - optionOverrides (scalar struct, optional; default struct())
%       Controls workspace, visibility, kinematic, swept-surface, and
%       animation displays. Hidden figures never pause.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Stable workspace, visibility, kinematic, and animation handles.
%**************************************************************************
% UNITS
%   - Axes show degrees, seconds, deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Display Controls

defaults = struct( ...
    "FigureVisible", "on", ...
    "Title", "Azimuth/elevation HS3 plan", ...
    "ShowWorkspace", true, ...
    "ShowKinematics", true, ...
    "ShowAnimation", true, ...
    "ShowVisibilityGraphs", true, ...
    "FrameStride", 4, ...
    "Pause_s", 0.01, ...
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
if ~isstruct(result) || ~isscalar(result) || ...
        ~all(isfield(result, {'Inputs', 'Success', 'SearchDiagnostics'}))
    error("plotAzElMotion:InvalidResult", ...
        "result must be a scalar planAzElMotion result.");
end
optionOverrides = normalizePlotAliases(optionOverrides);
[options, unknownNames] = azElInternal.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("plotAzElMotion:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
options.FigureVisible = lower(string(options.FigureVisible));
options.Title = string(options.Title);
if ~isscalar(options.FigureVisible) || ...
        ~any(options.FigureVisible == ["on", "off"])
    error("plotAzElMotion:InvalidFigureVisible", ...
        "FigureVisible must be 'on' or 'off'.");
end
if ~isscalar(options.Title)
    error("plotAzElMotion:InvalidTitle", ...
        "Title must be scalar text.");
end
logicalNames = ["ShowWorkspace", "ShowKinematics", "ShowAnimation", ...
    "ShowVisibilityGraphs", "ShowSweptSurfaces"];
for name = logicalNames
    options.(name) = azElInternal.normalizeLogicalScalar( ...
        options.(name), name, "plotAzElMotion:InvalidLogicalOption");
end
validateattributes(options.FrameStride, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(options.Pause_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
countNames = ["MaximumDisplayedSlicesPerObstacle", ...
    "MaximumDisplayedVisibilitySnapshots"];
for name = countNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'integer', 'positive'});
end
handles = emptyHandles(options);

%% Section 2: Plot Workspace And Failure Diagnostics

if options.ShowWorkspace
    workspaceFigure = figure( ...
        "Name", options.Title, "Visible", options.FigureVisible);
    workspaceAxes = axes(workspaceFigure);
    configureSpatialAxes(workspaceAxes);
    obstacles = result.Inputs.obstacles;
    displayTime_s = result.Inputs.initialState.time_s;
    drawObstacles(workspaceAxes, obstacles, displayTime_s, true);
    gridRecord = result.SearchDiagnostics.Grid;
    drawSearchEdges(workspaceAxes, gridRecord);
    if isfield(gridRecord, "ExploredNodes_deg") && ...
            ~isempty(gridRecord.ExploredNodes_deg)
        explored_deg = gridRecord.ExploredNodes_deg;
        scatter(workspaceAxes, explored_deg(:, 1), explored_deg(:, 2), ...
            9, [0.45 0.45 0.45], "filled", ...
            "DisplayName", "Expanded search node");
    end
    for seedIndex = 1:numel(result.Seeds)
        route_deg = result.Seeds(seedIndex).position_deg;
        plot(workspaceAxes, route_deg(:, 1), route_deg(:, 2), ...
            ":", "Color", [0.55 0.55 0.55], ...
            "DisplayName", "Topology seed " + seedIndex);
    end
    if result.Success
        plot(workspaceAxes, result.SelectedSeed_deg(:, 1), ...
            result.SelectedSeed_deg(:, 2), "m--", "LineWidth", 1.5, ...
            "DisplayName", "Selected topology seed");
        plot(workspaceAxes, result.position_deg(:, 1), ...
            result.position_deg(:, 2), "b-", "LineWidth", 2, ...
            "DisplayName", "Validated HS3 motion");
    elseif result.SearchDiagnostics.BestPartialSeedIndex > 0
        partialSeed = result.Seeds( ...
            result.SearchDiagnostics.BestPartialSeedIndex).position_deg;
        plot(workspaceAxes, partialSeed(:, 1), partialSeed(:, 2), ...
            "m--", "LineWidth", 1.5, ...
            "DisplayName", "Best partial seed");
    end
    drawTargetTrack(workspaceAxes, result.Inputs.goalState);
    start_deg = result.Inputs.initialState.position_deg;
    goal_deg = goalPositionAtTime( ...
        result.Inputs.goalState, result.Inputs.goalState.time_s);
    scatter(workspaceAxes, start_deg(1), start_deg(2), 50, "g", ...
        "filled", "DisplayName", "Start");
    scatter(workspaceAxes, goal_deg(1), goal_deg(2), 50, "r", ...
        "filled", "DisplayName", "Goal");
    xlabel(workspaceAxes, "Azimuth (deg)");
    ylabel(workspaceAxes, "Elevation (deg)");
    title(workspaceAxes, diagnosticTitle(result, options.Title));
    legend(workspaceAxes, "Location", "best");
    handles.WorkspaceFigure = workspaceFigure;
    handles.WorkspaceAxes = workspaceAxes;
end

%% Section 3: Plot Time-Expanded Visibility Diagnostics

if options.ShowVisibilityGraphs
    visibilityFigure = figure( ...
        "Name", options.Title + " visibility diagnostics", ...
        "Visible", options.FigureVisible);
    visibilityAxes = axes(visibilityFigure);
    hold(visibilityAxes, "on");
    grid(visibilityAxes, "on");
    box(visibilityAxes, "on");
    gridRecord = result.SearchDiagnostics.Grid;
    layerTimes_s = visibilityLayerTimes(gridRecord, result.Inputs);
    layerIndices = sampledIndices(numel(layerTimes_s), ...
        options.MaximumDisplayedVisibilitySnapshots);
    if options.ShowSweptSurfaces
        drawObstacleLayers(visibilityAxes, result.Inputs.obstacles, ...
            layerTimes_s(layerIndices), ...
            options.MaximumDisplayedSlicesPerObstacle);
    end
    if isfield(gridRecord, "NodePosition_deg") && ...
            ~isempty(gridRecord.NodePosition_deg)
        nodePosition_deg = gridRecord.NodePosition_deg;
        for layerIndex = reshape(layerIndices, 1, [])
            scatter3(visibilityAxes, nodePosition_deg(:, 1), ...
                nodePosition_deg(:, 2), ...
                repmat(layerTimes_s(layerIndex), ...
                size(nodePosition_deg, 1), 1), ...
                5, [0.55 0.55 0.55], "filled", ...
                "HandleVisibility", "off");
        end
    end
    for seedIndex = 1:numel(result.Seeds)
        seed = result.Seeds(seedIndex);
        seedTime_s = result.Inputs.initialState.time_s + seed.tau * ...
            (result.Inputs.goalState.time_s - ...
            result.Inputs.initialState.time_s);
        plot3(visibilityAxes, seed.position_deg(:, 1), ...
            seed.position_deg(:, 2), seedTime_s, ...
            ":", "Color", [0.45 0.45 0.45], ...
            "DisplayName", "Seed " + seedIndex);
    end
    if result.Success
        plot3(visibilityAxes, result.position_deg(:, 1), ...
            result.position_deg(:, 2), result.time_s, ...
            "b-", "LineWidth", 2.2, ...
            "DisplayName", "Validated HS3 motion");
    end
    xlabel(visibilityAxes, "Azimuth (deg)");
    ylabel(visibilityAxes, "Elevation (deg)");
    zlabel(visibilityAxes, "Time (s)");
    title(visibilityAxes, diagnosticTitle(result, options.Title));
    view(visibilityAxes, 3);
    legend(visibilityAxes, "Location", "best");
    handles.VisibilityFigure = visibilityFigure;
    handles.VisibilityAxes = visibilityAxes;
    handles.VisibilityGraphs = struct( ...
        "Figure", visibilityFigure, "Axes", visibilityAxes);
end

%% Section 4: Plot Returned Kinematics

if options.ShowKinematics && result.Success
    kinematicFigure = figure( ...
        "Name", options.Title + " kinematics", ...
        "Visible", options.FigureVisible);
    tiledLayout = tiledlayout(kinematicFigure, 4, 1, ...
        "TileSpacing", "compact", "Padding", "compact");
    quantityNames = ["position_deg", "velocity_deg_s", ...
        "acceleration_deg_s2", "jerk_deg_s3"];
    yLabels = ["Position (deg)", "Velocity (deg/s)", ...
        "Acceleration (deg/s^2)", "Jerk (deg/s^3)"];
    limitValues = {[], result.Inputs.limits.maxVelocity_deg_s, ...
        result.Inputs.limits.maxAcceleration_deg_s2, ...
        result.Inputs.limits.maxJerk_deg_s3};
    axesHandles = gobjects(4, 1);
    for quantityIndex = 1:4
        axesHandles(quantityIndex) = nexttile(tiledLayout);
        axesHandle = axesHandles(quantityIndex);
        hold(axesHandle, "on");
        grid(axesHandle, "on");
        box(axesHandle, "on");
        values = result.(quantityNames(quantityIndex));
        plot(axesHandle, result.time_s, values(:, 1), ...
            "DisplayName", "Azimuth");
        plot(axesHandle, result.time_s, values(:, 2), ...
            "DisplayName", "Elevation");
        if ~isempty(limitValues{quantityIndex})
            for axisIndex = 1:2
                yline(axesHandle, limitValues{quantityIndex}(axisIndex), ...
                    "--", "HandleVisibility", "off");
                yline(axesHandle, -limitValues{quantityIndex}(axisIndex), ...
                    "--", "HandleVisibility", "off");
            end
        end
        ylabel(axesHandle, yLabels(quantityIndex));
    end
    xlabel(axesHandles(end), "Time (s)");
    legend(axesHandles(1), "Location", "best");
    title(tiledLayout, options.Title);
    handles.KinematicFigure = kinematicFigure;
    handles.KinematicAxes = axesHandles;
    handles.KinematicsFigure = kinematicFigure;
    handles.KinematicsAxes = axesHandles;
end

%% Section 5: Animate Returned Motion

if options.ShowAnimation && result.Success
    animationFigure = figure( ...
        "Name", options.Title + " animation", ...
        "Visible", options.FigureVisible);
    animationAxes = axes(animationFigure);
    frameIndices = unique([ ...
        1:options.FrameStride:numel(result.time_s), ...
        numel(result.time_s)]);
    for frameIndex = frameIndices
        cla(animationAxes);
        configureSpatialAxes(animationAxes);
        drawObstacles(animationAxes, result.Inputs.obstacles, ...
            result.time_s(frameIndex), false);
        drawTargetTrack(animationAxes, result.Inputs.goalState);
        plot(animationAxes, result.position_deg(1:frameIndex, 1), ...
            result.position_deg(1:frameIndex, 2), "b-", ...
            "LineWidth", 1.5, "DisplayName", "Motion history");
        scatter(animationAxes, result.position_deg(frameIndex, 1), ...
            result.position_deg(frameIndex, 2), 45, "b", "filled", ...
            "DisplayName", "Current position");
        xlabel(animationAxes, "Azimuth (deg)");
        ylabel(animationAxes, "Elevation (deg)");
        title(animationAxes, sprintf("%s | t = %.3f s", ...
            options.Title, result.time_s(frameIndex)));
        drawnow;
        if options.FigureVisible == "on" && options.Pause_s > 0
            pause(options.Pause_s);
        end
    end
    handles.AnimationFigure = animationFigure;
    handles.AnimationAxes = animationAxes;
    handles.Animation = struct( ...
        "Figure", animationFigure, "Axes", animationAxes);
end
end

%% Section 6: Local Functions

function options = normalizePlotAliases(options)
% PURPOSE
%   - Forward deprecated display spellings through one compatibility map.
if ~isstruct(options) || ~isscalar(options)
    error("plotAzElMotion:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
aliases = [ ...
    "AnimationFrameStride", "FrameStride"; ...
    "ShowKinematicPlot", "ShowKinematics"; ...
    "AnimationPause_s", "Pause_s"];
for aliasIndex = 1:size(aliases, 1)
    oldName = aliases(aliasIndex, 1);
    newName = aliases(aliasIndex, 2);
    if isfield(options, oldName)
        if ~isfield(options, newName)
            options.(newName) = options.(oldName);
        end
        options = rmfield(options, oldName);
    end
end
compatibilityNames = ["JerkConstraintEnabled", "MaxJerk_deg_s3", ...
    "ConfiguredFiniteMaxJerk_deg_s3", "PlotOptions"];
for name = compatibilityNames
    if isfield(options, name)
        options = rmfield(options, name);
    end
end
end

function configureSpatialAxes(axesHandle)
% PURPOSE
%   - Apply one explicit spatial-axis style to workspace and animation axes.
hold(axesHandle, "on");
grid(axesHandle, "on");
box(axesHandle, "on");
axis(axesHandle, "equal");
end

function drawSearchEdges(axesHandle, gridRecord)
% PURPOSE
%   - Draw retained accepted and collision-rejected visibility tests.
if isfield(gridRecord, "AcceptedEdges_deg") && ...
        ~isempty(gridRecord.AcceptedEdges_deg)
    [azimuth_deg, elevation_deg] = edgeLineData( ...
        gridRecord.AcceptedEdges_deg);
    plot(axesHandle, azimuth_deg, elevation_deg, ...
        "-", "Color", [0.30 0.75 0.78], "LineWidth", 0.4, ...
        "DisplayName", "Accepted visibility edge");
end
if isfield(gridRecord, "RejectedEdges_deg") && ...
        ~isempty(gridRecord.RejectedEdges_deg)
    [azimuth_deg, elevation_deg] = edgeLineData( ...
        gridRecord.RejectedEdges_deg);
    plot(axesHandle, azimuth_deg, elevation_deg, ...
        ":", "Color", [0.90 0.55 0.55], "LineWidth", 0.4, ...
        "DisplayName", "Collision-rejected edge");
end
end

function [azimuth_deg, elevation_deg] = edgeLineData(edges_deg)
% PURPOSE
%   - Convert N-by-4 edge endpoints to NaN-separated plot vectors.
edgeCount = size(edges_deg, 1);
azimuth_deg = reshape([ ...
    edges_deg(:, 1), edges_deg(:, 3), nan(edgeCount, 1)].', [], 1);
elevation_deg = reshape([ ...
    edges_deg(:, 2), edges_deg(:, 4), nan(edgeCount, 1)].', [], 1);
end

function drawObstacles(axesHandle, obstacles, time_s, showOriginal)
% PURPOSE
%   - Draw original and protected geometry from one canonical source.
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    if showOriginal
        originalObstacle = obstacle;
        originalObstacle.az_deg = obstacle.originalAz_deg;
        originalObstacle.el_deg = obstacle.originalEl_deg;
        originalShape = azElInternal.obstacleShapeAtTime( ...
            originalObstacle, time_s);
        drawShape(axesHandle, originalShape, [0.6 0.6 0.6], "--", ...
            "Original obstacle");
    end
    protectedShape = azElInternal.obstacleShapeAtTime(obstacle, time_s);
    drawShape(axesHandle, protectedShape, [0.8 0.2 0.2], "-", ...
        "Protected obstacle");
end
end

function drawShape(axesHandle, shape, color, lineStyle, displayName)
% PURPOSE
%   - Draw each NaN-separated polyshape boundary on explicit axes.
if isempty(shape.Vertices)
    return;
end
[azimuth_deg, elevation_deg] = boundary(shape);
plot(axesHandle, azimuth_deg, elevation_deg, ...
    "Color", color, "LineStyle", lineStyle, ...
    "LineWidth", 1.2, "DisplayName", displayName);
end

function drawObstacleLayers(axesHandle, obstacles, times_s, maximumCount)
% PURPOSE
%   - Draw protected obstacle slices in azimuth/elevation/time space.
timeIndices = sampledIndices(numel(times_s), maximumCount);
isFirstShape = true;
for timeIndex = reshape(timeIndices, 1, [])
    for obstacleIndex = 1:numel(obstacles)
        shape = azElInternal.obstacleShapeAtTime( ...
            obstacles(obstacleIndex), times_s(timeIndex));
        if isempty(shape.Vertices)
            continue;
        end
        [azimuth_deg, elevation_deg] = boundary(shape);
        displayName = "";
        visibility = "off";
        if isFirstShape
            displayName = "Protected obstacle slices";
            visibility = "on";
            isFirstShape = false;
        end
        plot3(axesHandle, azimuth_deg, elevation_deg, ...
            repmat(times_s(timeIndex), size(azimuth_deg)), ...
            "-", "Color", [0.85 0.35 0.35], "LineWidth", 0.7, ...
            "DisplayName", displayName, "HandleVisibility", visibility);
    end
end
end

function drawTargetTrack(axesHandle, goalState)
% PURPOSE
%   - Draw a sampled moving target when the result contains one.
if isfield(goalState, "targetPosition_deg") && ...
        ~isempty(goalState.targetPosition_deg)
    plot(axesHandle, goalState.targetPosition_deg(:, 1), ...
        goalState.targetPosition_deg(:, 2), "c-.", ...
        "LineWidth", 1.1, "DisplayName", "Moving target track");
end
end

function position_deg = goalPositionAtTime(goalState, time_s)
% PURPOSE
%   - Evaluate fixed or sampled goal geometry for display.
if isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s)
    position_deg = interp1( ...
        goalState.targetTime_s, goalState.targetPosition_deg, ...
        time_s, goalState.InterpolationMethod);
else
    position_deg = goalState.position_deg;
end
end

function times_s = visibilityLayerTimes(gridRecord, inputs)
% PURPOSE
%   - Select retained temporal layers or the planning endpoints.
times_s = zeros(0, 1);
if isfield(gridRecord, "TemporalLayerTimes_s")
    times_s = gridRecord.TemporalLayerTimes_s;
end
if isempty(times_s) && isfield(gridRecord, "SampleTimes_s")
    times_s = gridRecord.SampleTimes_s;
end
if isempty(times_s)
    times_s = [inputs.initialState.time_s; inputs.goalState.time_s];
end
times_s = unique(times_s(:));
end

function indices = sampledIndices(count, maximumCount)
% PURPOSE
%   - Retain evenly distributed display indices without changing counts.
if count <= 0
    indices = zeros(0, 1);
elseif count <= maximumCount
    indices = (1:count).';
else
    indices = unique(round(linspace(1, count, maximumCount))).';
end
end

function titleText = diagnosticTitle(result, prefix)
% PURPOSE
%   - Include the termination reason and complete key search counts.
gridRecord = result.SearchDiagnostics.Grid;
expanded = fieldOrZero(gridRecord, "ExpandedCount");
rejected = fieldOrZero(gridRecord, "RejectedTransitionCount");
titleText = sprintf("%s | %s | seeds %d | expanded %d | rejected %d", ...
    prefix, result.TerminationReason, numel(result.Seeds), ...
    expanded, rejected);
end

function value = fieldOrZero(record, fieldName)
% PURPOSE
%   - Read one optional diagnostic count without plot-time reconstruction.
value = 0;
if isfield(record, fieldName)
    value = record.(fieldName);
end
end

function handles = emptyHandles(options)
% PURPOSE
%   - Define stable empty graphics output for every display combination.
handles = struct( ...
    "WorkspaceFigure", gobjects(0), ...
    "WorkspaceAxes", gobjects(0), ...
    "VisibilityFigure", gobjects(0), ...
    "VisibilityAxes", gobjects(0), ...
    "VisibilityGraphs", struct(), ...
    "KinematicFigure", gobjects(0), ...
    "KinematicAxes", gobjects(0), ...
    "KinematicsFigure", gobjects(0), ...
    "KinematicsAxes", gobjects(0), ...
    "AnimationFigure", gobjects(0), ...
    "AnimationAxes", gobjects(0), ...
    "Animation", struct(), ...
    "Options", options);
end
