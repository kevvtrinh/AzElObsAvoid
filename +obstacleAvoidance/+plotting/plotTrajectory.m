function handles = plotTrajectory(result, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.plotting.plotTrajectory()
%   handles = obstacleAvoidance.plotting.plotTrajectory(result)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, optionOverrides)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, axesHandle)
%**************************************************************************
% PURPOSE
%   - Plot retained core geometry, visibility data, returned motion, and
%     physical limits without rerunning the planner.
%   - Animate returned samples against time-varying obstacles and targets.
%**************************************************************************
% INPUTS
%   - result (scalar planner result)
%       Successful or failed public result containing retained core data.
%   - optionOverrides (scalar struct or Cartesian axes handle, optional)
%       Display, animation, and GIF controls. An axes handle requests only
%       the workspace plot. Hidden figures never pause.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Stable workspace, visibility, kinematic, and animation handles plus
%       the established core Axes alias. Invalid input throws an error.
%   - options (scalar struct, zero-input call)
%       Default plotting controls.
%**************************************************************************
% UNITS
%   - Axes use coordinate units, seconds, units/s, units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Resolve Display Controls

defaults = struct();
defaults.FigureVisible                       = "on";
defaults.Title                               = "X/Y motion plan";
defaults.ShowWorkspace                       = true;
defaults.ShowKinematics                      = true;
defaults.ShowAnimation                       = true;
defaults.ShowSearchEdges                     = true;
defaults.ShowVisibilityGraphs                = true;
defaults.FrameStride                         = 5;
defaults.Pause_s                             = 0.001;
defaults.SaveAnimationGif                    = false;
defaults.AnimationGifFile                    = "obstacleAvoidanceTrajectory.gif";
defaults.AnimationGifDelay_s                 = 0.01;
if nargin == 0
    handles = defaults;
    return
end
if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end
requiredNames = {'Inputs', 'Options', 'Limits', 'RequestedLimits', 'Success', ...
    'TerminationReason', 'PreparedObstacles', 'VisibilityGraph', 'Route_units', ...
    'time_s', 'position_units', 'velocity_units_s', 'acceleration_units_s2', 'jerk_units_s3'};
if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, requiredNames))
    error("plotTrajectory:InvalidResult", "result must be a scalar planner result.");
end
workspaceAxesHandle = gobjects(0);
if ~isstruct(optionOverrides)
    if ~isscalar(optionOverrides) || ~isgraphics(optionOverrides, "axes")
        error("plotTrajectory:InvalidAxes", "The second input must be plot options or Cartesian axes.");
    end
    workspaceAxesHandle = optionOverrides;
    optionOverrides     = struct('ShowKinematics', false, 'ShowAnimation', false);
end
useSuppliedAxes = ~isempty(workspaceAxesHandle);
if ~isscalar(optionOverrides)
    error("plotTrajectory:InvalidOptions", "optionOverrides must be a scalar struct.");
end
[options, unknownNames] = obstacleAvoidance.input.resolveOptions(defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("plotTrajectory:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
options.FigureVisible    = lower(string(options.FigureVisible));
options.Title            = string(options.Title);
options.AnimationGifFile = string(options.AnimationGifFile);
if ~isscalar(options.FigureVisible) || ~any(options.FigureVisible == ["on", "off"])
    error("plotTrajectory:InvalidFigureVisible", "FigureVisible must be 'on' or 'off'.");
end
if ~isscalar(options.Title) || ~isscalar(options.AnimationGifFile) || strlength(options.AnimationGifFile) == 0
    error("plotTrajectory:InvalidTextOption", "Title/file must be nonempty scalar text.");
end
logicalOptionNames = ["ShowWorkspace", "ShowKinematics", "ShowAnimation", ...
    "ShowSearchEdges", "ShowVisibilityGraphs", "SaveAnimationGif"];
for optionName = logicalOptionNames
    options.(optionName) = obstacleAvoidance.input.normalizeLogicalScalar( ...
        options.(optionName), optionName, "plotTrajectory:InvalidLogicalOption");
end
nonnegativeOptionNames = ["Pause_s", "AnimationGifDelay_s"];
for optionName = nonnegativeOptionNames
    validateattributes(options.(optionName), {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
end
validateattributes(options.FrameStride, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
handles         = createEmptyHandles(options);
% A raw failure keeps the declaration of the trial that failed, so its goal
% clock is that trial's, not the horizon the caller asked about. Plot the
% requested horizon when the record still carries it.
plotHorizon_s = result.Inputs.goalState.time_s;
% Only a failed child request keeps its parent request; widen the window
% to the outer clock and never shrink it.
if ~result.Success && isfield(result, 'ParentRequest')
    outerHorizon_s = result.ParentRequest.GoalTime_s;
    if isnumeric(outerHorizon_s) && isscalar(outerHorizon_s) && ...
            isreal(outerHorizon_s) && isfinite(outerHorizon_s)
        plotHorizon_s = max(plotHorizon_s, double(outerHorizon_s));
    end
end
plotTimeRange_s = [result.Inputs.initialState.time_s, plotHorizon_s];
obstacles       = obstacleAvoidance.obstacles.prepareObstacles(result.PreparedObstacles, plotTimeRange_s);
originalObstacles = obstacles;
if options.ShowWorkspace || options.ShowVisibilityGraphs || options.ShowAnimation || options.SaveAnimationGif
    % The shared evaluator draws moving-cell-cell unions inside their intervals.
    % Reuse protected geometry verbatim. Cache the original histories once
    % for display, without constructing or applying another safety margin.
    for obstacleIndex = 1:numel(originalObstacles)
        originalObstacles(obstacleIndex).x_units = obstacles(obstacleIndex).originalX_units;
        originalObstacles(obstacleIndex).y_units = obstacles(obstacleIndex).originalY_units;
        originalObstacles(obstacleIndex).safetyMargin_units = 0;
        originalObstacles(obstacleIndex).InternalPreparation = struct();
    end
    originalObstacles = obstacleAvoidance.obstacles.prepareObstacles(originalObstacles, plotTimeRange_s);
end

%% Section 2: Plot Workspace And Failure Diagnostics

if options.ShowWorkspace
    if isempty(workspaceAxesHandle)
        workspaceFigureHandle = figure("Name", options.Title, "Visible", options.FigureVisible);
        workspaceAxesHandle   = axes(workspaceFigureHandle);
    else
        workspaceFigureHandle = ancestor(workspaceAxesHandle, 'figure');
        cla(workspaceAxesHandle);
    end
    configureSpatialAxes(workspaceAxesHandle, result);
    if useSuppliedAxes
        xlim(workspaceAxesHandle, result.RequestedLimits.xInterval_units);
        ylim(workspaceAxesHandle, result.RequestedLimits.yInterval_units);
    end
    drawObstacles(workspaceAxesHandle, obstacles, originalObstacles, result.Inputs.initialState.time_s);
    if options.ShowVisibilityGraphs
        drawSearchDiagnostics(workspaceAxesHandle, result, options.ShowSearchEdges);
    end
    drawPlannerRoute(workspaceAxesHandle, result);
    drawTarget(workspaceAxesHandle, result, result.Inputs.initialState.time_s);
    drawEndpoints(workspaceAxesHandle, result);
    finishAxes(workspaceAxesHandle, result, options.Title);
    handles.WorkspaceFigure = workspaceFigureHandle;
    handles.WorkspaceAxes   = workspaceAxesHandle;
end
pathCrossesSeam = false;
if result.Success && (result.Options.WrapX || result.Options.WrapY)
    seamPath_units = displayPath(result, result.position_units);
    pathCrossesSeam = any(isnan(seamPath_units(:, 1)));
end
createsSpatialDisplay = options.ShowWorkspace || options.ShowVisibilityGraphs || ...
    options.ShowAnimation || options.SaveAnimationGif;
if createsSpatialDisplay && pathCrossesSeam && ~useSuppliedAxes
    [handles.ContinuousWorkspaceFigure, handles.ContinuousWorkspaceAxes] = createContinuousWorkspace(result, options);
end

%% Section 3: Plot The Retained Visibility Graph

if options.ShowVisibilityGraphs
    if options.ShowWorkspace
        handles.VisibilityFigure = handles.WorkspaceFigure;
        handles.VisibilityAxes   = handles.WorkspaceAxes;
    else
        handles.VisibilityFigure = figure("Name", options.Title + " search", "Visible", options.FigureVisible);
        handles.VisibilityAxes   = axes(handles.VisibilityFigure);
        configureSpatialAxes(handles.VisibilityAxes, result);
        drawObstacles(handles.VisibilityAxes, obstacles, originalObstacles, result.Inputs.initialState.time_s);
        drawSearchDiagnostics(handles.VisibilityAxes, result, options.ShowSearchEdges);
        drawPlannerRoute(handles.VisibilityAxes, result);
        drawTarget(handles.VisibilityAxes, result, result.Inputs.initialState.time_s);
        drawEndpoints(handles.VisibilityAxes, result);
        finishAxes(handles.VisibilityAxes, result, options.Title);
    end
end

%% Section 4: Plot Returned Kinematics

if options.ShowKinematics && result.Success
    kinematicFigureHandle = figure("Name", options.Title + " kinematics", "Visible", options.FigureVisible);
    kinematicLayoutHandle = tiledlayout( ...
        kinematicFigureHandle, 4, 1, "TileSpacing", "compact", "Padding", "compact");
    kinematicAxesHandles = createKinematicPanels(kinematicLayoutHandle, result, false);
    title(kinematicLayoutHandle, options.Title);
    handles.KinematicFigure = kinematicFigureHandle;
    handles.KinematicAxes   = kinematicAxesHandles;
end

%% Section 5: Animate Returned Motion

if (options.ShowAnimation || options.SaveAnimationGif) && result.Success
    animationVisibility = options.FigureVisible;
    if options.SaveAnimationGif
        animationVisibility = "on";
    end
    animationFigureHandle = figure("Name", options.Title + " animation", "Visible", animationVisibility);
    animationLayoutHandle = tiledlayout( ...
        animationFigureHandle, 4, 2, "TileSpacing", "compact", "Padding", "compact");
    animationAxesHandle   = nexttile(animationLayoutHandle, 1, [4 1]);
    kinematicAxesHandles  = createKinematicPanels(animationLayoutHandle, result, true);
    animationLegendHandle = legend(kinematicAxesHandles(1), "Location", "best");
    frameIndices          = unique([1:options.FrameStride:numel(result.time_s), numel(result.time_s)]);
    [completePath_units, sourceIndex] = displayPath(result, result.position_units);
    gifFrameCount = 0;
    for frameIndex = frameIndices
        cla(animationAxesHandle);
        configureSpatialAxes(animationAxesHandle, result);
        drawObstacles(animationAxesHandle, obstacles, originalObstacles, result.time_s(frameIndex));
        drawTarget(animationAxesHandle, result, result.time_s(frameIndex));
        drawLine(animationAxesHandle, completePath_units, "-", "Complete timed path", 1);
        elapsedEndIndex = find(sourceIndex <= frameIndex, 1, "last");
        elapsedPath_units = completePath_units(1:elapsedEndIndex, :);
        currentPosition_units = displayPath(result, result.position_units(frameIndex, :));
        drawLine(animationAxesHandle, elapsedPath_units, "c-", "Elapsed path", 3);
        scatter(animationAxesHandle, currentPosition_units(1), currentPosition_units(2), ...
            60, [0.95 0.25 0.15], "filled", "DisplayName", "Current state");
        xlabel(animationAxesHandle, "X (units)");
        ylabel(animationAxesHandle, "Y (units)");
        title(animationAxesHandle, sprintf("%s | t = %.3f s", options.Title, result.time_s(frameIndex)));
        drawnow
        if options.SaveAnimationGif
            [indexedImage, colorMap] = rgb2ind(frame2im(getframe(animationFigureHandle)), 256);
            if gifFrameCount == 0
                imwrite(indexedImage, colorMap, char(options.AnimationGifFile), ...
                    "gif", "LoopCount", inf, "DelayTime", options.AnimationGifDelay_s);
            else
                imwrite(indexedImage, colorMap, char(options.AnimationGifFile), ...
                    "gif", "WriteMode", "append", "DelayTime", options.AnimationGifDelay_s);
            end
            gifFrameCount = gifFrameCount + 1;
        end
        if options.FigureVisible == "on" && options.Pause_s > 0
            pause(options.Pause_s);
        end
    end
    if options.FigureVisible == "off"
        animationFigureHandle.Visible = "off";
    end
    handles.AnimationFigure        = animationFigureHandle;
    handles.AnimationAxes          = animationAxesHandle;
    handles.AnimationKinematicAxes = kinematicAxesHandles;
    handles.AnimationLegend        = animationLegendHandle;
    if options.SaveAnimationGif
        handles.AnimationGifFile = options.AnimationGifFile;
    end
end

% Preserve the core's existing handles for callers that use one spatial view.
handles.Axes = handles.WorkspaceAxes;
if isempty(handles.Axes)
    handles.Axes = handles.VisibilityAxes;
end
end

%% Section 6: Local Functions

function configureSpatialAxes(axesHandle, result)
    % Fit ordinary scenes; keep wrapped views in the requested workspace.
    hold(axesHandle, "on");
    grid(axesHandle, "on");
    box(axesHandle, "on");
    axis(axesHandle, "equal");
    if result.Options.WrapX || result.Options.WrapY
        xlim(axesHandle, result.RequestedLimits.xInterval_units);
        ylim(axesHandle, result.RequestedLimits.yInterval_units);
    end
end

function [position_units, sourceIndex] = displayPath(result, position_units)
    % Apply the result's wrap settings to the displayed path.
    intervals_units = [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units];
    wrapAxes = [result.Options.WrapX result.Options.WrapY];
    [position_units, sourceIndex] = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        position_units, intervals_units, wrapAxes);
end

function [figureHandle, axesHandle] = createContinuousWorkspace(result, options)
    % Show the unwrapped path and crossed wrap boundaries.
    figureHandle = figure("Name", options.Title + " continuous coordinates", "Visible", options.FigureVisible);
    axesHandle   = axes(figureHandle);
    hold(axesHandle, "on");
    grid(axesHandle, "on");
    box(axesHandle, "on");
    axis(axesHandle, "equal");
    drawLine(axesHandle, result.position_units, "k-", "Timed motion", 2);
    intervals_units = [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units];
    wrapAxes         = [result.Options.WrapX result.Options.WrapY];
    for axisIndex = find(wrapAxes)
        interval_units  = intervals_units(axisIndex, :);
        period_units    = diff(interval_units);
        firstMultiplier = ceil((min(result.position_units(:, axisIndex)) - interval_units(1)) / period_units);
        lastMultiplier  = floor((max(result.position_units(:, axisIndex)) - interval_units(1)) / period_units);
        seamMultipliers = firstMultiplier:lastMultiplier;
        for seam_units = interval_units(1) + period_units * seamMultipliers
            if axisIndex == 1
                xline(axesHandle, seam_units, ":", "HandleVisibility", "off");
            else
                yline(axesHandle, seam_units, ":", "HandleVisibility", "off");
            end
        end
    end
    xlabel(axesHandle, "Continuous x (units)");
    ylabel(axesHandle, "Continuous y (units)");
end

function drawPlannerRoute(axesHandle, result)
    % Failed solves can still retain a geometric guide or candidate motion.
    if ~isempty(result.Route_units)
        selectedRoute_units = displayPath(result, result.Route_units);
        drawLine(axesHandle, selectedRoute_units, "--", "Selected geometric route", 1);
    end
    if ~isempty(result.position_units)
        motionPath_units = displayPath(result, result.position_units);
        drawLine(axesHandle, motionPath_units, "k-", "Timed motion", 2);
    end
end

function drawEndpoints(axesHandle, result)
    % Draw the requested start and terminal positions under the wrap policy.
    start_units = displayPath(result, result.Inputs.initialState.position_units);
    goal_units  = result.Inputs.goalState.position_units;
    if result.Success && hasData(result, 'Intercept') && isfinite(result.Intercept.Time_s)
        goal_units = result.Intercept.TargetPosition_units;
    end
    goal_units = displayPath(result, goal_units);
    plot(axesHandle, start_units(1), start_units(2), "go", "DisplayName", "Start");
    plot(axesHandle, goal_units(1), goal_units(2), "ro", "DisplayName", "Goal");
end

function lineHandle = drawLine(axesHandle, position_units, lineStyle, displayName, lineWidth)
    % Draw one named spatial polyline with the requested style.
    lineHandle = plot(axesHandle, position_units(:, 1), position_units(:, 2), ...
        lineStyle, "LineWidth", lineWidth, "DisplayName", displayName);
end

function drawSearchDiagnostics(axesHandle, result, showEdges)
    % Draw only edges and nodes retained by the exact visibility search.
    graphRecord = result.VisibilityGraph;
    nodes_units = graphRecord.NodePosition_units;
    edgeNames   = ["AcceptedNodeIndex", "RejectedNodeIndex"];
    edgeStyles  = ["-", ":"];
    edgeLabels  = ["Accepted visibility edge", "Collision-rejected edge"];
    if showEdges
        for categoryIndex = 1:2
            if hasData(graphRecord, edgeNames(categoryIndex))
                edgeNodeIndices = graphRecord.(edgeNames(categoryIndex));
                edgeCount       = size(edgeNodeIndices, 1);
                x_units = reshape([nodes_units(edgeNodeIndices(:, 1), 1), ...
                    nodes_units(edgeNodeIndices(:, 2), 1), nan(edgeCount, 1)].', [], 1);
                y_units = reshape([nodes_units(edgeNodeIndices(:, 1), 2), ...
                    nodes_units(edgeNodeIndices(:, 2), 2), nan(edgeCount, 1)].', [], 1);
                if result.Options.WrapX || result.Options.WrapY
                    paths_units = cell(edgeCount, 1);
                    for edgeIndex = 1:edgeCount
                        edgeNode_units = nodes_units(edgeNodeIndices(edgeIndex, :), :);
                        paths_units{edgeIndex} = [displayPath(result, edgeNode_units); NaN NaN];
                    end
                    paths_units = vertcat(paths_units{:});
                    x_units     = paths_units(:, 1);
                    y_units     = paths_units(:, 2);
                end
                plot(axesHandle, x_units, y_units, edgeStyles(categoryIndex), ...
                    "DisplayName", edgeLabels(categoryIndex));
            end
        end
    end
    intervals_units = [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units];
    for axisIndex = find([result.Options.WrapX result.Options.WrapY])
        nodes_units(:, axisIndex) = intervals_units(axisIndex, 1) + ...
            mod(nodes_units(:, axisIndex) - intervals_units(axisIndex, 1), diff(intervals_units(axisIndex, :)));
    end
    if ~isempty(nodes_units)
        scatter(axesHandle, nodes_units(:, 1), nodes_units(:, 2), ...
            17, "filled", "DisplayName", "Visibility node");
    end
end

function drawObstacles(axesHandle, obstacles, originalObstacles, time_s)
    % Draw original and safety-adjusted geometry from retained obstacle histories.
    colors = lines(max(1, numel(obstacles)));
    for obstacleIndex = 1:numel(obstacles)
        obstacle = obstacles(obstacleIndex);
        originalShape = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
            originalObstacles(obstacleIndex), time_s);
        drawShape(axesHandle, originalShape, colors(obstacleIndex, :), "-", "Original obstacle");

        protectedShape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, time_s);
        drawShape(axesHandle, protectedShape, "none", "--", "Protected obstacle");
    end
end

function drawShape(axesHandle, shape, faceColor, lineStyle, displayName)
    % Draw one nonempty polygon with the requested face and line styling.
    if ~isempty(shape.Vertices)
        plot(axesHandle, shape, "FaceColor", faceColor, "FaceAlpha", 0.18, ...
            "LineStyle", lineStyle, "LineWidth", 1.2, "DisplayName", displayName);
    end
end

function drawTarget(axesHandle, result, displayTime_s)
    % Draw the moving target's track and current position.
    goalState = result.Inputs.goalState;
    if ~hasData(goalState, "targetMotion")
        return
    end
    targetMotion    = goalState.targetMotion;
    trackTime_s     = unique([targetMotion.time_s(:); ...
        linspace(targetMotion.time_s(1), targetMotion.time_s(end), 200).']);
    targetPath_units = displayPath( ...
        result, obstacleAvoidance.input.targetPositionAtTime(targetMotion, trackTime_s));
    drawLine(axesHandle, targetPath_units, "-.", "Moving target track", 1);
    if displayTime_s < targetMotion.time_s(1) || displayTime_s > targetMotion.time_s(end)
        return
    end
    targetPosition_units = obstacleAvoidance.input.targetPositionAtTime(targetMotion, displayTime_s);
    targetPosition_units = displayPath(result, targetPosition_units);
    plot(axesHandle, targetPosition_units(1), targetPosition_units(2), ...
        "md", "MarkerFaceColor", "m", "DisplayName", "Moving target");
end

function axesHandles = createKinematicPanels(layoutHandle, result, isAnimated)
    % Plot position, velocity, acceleration, and jerk with their limits.
    quantityNames = ["position_units", "velocity_units_s", "acceleration_units_s2", "jerk_units_s3"];
    yLabels       = ["Position (units)", "Velocity (units/s)", "Acceleration (units/s^2)", "Jerk (units/s^3)"];
    limits        = [nan(1, 2); result.Limits.maxVelocity_units_s; ...
        result.Limits.maxAcceleration_units_s2; result.Limits.maxJerk_units_s3];
    axesHandles = gobjects(4, 1);
    for quantityIndex = 1:4
        tileIndex = quantityIndex * (1 + isAnimated);
        axesHandles(quantityIndex) = nexttile(layoutHandle, tileIndex);
        axesHandle = axesHandles(quantityIndex);
        hold(axesHandle, "on");
        grid(axesHandle, "on");
        box(axesHandle, "on");
        values      = result.(quantityNames(quantityIndex));
        lineHandles = plot(axesHandle, result.time_s, values);
        set(lineHandles, {'DisplayName'}, {'X'; 'Y'});
        if quantityIndex > 1
            yline(axesHandle, [-limits(quantityIndex, :), limits(quantityIndex, :)], ...
                "r--", "HandleVisibility", "off");
        end
        ylabel(axesHandle, yLabels(quantityIndex));
        xlim(axesHandle, result.time_s([1 end]));
    end
    xlabel(axesHandles(end), "Time (s)");
    legend(axesHandles(1), "Location", "best");
end

function finishAxes(axesHandle, result, titlePrefix)
    % Label the plot with the planner's actual termination reason.
    xlabel(axesHandle, "X (units)");
    ylabel(axesHandle, "Y (units)");
    title(axesHandle, sprintf("%s | %s", titlePrefix, result.TerminationReason));
    legend(axesHandle, "Location", "best");
end

function hasValue = hasData(record, fieldName)
    % Report whether the named field exists and contains data.
    hasValue = isfield(record, fieldName) && ~isempty(record.(fieldName));
end

function handles = createEmptyHandles(options)
    % Initialize every stable output field with an empty graphics handle.
    emptyHandles = gobjects(0);
    handles = struct( ...
        "WorkspaceFigure",            emptyHandles, ...
        "WorkspaceAxes",              emptyHandles, ...
        "ContinuousWorkspaceFigure",  emptyHandles, ...
        "ContinuousWorkspaceAxes",    emptyHandles, ...
        "VisibilityFigure",           emptyHandles, ...
        "VisibilityAxes",             emptyHandles, ...
        "KinematicFigure",            emptyHandles, ...
        "KinematicAxes",              emptyHandles, ...
        "AnimationFigure",            emptyHandles, ...
        "AnimationAxes",              emptyHandles, ...
        "AnimationKinematicAxes",     emptyHandles, ...
        "AnimationLegend",            emptyHandles, ...
        "AnimationGifFile",           "", ...
        "Options",                    options, ...
        "Axes",                       emptyHandles);
end
