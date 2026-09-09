function handles = plotTrajectory(result, optionOverrides, ~)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.plotting.plotTrajectory()
%   handles = obstacleAvoidance.plotting.plotTrajectory(result)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, optionOverrides)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, optionOverrides, diagnosis)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, axesHandle)
%
% PURPOSE
%   - Plot retained core geometry, visibility graph, motion, and physical limits.
%   - Animate returned samples against time-varying obstacles and targets.
%
% INPUTS
%   - result (scalar planner result)
%       Success or failure record; plotting never reruns the planner.
%   - optionOverrides (scalar struct, optional; default struct())
%       Display, animation, and GIF controls. Hidden figures never pause.
%       A Cartesian axes handle instead selects a workspace-only plot.
%       ShowSeedPaths, ShowSweptSurfaces, and MaximumDisplayed* are retained
%       for example compatibility; the core has no such diagnostic histories.
%   - diagnosis (optional compatibility argument, unused)
%       The retained visibility graph is read directly from result.
%
% OUTPUTS
%   - handles (scalar struct)
%       Stable workspace, visibility, kinematic, and animation handles,
%       plus the core Axes, obstacle, graph, Route, Trajectory, Endpoint aliases.
%
% UNITS
%   - Axes use coordinate units, seconds, units/s, units/s^2, and units/s^3.
%

%% Section 1: Resolve Display Controls

defaults = struct();
defaults.FigureVisible                       = "on";
defaults.Title                               = "X/Y motion plan";
defaults.ShowWorkspace                       = true;
defaults.ShowKinematics                      = true;
defaults.ShowAnimation                       = true;
defaults.ShowSeedPaths                       = false;
defaults.ShowSearchEdges                     = true;
defaults.ShowVisibilityGraphs                = true;
defaults.FrameStride                         = 5;
defaults.Pause_s                             = 0.001;
defaults.SaveAnimationGif                    = false;
defaults.AnimationGifFile                    = "obstacleAvoidanceTrajectory.gif";
defaults.AnimationGifDelay_s                 = 0.01;
defaults.ShowSweptSurfaces                   = true;
defaults.MaximumDisplayedSlicesPerObstacle   = 30;
defaults.MaximumDisplayedVisibilitySnapshots = 30;
if nargin == 0
    handles = defaults;
    return;
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
workspaceAxes = gobjects(0);
if ~isstruct(optionOverrides)
    if ~isscalar(optionOverrides) || ~isgraphics(optionOverrides, "axes")
        error("plotTrajectory:InvalidAxes", "The second input must be plot options or Cartesian axes.");
    end
    workspaceAxes = optionOverrides;
    optionOverrides = struct('ShowKinematics', false, 'ShowAnimation', false);
end
useSuppliedAxes = ~isempty(workspaceAxes);
[options, unknownNames] = obstacleAvoidance.input.resolveOptions(defaults, normalizePlotAliases(optionOverrides));
if ~isempty(unknownNames)
    warning("plotTrajectory:UnknownOptions", "Ignoring unknown fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
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
logicalNames = ["ShowWorkspace", "ShowKinematics", "ShowAnimation", "ShowSeedPaths", ...
    "ShowSearchEdges", "ShowVisibilityGraphs", "ShowSweptSurfaces", "SaveAnimationGif"];
for name = logicalNames
    options.(name) = obstacleAvoidance.input.normalizeLogicalScalar(options.(name), name, "plotTrajectory:InvalidLogicalOption");
end
nonnegativeNames = ["Pause_s", "AnimationGifDelay_s"];
for name = nonnegativeNames
    validateattributes(options.(name), {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
end
validateattributes(options.FrameStride, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
for name = ["MaximumDisplayedSlicesPerObstacle", "MaximumDisplayedVisibilitySnapshots"]
    validateattributes(options.(name), {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
end
handles    = createEmptyHandles(options);
obstacles  = result.PreparedObstacles;
originalObstacles = obstacles;
if options.ShowWorkspace || options.ShowVisibilityGraphs || options.ShowAnimation || options.SaveAnimationGif
    % Reuse protected geometry verbatim. Cache the original histories once
    % for display, without constructing or applying another safety margin.
    for obstacleIndex = 1:numel(originalObstacles)
        originalObstacles(obstacleIndex).x_units = obstacles(obstacleIndex).originalX_units;
        originalObstacles(obstacleIndex).y_units = obstacles(obstacleIndex).originalY_units;
        originalObstacles(obstacleIndex).safetyMargin_units = 0;
        originalObstacles(obstacleIndex).InternalPreparation = struct();
    end
    originalObstacles = obstacleAvoidance.obstacles.prepareObstacles(originalObstacles);
end

%% Section 2: Plot Workspace And Failure Diagnostics

if options.ShowWorkspace
    if isempty(workspaceAxes)
        workspaceFigure = figure("Name", options.Title, "Visible", options.FigureVisible);
        workspaceAxes   = axes(workspaceFigure);
    else
        workspaceFigure = ancestor(workspaceAxes, 'figure');
        cla(workspaceAxes);
    end
    configureSpatialAxes(workspaceAxes, result);
    if useSuppliedAxes
        xlim(workspaceAxes, result.RequestedLimits.xInterval_units);
        ylim(workspaceAxes, result.RequestedLimits.yInterval_units);
    end
    drawObstacles(workspaceAxes, obstacles, originalObstacles, result.Inputs.initialState.time_s);
    if options.ShowVisibilityGraphs
        drawSearchDiagnostics(workspaceAxes, result, options.ShowSearchEdges);
    end
    drawPlannerRoute(workspaceAxes, result);
    drawTarget(workspaceAxes, result, result.Inputs.initialState.time_s);
    drawEndpoints(workspaceAxes, result);
    finishAxes(workspaceAxes, result, options.Title);
    handles.WorkspaceFigure = workspaceFigure;
    handles.WorkspaceAxes   = workspaceAxes;
end
doesCross = false;
if result.Success && (result.Options.WrapX || result.Options.WrapY)
    seamPath_units = displayPath(result, result.position_units);
    doesCross    = any(isnan(seamPath_units(:, 1)));
end
if (options.ShowWorkspace || options.ShowVisibilityGraphs || options.ShowAnimation || options.SaveAnimationGif) && doesCross && ~useSuppliedAxes
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
    handles.VisibilityGraphs = struct("Figure", handles.VisibilityFigure, "Axes", handles.VisibilityAxes);
end

%% Section 4: Plot Returned Kinematics

if options.ShowKinematics && result.Success
    kinematicFigure = figure("Name", options.Title + " kinematics", "Visible", options.FigureVisible);
    kinematicLayout = tiledlayout(kinematicFigure, 4, 1, "TileSpacing", "compact", "Padding", "compact");
    kinematicAxes   = createKinematicPanels(kinematicLayout, result, false);
    title(kinematicLayout, options.Title);
    handles.KinematicFigure  = kinematicFigure;
    handles.KinematicAxes    = kinematicAxes;
    handles.KinematicsFigure = kinematicFigure;
    handles.KinematicsAxes   = kinematicAxes;
end

%% Section 5: Animate Returned Motion

if (options.ShowAnimation || options.SaveAnimationGif) && result.Success
    animationVisibility = options.FigureVisible;
    if options.SaveAnimationGif
        animationVisibility = "on";
    end
    animationFigure = figure("Name", options.Title + " animation", "Visible", animationVisibility);
    animationLayout = tiledlayout(animationFigure, 4, 2, "TileSpacing", "compact", "Padding", "compact");
    animationAxes   = nexttile(animationLayout, 1, [4 1]);
    kinematicAxes   = createKinematicPanels(animationLayout, result, true);
    animationLegend = legend(kinematicAxes(1), "Location", "best");
    frameIndices    = unique([1:options.FrameStride:numel(result.time_s), numel(result.time_s)]);
    [complete_units, sourceIndex] = displayPath(result, result.position_units);
    gifFrameCount = 0;
    for frameIndex = frameIndices
        cla(animationAxes);
        configureSpatialAxes(animationAxes, result);
        drawObstacles(animationAxes, obstacles, originalObstacles, result.time_s(frameIndex));
        drawTarget(animationAxes, result, result.time_s(frameIndex));
        drawLine(animationAxes, complete_units, "-", "Complete timed path", 1);
        elapsedEnd  = find(sourceIndex <= frameIndex, 1, "last");
        elapsed_units = complete_units(1:elapsedEnd, :);
        current_units = displayPath(result, result.position_units(frameIndex, :));
        drawLine(animationAxes, elapsed_units, "c-", "Elapsed path", 3);
        scatter(animationAxes, current_units(1), current_units(2), 60, [0.95 0.25 0.15], "filled", "DisplayName", "Current state");
        xlabel(animationAxes, "X (units)");
        ylabel(animationAxes, "Y (units)");
        title(animationAxes, sprintf("%s | t = %.3f s", options.Title, result.time_s(frameIndex)));
        drawnow;
        if options.SaveAnimationGif
            [image, colorMap] = rgb2ind(frame2im(getframe(animationFigure)), 256);
            if gifFrameCount == 0
                imwrite(image, colorMap, char(options.AnimationGifFile), "gif", "LoopCount", inf, "DelayTime", options.AnimationGifDelay_s);
            else
                imwrite(image, colorMap, char(options.AnimationGifFile), "gif", "WriteMode", "append", "DelayTime", options.AnimationGifDelay_s);
            end
            gifFrameCount = gifFrameCount + 1;
        end
        if options.FigureVisible == "on" && options.Pause_s > 0
            pause(options.Pause_s);
        end
    end
    if options.FigureVisible == "off"
        animationFigure.Visible = "off";
    end
    handles.AnimationFigure        = animationFigure;
    handles.AnimationAxes          = animationAxes;
    handles.AnimationKinematicAxes = kinematicAxes;
    handles.AnimationLegend        = animationLegend;
    if options.SaveAnimationGif
        handles.AnimationGifFile = options.AnimationGifFile;
    end
    handles.Animation = struct("Figure", animationFigure, "Axes", animationAxes, ...
        "KinematicAxes", kinematicAxes, "CurrentKinematicMarkers", gobjects(0), ...
        "ElapsedKinematicLines", gobjects(0), "TimeCursors", gobjects(0), ...
        "Legend", animationLegend, "GifFile", handles.AnimationGifFile);
end

% Preserve the core's existing handles for callers that use one spatial view.
handles.Axes = handles.WorkspaceAxes;
if isempty(handles.Axes), handles.Axes = handles.VisibilityAxes; end
if ~isempty(handles.Axes)
    handles.OriginalObstacle = findobj(handles.Axes, 'DisplayName', 'Original obstacle');
    handles.ProtectedObstacle = findobj(handles.Axes, 'DisplayName', 'Protected obstacle');
    handles.VisibilityEdge = findobj(handles.Axes, 'DisplayName', 'Accepted visibility edge');
    handles.VisibilityNode = findobj(handles.Axes, 'DisplayName', 'Visibility node');
    handles.Route = findobj(handles.Axes, 'DisplayName', 'Selected geometric route');
    handles.Trajectory = findobj(handles.Axes, 'DisplayName', 'Timed motion');
    handles.Endpoint = [findobj(handles.Axes, 'DisplayName', 'Start'); findobj(handles.Axes, 'DisplayName', 'Goal')];
end
end

%% Section 6: Local Functions

function options = normalizePlotAliases(options)
    % Normalize the display aliases used by existing examples.
    if ~isstruct(options) || ~isscalar(options)
        error("plotTrajectory:InvalidOptions", "optionOverrides must be a scalar struct.");
    end
    aliases = ["AnimationFrameStride", "FrameStride"; ...
        "ShowKinematicPlot", "ShowKinematics"; "AnimationPause_s", "Pause_s"];
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
end

function configureSpatialAxes(axesHandle, result)
    % Fit ordinary scenes; keep periodic views in the requested workspace.
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
    [position_units, sourceIndex] = obstacleAvoidance.plotting.createWrappedSpatialPath(position_units, intervals_units, [result.Options.WrapX result.Options.WrapY]);
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
    wrapAxes = [result.Options.WrapX result.Options.WrapY];
    for axisIndex = find(wrapAxes)
        interval_units = intervals_units(axisIndex, :);
        period_units = diff(interval_units);
        seamMultipliers = ceil((min(result.position_units(:, axisIndex)) - interval_units(1)) / period_units):floor((max(result.position_units(:, axisIndex)) - interval_units(1)) / period_units);
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
        selected_units = displayPath(result, result.Route_units);
        drawLine(axesHandle, selected_units, "--", "Selected geometric route", 1);
    end
    if ~isempty(result.position_units)
        motion_units = displayPath(result, result.position_units);
        drawLine(axesHandle, motion_units, "k-", "Timed motion", 2);
    end
end

function drawEndpoints(axesHandle, result)
    % Draw the requested start and terminal positions under the wrap policy.
    start_units = displayPath(result, result.Inputs.initialState.position_units);
    goal_units = result.Inputs.goalState.position_units;
    if result.Success && hasData(result, 'Intercept') && isfinite(result.Intercept.Time_s)
        goal_units = result.Intercept.TargetPosition_units;
    end
    goal_units  = displayPath(result, goal_units);
    plot(axesHandle, start_units(1), start_units(2), "go", "DisplayName", "Start");
    plot(axesHandle, goal_units(1), goal_units(2), "ro", "DisplayName", "Goal");
end

function lineHandle = drawLine(axesHandle, position_units, style, name, width)
    lineHandle = plot(axesHandle, position_units(:, 1), position_units(:, 2), style, "LineWidth", width, "DisplayName", name);
end

function drawSearchDiagnostics(axesHandle, result, showEdges)
    % Draw only edges and nodes retained by the exact visibility search.
    graphRecord = result.VisibilityGraph;
    nodes_units = graphRecord.NodePosition_units;
    edgeNames  = ["AcceptedNodeIndex", "RejectedNodeIndex"];
    edgeStyles = ["-", ":"];
    edgeLabels = ["Accepted visibility edge", "Collision-rejected edge"];
    if showEdges
        for categoryIndex = 1:2
            if hasData(graphRecord, edgeNames(categoryIndex))
                indices = graphRecord.(edgeNames(categoryIndex));
                edgeCount = size(indices, 1);
                x_units = reshape([nodes_units(indices(:, 1), 1), nodes_units(indices(:, 2), 1), nan(edgeCount, 1)].', [], 1);
                y_units = reshape([nodes_units(indices(:, 1), 2), nodes_units(indices(:, 2), 2), nan(edgeCount, 1)].', [], 1);
                if result.Options.WrapX || result.Options.WrapY
                    paths_units = cell(edgeCount, 1);
                    for edgeIndex = 1:edgeCount
                        paths_units{edgeIndex} = [displayPath(result, nodes_units(indices(edgeIndex, :), :)); NaN NaN];
                    end
                    paths_units = vertcat(paths_units{:});
                    x_units = paths_units(:, 1); y_units = paths_units(:, 2);
                end
                plot(axesHandle, x_units, y_units, edgeStyles(categoryIndex), "DisplayName", edgeLabels(categoryIndex));
            end
        end
    end
    intervals_units = [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units];
    for axisIndex = find([result.Options.WrapX result.Options.WrapY])
        nodes_units(:, axisIndex) = intervals_units(axisIndex, 1) + ...
            mod(nodes_units(:, axisIndex) - intervals_units(axisIndex, 1), diff(intervals_units(axisIndex, :)));
    end
    if ~isempty(nodes_units)
        scatter(axesHandle, nodes_units(:, 1), nodes_units(:, 2), 17, "filled", "DisplayName", "Visibility node");
    end
end

function drawObstacles(axesHandle, obstacles, originalObstacles, time_s)
    % Draw original and safety-adjusted geometry from retained obstacle histories.
    colors = lines(max(1, numel(obstacles)));
    for obstacleIndex = 1:numel(obstacles)
        obstacle = obstacles(obstacleIndex);
        shape = obstacleAvoidance.obstacles.preparedShapeAtTime(originalObstacles(obstacleIndex), time_s);
        drawShape(axesHandle, shape, colors(obstacleIndex, :), "-", "Original obstacle");
        shape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, time_s);
        drawShape(axesHandle, shape, "none", "--", "Protected obstacle");
    end
end

function drawShape(axesHandle, shape, faceColor, style, name)
    if ~isempty(shape.Vertices)
        plot(axesHandle, shape, "FaceColor", faceColor, "FaceAlpha", 0.18, "LineStyle", style, "LineWidth", 1.2, "DisplayName", name);
    end
end

function drawTarget(axesHandle, result, displayTime_s)
    % Draw the moving target's track and current position.
    goalState = result.Inputs.goalState;
    if ~hasData(goalState, "targetMotion")
        return;
    end
    target = goalState.targetMotion;
    trackTime_s = unique([target.time_s(:); linspace(target.time_s(1), target.time_s(end), 200).']);
    track_units = displayPath(result, obstacleAvoidance.input.targetPositionAtTime(target, trackTime_s));
    drawLine(axesHandle, track_units, "-.", "Moving target track", 1);
    if displayTime_s < target.time_s(1) || displayTime_s > target.time_s(end)
        return;
    end
    target_units = obstacleAvoidance.input.targetPositionAtTime(target, displayTime_s);
    target_units = displayPath(result, target_units);
    plot(axesHandle, target_units(1), target_units(2), "md", "MarkerFaceColor", "m", "DisplayName", "Moving target");
end

function axesHandles = createKinematicPanels(layout, result, animated)
    % Plot position, velocity, acceleration, and jerk with their limits.
    quantityNames = ["position_units", "velocity_units_s", "acceleration_units_s2", "jerk_units_s3"];
    yLabels       = ["Position (units)", "Velocity (units/s)", "Acceleration (units/s^2)", "Jerk (units/s^3)"];
    limits        = [nan(1, 2); result.Limits.maxVelocity_units_s; ...
        result.Limits.maxAcceleration_units_s2; result.Limits.maxJerk_units_s3];
    axesHandles = gobjects(4, 1);
    for quantityIndex = 1:4
        tileIndex = quantityIndex * (1 + animated);
        axesHandles(quantityIndex) = nexttile(layout, tileIndex);
        axesHandle = axesHandles(quantityIndex);
        hold(axesHandle, "on");
        grid(axesHandle, "on");
        box(axesHandle, "on");
        values      = result.(quantityNames(quantityIndex));
        lineHandles = plot(axesHandle, result.time_s, values);
        set(lineHandles, {'DisplayName'}, {'X'; 'Y'});
        if quantityIndex > 1
            yline(axesHandle, [-limits(quantityIndex, :), limits(quantityIndex, :)], "r--", "HandleVisibility", "off");
        end
        ylabel(axesHandle, yLabels(quantityIndex));
        xlim(axesHandle, result.time_s([1 end]));
    end
    xlabel(axesHandles(end), "Time (s)");
    legend(axesHandles(1), "Location", "best");
end

function finishAxes(axesHandle, result, prefix)
    % Label the plot with the planner's actual termination reason.
    xlabel(axesHandle, "X (units)");
    ylabel(axesHandle, "Y (units)");
    title(axesHandle, sprintf("%s | %s", prefix, result.TerminationReason));
    legend(axesHandle, "Location", "best");
end

function value = hasData(record, fieldName)
    value = isfield(record, fieldName) && ~isempty(record.(fieldName));
end

function handles = createEmptyHandles(options)
    none    = gobjects(0);
    handles = struct("WorkspaceFigure", none, "WorkspaceAxes", none, ...
        "ContinuousWorkspaceFigure", none, "ContinuousWorkspaceAxes", none, ...
        "VisibilityFigure", none, "VisibilityAxes", none, "VisibilityGraphs", struct(), ...
        "KinematicFigure", none, "KinematicAxes", none, "KinematicsFigure", none, ...
        "KinematicsAxes", none, "AnimationFigure", none, "AnimationAxes", none, ...
        "AnimationKinematicAxes", none, "AnimationLegend", none, "AnimationGifFile", "", ...
        "Animation", struct(), "Options", options, "Axes", none, ...
        "OriginalObstacle", none, "ProtectedObstacle", none, ...
        "VisibilityEdge", none, "VisibilityNode", none, "Route", none, ...
        "Trajectory", none, "Endpoint", none);
end
