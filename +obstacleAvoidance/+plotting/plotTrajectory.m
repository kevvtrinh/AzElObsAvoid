function handles = plotTrajectory(result, optionOverrides, diagnosis)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.plotting.plotTrajectory()
%   handles = obstacleAvoidance.plotting.plotTrajectory(result)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, optionOverrides)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, optionOverrides, diagnosis)
%
% PURPOSE
%   - Plot retained motion, search diagnostics, and physical limits.
%   - Animate returned samples against time-varying obstacles and targets.
%
% INPUTS
%   - result (scalar planTrajectory result)
%       Success or failure record; plotting never reruns the planner.
%   - optionOverrides (scalar struct, optional; default struct())
%       Display, animation, and GIF controls. Hidden figures never pause.
%   - diagnosis (optional second planner output)
%       Adds candidate routes, search edges, and frontier diagnostics.
%
% OUTPUTS
%   - handles (scalar struct)
%       Stable workspace, visibility, kinematic, and animation handles.
%
% UNITS
%   - Axes use degrees, seconds, deg/s, deg/s^2, and deg/s^3.
%

%% Section 1: Resolve Display Controls

defaults = struct();
defaults.FigureVisible                       = "on";
defaults.Title                               = "Az/El motion plan";
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
if nargin < 3, diagnosis = struct(); end
requiredNames = {'Inputs', 'Options', 'Success', 'Route_deg', 'BestPartialRoute_deg'};
if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, requiredNames))
    error("plotTrajectory:InvalidResult", "result must be a scalar planner result.");
end
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
% Apply the required validation or transfer to each name.
for name = logicalNames
    options.(name) = obstacleAvoidance.input.normalizeLogicalScalar(options.(name), name, "plotTrajectory:InvalidLogicalOption");
end
nonnegativeNames = ["Pause_s", "AnimationGifDelay_s"];
% Apply the required validation or transfer to each name.
for name = nonnegativeNames
    validateattributes(options.(name), {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
end
validateattributes(options.FrameStride, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
handles    = createEmptyHandles(options);
obstacles  = obstacleAvoidance.obstacles.prepareObstacles(result.Inputs.obstacles);
gridRecord = struct();
routes     = struct([]);
if isfield(diagnosis, "Search"), gridRecord = diagnosis.Search; end
if isfield(diagnosis, "Routes"), routes = diagnosis.Routes; end

%% Section 2: Plot Workspace And Failure Diagnostics

if options.ShowWorkspace
    workspaceFigure = figure("Name", options.Title, "Visible", options.FigureVisible);
    workspaceAxes   = axes(workspaceFigure);
    configureSpatialAxes(workspaceAxes, result);
    drawObstacles(workspaceAxes, obstacles, result.Inputs.initialState.time_s);
    drawSearchDiagnostics(workspaceAxes, gridRecord, options.ShowSearchEdges);
    if options.ShowSeedPaths
        % Evaluate each seed before retaining the best admissible candidate.
        for seedIndex = 1:numel(routes)
            route_deg = displayPath(result, routes(seedIndex).position_deg);
            label     = "Route " + seedIndex + ": " + routes(seedIndex).Source;
            drawLine(workspaceAxes, route_deg, "-o", label, 1);
        end
    end
    drawPlannerRoute(workspaceAxes, result);
    drawTarget(workspaceAxes, result, result.Inputs.initialState.time_s);
    drawEndpoints(workspaceAxes, result);
    finishAxes(workspaceAxes, result, options.Title);
    handles.WorkspaceFigure = workspaceFigure;
    handles.WorkspaceAxes   = workspaceAxes;
end
doesCross = false;
if result.Success && result.Options.AllowAzimuthWrapping
    seamPath_deg = displayPath(result, result.position_deg);
    doesCross    = any(isnan(seamPath_deg(:, 1)));
end
if (options.ShowWorkspace || options.ShowAnimation || options.SaveAnimationGif) && doesCross
    [handles.ContinuousWorkspaceFigure, handles.ContinuousWorkspaceAxes] = createContinuousWorkspace(result, options);
end

%% Section 3: Plot Time-Expanded Diagnostics

if options.ShowVisibilityGraphs && ~isempty(fieldnames(gridRecord))
    if options.ShowWorkspace
        handles.VisibilityFigure = handles.WorkspaceFigure;
        handles.VisibilityAxes   = handles.WorkspaceAxes;
    else
        handles.VisibilityFigure = figure("Name", options.Title + " search", "Visible", options.FigureVisible);
        handles.VisibilityAxes   = axes(handles.VisibilityFigure);
        configureSpatialAxes(handles.VisibilityAxes, result);
        drawObstacles(handles.VisibilityAxes, obstacles, result.Inputs.initialState.time_s);
        drawSearchDiagnostics(handles.VisibilityAxes, gridRecord, options.ShowSearchEdges);
        drawPlannerRoute(handles.VisibilityAxes, result);
        drawTarget(handles.VisibilityAxes, result, result.Inputs.goalState.time_s);
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
    [complete_deg, sourceIndex] = displayPath(result, result.position_deg);
    gifFrameCount = 0;
    % Process each frame in temporal order and accumulate its result.
    for frameIndex = frameIndices
        cla(animationAxes);
        configureSpatialAxes(animationAxes, result);
        drawObstacles(animationAxes, obstacles, result.time_s(frameIndex));
        drawTarget(animationAxes, result, result.time_s(frameIndex));
        drawLine(animationAxes, complete_deg, "-", "Complete timed path", 1);
        elapsedEnd  = find(sourceIndex <= frameIndex, 1, "last");
        elapsed_deg = complete_deg(1:elapsedEnd, :);
        current_deg = displayPath(result, result.position_deg(frameIndex, :));
        drawLine(animationAxes, elapsed_deg, "c-", "Elapsed path", 3);
        scatter(animationAxes, current_deg(1), current_deg(2), 60, [0.95 0.25 0.15], "filled", "DisplayName", "Current state");
        xlabel(animationAxes, "Azimuth (deg)");
        ylabel(animationAxes, "Elevation (deg)");
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
end

%% Section 6: Local Functions

function options = normalizePlotAliases(options)
    % Normalize deprecated aliases and discard example-only reporting fields.
    if ~isstruct(options) || ~isscalar(options)
        error("plotTrajectory:InvalidOptions", "optionOverrides must be a scalar struct.");
    end
    aliases = ["AnimationFrameStride", "FrameStride"; ...
        "ShowKinematicPlot", "ShowKinematics"; "AnimationPause_s", "Pause_s"];
    % Process each alias needed to prepare plot aliases.
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
    % Apply periodic-axis display settings.
    hold(axesHandle, "on");
    grid(axesHandle, "on");
    box(axesHandle, "on");
    axis(axesHandle, "equal");
    if result.Options.AllowAzimuthWrapping
        xlim(axesHandle, result.Inputs.limits.azimuthInterval_deg);
    end
end

function [position_deg, sourceIndex] = displayPath(result, position_deg)
    % Apply the result's wrap settings to the displayed path.
    [position_deg, sourceIndex] = obstacleAvoidance.plotting.createWrappedSpatialPath(position_deg, result.Inputs.limits.azimuthInterval_deg, result.Options.AllowAzimuthWrapping);
end

function [figureHandle, axesHandle] = createContinuousWorkspace(result, options)
    % Show the unwrapped path and crossed wrap boundaries.
    figureHandle = figure("Name", options.Title + " continuous azimuth", "Visible", options.FigureVisible);
    axesHandle   = axes(figureHandle);
    hold(axesHandle, "on");
    grid(axesHandle, "on");
    box(axesHandle, "on");
    axis(axesHandle, "equal");
    drawLine(axesHandle, result.position_deg, "k-", "Timed motion", 2);
    interval_deg    = result.Inputs.limits.azimuthInterval_deg;
    period_deg      = diff(interval_deg);
    seamMultipliers = ceil((min(result.position_deg(:, 1)) - interval_deg(1)) / period_deg): floor((max(result.position_deg(:, 1)) - interval_deg(1)) / period_deg);
    % Process each seam deg needed to build continuous workspace.
    for seam_deg = interval_deg(1) + period_deg * seamMultipliers
        xline(axesHandle, seam_deg, ":", "HandleVisibility", "off");
    end
    xlabel(axesHandle, "Continuous azimuth (deg)");
    ylabel(axesHandle, "Elevation (deg)");
end

function drawPlannerRoute(axesHandle, result)
    % Draw the successful selected/timed path or the retained best partial route.
    if result.Success
        selected_deg = displayPath(result, result.Route_deg);
        motion_deg   = displayPath(result, result.position_deg);
        drawLine(axesHandle, selected_deg, "--", "Selected geometric route", 1);
        drawLine(axesHandle, motion_deg, "k-", "Timed motion", 2);
    elseif ~isempty(result.BestPartialRoute_deg)
        partial_deg = displayPath(result, result.BestPartialRoute_deg);
        drawLine(axesHandle, partial_deg, "--", "Best partial route", 1);
    end
end

function drawEndpoints(axesHandle, result)
    % Draw the requested start and terminal positions under the wrap policy.
    start_deg = displayPath(result, result.Inputs.initialState.position_deg);
    goal_deg  = obstacleAvoidance.input.goalPositionAtTime(result.Inputs.goalState, result.Inputs.goalState.time_s);
    goal_deg  = displayPath(result, goal_deg);
    plot(axesHandle, start_deg(1), start_deg(2), "go", "DisplayName", "Start");
    plot(axesHandle, goal_deg(1), goal_deg(2), "ro", "DisplayName", "Goal");
end

function lineHandle = drawLine(axesHandle, position_deg, style, name, width)
    % Draw one labelled two-dimensional path on explicit axes.
    lineHandle = plot(axesHandle, position_deg(:, 1), position_deg(:, 2), style, "LineWidth", width, "DisplayName", name);
end

function drawSearchDiagnostics(axesHandle, gridRecord, showEdges)
    % Draw explored nodes, accepted/rejected edges, and the search frontier.
    edgeNames  = ["AcceptedEdges_deg", "RejectedEdges_deg"];
    edgeStyles = ["-", ":"];
    edgeLabels = ["Accepted visibility edge", "Collision-rejected edge"];
    if showEdges
        % Process each category needed to complete s.
        for categoryIndex = 1:2
            if hasData(gridRecord, edgeNames(categoryIndex))
                edges_deg     = gridRecord.(edgeNames(categoryIndex));
                edgeCount     = size(edges_deg, 1);
                azimuth_deg   = reshape([edges_deg(:, [1 3]), nan(edgeCount, 1)].', [], 1);
                elevation_deg = reshape([edges_deg(:, [2 4]), nan(edgeCount, 1)].', [], 1);
                plot(axesHandle, azimuth_deg, elevation_deg, edgeStyles(categoryIndex), "DisplayName", edgeLabels(categoryIndex));
            end
        end
    end
    pointNames  = ["ExploredNodes_deg", "FrontierNodes_deg"];
    pointLabels = ["Expanded search node", "Final search frontier"];
    % Process each category needed to complete s.
    for categoryIndex = 1:2
        if hasData(gridRecord, pointNames(categoryIndex))
            points_deg = gridRecord.(pointNames(categoryIndex));
            scatter(axesHandle, points_deg(:, 1), points_deg(:, 2), 8 + 9 * categoryIndex, "filled", "DisplayName", pointLabels(categoryIndex));
        end
    end
end

function drawObstacles(axesHandle, obstacles, time_s)
    % Draw original and safety-adjusted geometry from retained obstacle histories.
    colors = lines(max(1, numel(obstacles)));
    % Evaluate each obstacle against the current geometry or motion.
    for obstacleIndex = 1:numel(obstacles)
        obstacle = obstacles(obstacleIndex);
        original = obstacle;
        original.az_deg = obstacle.originalAz_deg;
        original.el_deg = obstacle.originalEl_deg;
        if isfield(original, "InternalPreparation")
            original = rmfield(original, "InternalPreparation");
        end
        shape = obstacleAvoidance.obstacles.shapeAtTime(original, time_s);
        drawShape(axesHandle, shape, colors(obstacleIndex, :), "-", "Original obstacle");
        shape = obstacleAvoidance.obstacles.shapeAtTime(obstacle, time_s);
        drawShape(axesHandle, shape, "none", "--", "Protected obstacle");
    end
end

function drawShape(axesHandle, shape, faceColor, style, name)
    % Draw one nonempty polygon on explicit axes.
    if ~isempty(shape.Vertices)
        plot(axesHandle, shape, "FaceColor", faceColor, "FaceAlpha", 0.18, "LineStyle", style, "LineWidth", 1.2, "DisplayName", name);
    end
end

function drawTarget(axesHandle, result, displayTime_s)
    % Draw the moving target's track and current position.
    goalState = result.Inputs.goalState;
    if ~hasData(goalState, "targetPosition_deg")
        return;
    end
    track_deg = displayPath(result, goalState.targetPosition_deg);
    drawLine(axesHandle, track_deg, "-.", "Moving target track", 1);
    target_deg = obstacleAvoidance.input.goalPositionAtTime(goalState, displayTime_s);
    target_deg = displayPath(result, target_deg);
    plot(axesHandle, target_deg(1), target_deg(2), "md", "MarkerFaceColor", "m", "DisplayName", "Moving target");
end

function axesHandles = createKinematicPanels(layout, result, animated)
    % Plot position, velocity, acceleration, and jerk with their limits.
    quantityNames = ["position_deg", "velocity_deg_s", "acceleration_deg_s2", "jerk_deg_s3"];
    yLabels       = ["Position (deg)", "Velocity (deg/s)", "Acceleration (deg/s^2)", "Jerk (deg/s^3)"];
    limits        = [nan(1, 2); result.Inputs.limits.maxVelocity_deg_s; ...
        result.Inputs.limits.maxAcceleration_deg_s2; result.Inputs.limits.maxJerk_deg_s3];
    axesHandles = gobjects(4, 1);
    % Process each quantity needed to build kinematic panels.
    for quantityIndex = 1:4
        tileIndex = quantityIndex * (1 + animated);
        axesHandles(quantityIndex) = nexttile(layout, tileIndex);
        axesHandle = axesHandles(quantityIndex);
        hold(axesHandle, "on");
        grid(axesHandle, "on");
        box(axesHandle, "on");
        values      = result.(quantityNames(quantityIndex));
        lineHandles = plot(axesHandle, result.time_s, values);
        set(lineHandles, {'DisplayName'}, {'Azimuth'; 'Elevation'});
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
    % Label the plot with search counts.
    xlabel(axesHandle, "Azimuth (deg)");
    ylabel(axesHandle, "Elevation (deg)");
    title(axesHandle, sprintf("%s | %s", prefix, result.TerminationReason));
    legend(axesHandle, "Location", "best");
end

function value = hasData(record, fieldName)
    % Check whether an optional result field is present and nonempty.
    value = isfield(record, fieldName) && ~isempty(record.(fieldName));
end

function handles = createEmptyHandles(options)
    % Initialize graphics handles before choosing a display mode.
    none    = gobjects(0);
    handles = struct("WorkspaceFigure", none, "WorkspaceAxes", none, ...
        "ContinuousWorkspaceFigure", none, "ContinuousWorkspaceAxes", none, ...
        "VisibilityFigure", none, "VisibilityAxes", none, "VisibilityGraphs", struct(), ...
        "KinematicFigure", none, "KinematicAxes", none, "KinematicsFigure", none, ...
        "KinematicsAxes", none, "AnimationFigure", none, "AnimationAxes", none, ...
        "AnimationKinematicAxes", none, "AnimationLegend", none, "AnimationGifFile", "", ...
        "Animation", struct(), "Options", options);
end
