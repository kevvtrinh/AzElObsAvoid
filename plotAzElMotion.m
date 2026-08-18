function handles = plotAzElMotion(result, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = plotAzElMotion()
%   handles = plotAzElMotion(result)
%   handles = plotAzElMotion(result, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot returned HS3 seeds, motion, limits, and failure search diagnostics.
%   - Animate returned motion without rerunning planning or collision logic.
%**************************************************************************
% INPUTS
%   - result (scalar planAzElMotion result)
%   - optionOverrides (scalar struct, optional; default struct())
%       FigureVisible is "on" or "off". ShowWorkspace, ShowKinematics,
%       and ShowAnimation are logical. AnimationFrameStride is positive.
%       Pause_s is nonnegative and is ignored for hidden figures.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Workspace, kinematic, and animation figure and axes handles.
%**************************************************************************
% UNITS
%   - Axes show degrees, seconds, deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Display Controls

defaults = struct( ...
    "FigureVisible", "on", ...
    "ShowWorkspace", true, ...
    "ShowKinematics", true, ...
    "ShowAnimation", true, ...
    "AnimationFrameStride", 4, ...
    "Pause_s", 0.01);
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
[options, unknownNames] = azElInternal.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("plotAzElMotion:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
options.FigureVisible = lower(string(options.FigureVisible));
if ~isscalar(options.FigureVisible) || ...
        ~any(options.FigureVisible == ["on", "off"])
    error("plotAzElMotion:InvalidFigureVisible", ...
        "FigureVisible must be 'on' or 'off'.");
end
logicalNames = ["ShowWorkspace", "ShowKinematics", "ShowAnimation"];
for name = logicalNames
    options.(name) = azElInternal.normalizeLogicalScalar( ...
        options.(name), name, "plotAzElMotion:InvalidLogicalOption");
end
validateattributes(options.AnimationFrameStride, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(options.Pause_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
handles = emptyHandles(options);

%% Section 2: Plot Workspace And Search Diagnostics

if options.ShowWorkspace
    workspaceFigure = figure( ...
        "Name", "Azimuth/elevation HS3 planning", ...
        "Visible", options.FigureVisible);
    workspaceAxes = axes(workspaceFigure);
    hold(workspaceAxes, "on");
    grid(workspaceAxes, "on");
    box(workspaceAxes, "on");
    axis(workspaceAxes, "equal");
    obstacles = result.Inputs.obstacles;
    displayTime_s = result.Inputs.initialState.time_s;
    if result.Success
        displayTime_s = result.time_s(1);
    end
    drawObstacles(workspaceAxes, obstacles, displayTime_s, true);
    if isfield(result.SearchDiagnostics, "Grid") && ...
            isfield(result.SearchDiagnostics.Grid, "ExploredNodes_deg")
        explored_deg = result.SearchDiagnostics.Grid.ExploredNodes_deg;
        if ~isempty(explored_deg)
            scatter(workspaceAxes, explored_deg(:, 1), explored_deg(:, 2), ...
                8, [0.75 0.75 0.75], "filled", ...
                "DisplayName", "Explored visibility nodes");
        end
    end
    for seedIndex = 1:numel(result.Seeds)
        route_deg = result.Seeds(seedIndex).position_deg;
        plot(workspaceAxes, route_deg(:, 1), route_deg(:, 2), ...
            ":", "Color", [0.55 0.55 0.55], ...
            "DisplayName", "Topology seed");
    end
    if result.Success
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
    start_deg = result.Inputs.initialState.position_deg;
    goal_deg = goalPositionAtTime( ...
        result.Inputs.goalState, result.Inputs.goalState.time_s);
    scatter(workspaceAxes, start_deg(1), start_deg(2), 50, "g", ...
        "filled", "DisplayName", "Start");
    scatter(workspaceAxes, goal_deg(1), goal_deg(2), 50, "r", ...
        "filled", "DisplayName", "Goal");
    xlabel(workspaceAxes, "Azimuth (deg)");
    ylabel(workspaceAxes, "Elevation (deg)");
    title(workspaceAxes, sprintf( ...
        "%s | seeds %d | expanded %d", ...
        result.TerminationReason, numel(result.Seeds), ...
        expandedCount(result.SearchDiagnostics)));
    legend(workspaceAxes, "Location", "best");
    handles.WorkspaceFigure = workspaceFigure;
    handles.WorkspaceAxes = workspaceAxes;
end

%% Section 3: Plot Returned Kinematics

if options.ShowKinematics && result.Success
    kinematicFigure = figure( ...
        "Name", "HS3 kinematics", "Visible", options.FigureVisible);
    tiledLayout = tiledlayout(kinematicFigure, 4, 1);
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
    handles.KinematicFigure = kinematicFigure;
    handles.KinematicAxes = axesHandles;
end

%% Section 4: Animate Returned Motion

if options.ShowAnimation && result.Success
    animationFigure = figure( ...
        "Name", "HS3 motion animation", "Visible", options.FigureVisible);
    animationAxes = axes(animationFigure);
    frameIndices = unique([ ...
        1:options.AnimationFrameStride:numel(result.time_s), ...
        numel(result.time_s)]);
    for frameIndex = frameIndices
        cla(animationAxes);
        hold(animationAxes, "on");
        grid(animationAxes, "on");
        box(animationAxes, "on");
        axis(animationAxes, "equal");
        drawObstacles(animationAxes, result.Inputs.obstacles, ...
            result.time_s(frameIndex), false);
        plot(animationAxes, result.position_deg(1:frameIndex, 1), ...
            result.position_deg(1:frameIndex, 2), "b-", ...
            "LineWidth", 1.5);
        scatter(animationAxes, result.position_deg(frameIndex, 1), ...
            result.position_deg(frameIndex, 2), 45, "b", "filled");
        xlabel(animationAxes, "Azimuth (deg)");
        ylabel(animationAxes, "Elevation (deg)");
        title(animationAxes, sprintf("t = %.3f s", ...
            result.time_s(frameIndex)));
        drawnow;
        if options.FigureVisible == "on" && options.Pause_s > 0
            pause(options.Pause_s);
        end
    end
    handles.AnimationFigure = animationFigure;
    handles.AnimationAxes = animationAxes;
end
end

%% Section 5: Local Functions

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

function count = expandedCount(searchDiagnostics)
% PURPOSE
%   - Read bounded grid-search count without plot-time reconstruction.
count = 0;
if isfield(searchDiagnostics, "Grid") && ...
        isfield(searchDiagnostics.Grid, "ExpandedCount")
    count = searchDiagnostics.Grid.ExpandedCount;
end
end

function handles = emptyHandles(options)
% PURPOSE
%   - Define stable empty graphics output for every display combination.
handles = struct( ...
    "WorkspaceFigure", gobjects(0), ...
    "WorkspaceAxes", gobjects(0), ...
    "KinematicFigure", gobjects(0), ...
    "KinematicAxes", gobjects(0), ...
    "AnimationFigure", gobjects(0), ...
    "AnimationAxes", gobjects(0), ...
    "Options", options);
end
