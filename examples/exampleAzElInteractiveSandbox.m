function result = exampleAzElInteractiveSandbox(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleAzElInteractiveSandbox()
%   result = exampleAzElInteractiveSandbox(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Interactively define a sandbox scene in azimuth/elevation coordinates.
%   - Draw a forbidden path and optional polygon obstacles, then plan around
%     them with the maintained planner.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Display controls, planner options, and sandbox controls.
%       Supported custom sandbox overrides are:
%       - MissionTime_s (positive scalar, default 180)
%       - MaxVelocity_deg_s (1x2, default [2 2])
%       - MaxAcceleration_deg_s2 (1x2, default [0.75 0.75])
%       - PathObstacleRadius_deg (positive scalar, default 0.5)
%       - PathSafetyMargin_deg (nonnegative scalar, default 0)
%       - ObstacleSafetyMargin_deg (nonnegative scalar, default 0.2)
%       - WorkspaceAzimuthInterval_deg (1x2, default [-180 180])
%       - WorkspaceElevationInterval_deg (1x2, default [-90 90])
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Planner result, input capture, sandbox metadata, and optional plot
%       handles from the maintained plotting stack.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
if ~isstruct(exampleOverrides) || ~isscalar(exampleOverrides)
    error("exampleAzElInteractiveSandbox:InvalidOverrides", "exampleOverrides must be a scalar struct.");
end

[plannerOverrides, sandboxOverrides] = separateSandboxOverrides( exampleOverrides);
[options, displayOptions] = resolveAzElExampleOptions( ...
    plannerOverrides, struct( ...
        "GoalTimeMode", "earliestArrival", ...
        "Verbose", true, ...
        "ShowSearchEdges", false, "ShowVisibilityGraphs", false, "Title", "Interactive sandbox"), [2.5 2.5]);

[limits, sandboxControls] = resolveSandboxControls(sandboxOverrides, displayOptions.ConfiguredFiniteMaxJerk_deg_s3);

%% Section 2: Open Sandbox Canvas and Collect Geometry

sandboxFigure = figure( "Name", "Az/El Interactive Sandbox", "Visible", "on");
azimuthLimits_deg = sandboxControls.WorkspaceAzimuthInterval_deg;
elevationLimits_deg = sandboxControls.WorkspaceElevationInterval_deg;
axesHandle = axes(sandboxFigure, "Units", "normalized", "Position", [0.09 0.38 0.62 0.55]);
controlHandles = createSandboxControlPanel( sandboxFigure, sandboxControls, options.Verbose);
hold(axesHandle, "on");
grid(axesHandle, "on");
box(axesHandle, "on");
axis(axesHandle, [azimuthLimits_deg, elevationLimits_deg]);
xlabel(axesHandle, "Azimuth (deg)");
ylabel(axesHandle, "Elevation (deg)");
title(axesHandle, "Sandbox: click to define start, goal, path, and obstacles");

disp("Interactive sandbox setup:");
disp("1) Click start point");
disp("2) Click goal point");
disp("3) Set motion limits and verbosity in the panel on the right.");
disp("4) Hold left mouse and draw an obstacle, then release to add it.");
disp("   A simplified two-point stroke becomes a line obstacle.");
disp("5) Draw another obstacle or right-click once when you are done.");

startPosition_deg = collectPoint(axesHandle, "Click the start point.");
if size(startPosition_deg, 1) ~= 1
    close(sandboxFigure);
    error("exampleAzElInteractiveSandbox:MissingStartPoint", "Start point is required.");
end
plot(axesHandle, startPosition_deg(:, 1), startPosition_deg(:, 2), ...
    "go", "MarkerFaceColor", "g", "DisplayName", "Start", "LineWidth", 1.4);

goalPosition_deg = collectPoint(axesHandle, "Click the goal point.");
if size(goalPosition_deg, 1) ~= 1
    close(sandboxFigure);
    error("exampleAzElInteractiveSandbox:MissingGoalPoint", "Goal point is required.");
end
plot(axesHandle, goalPosition_deg(:, 1), goalPosition_deg(:, 2), ...
    "ro", "MarkerFaceColor", "r", "DisplayName", "Goal", "LineWidth", 1.4);

[drawnLineCollection_deg, polygonObstaclePositions_deg] = collectFreehandObstacles(axesHandle, ...
    "Draw obstacles: hold left to trace; right-click when finished.");
[limits, sandboxControls, options] = readSandboxControlPanel( limits, sandboxControls, options, controlHandles);

%% Section 3: Build Obstacles and Run Planner

time_s = [0; limits.MissionTime_s];
pathObstacleData = cell(numel(drawnLineCollection_deg), 1);

for lineIndex = 1:numel(drawnLineCollection_deg)
    pathObstacleData{lineIndex} = pathToObstacleData( ...
        drawnLineCollection_deg{lineIndex}, time_s, ...
        sandboxControls.PathObstacleRadius_deg, sandboxControls.PathSafetyMargin_deg);
end
userObstacleData = obstaclePolygonsToData( ...
    polygonObstaclePositions_deg, time_s, sandboxControls.ObstacleSafetyMargin_deg);
obstacles = combineAzElObstacles(pathObstacleData, userObstacleData);

initialState = struct( ...
    "time_s", 0, "position_deg", startPosition_deg, "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", limits.MissionTime_s, ...
    "position_deg", goalPosition_deg, "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);

result = runSandboxPlanner( obstacles, initialState, goalState, limits, options, controlHandles);
if ~result.Success
    fprintf("[Sandbox] %s (%s)\n", result.Message, result.TerminationReason);
end
result.PlotHandles = struct();
if displayOptions.PlotOutputs
    result.PlotHandles = plotAzElMotion( result, displayOptions.PlotOptions);
end

%% Section 4: Return Sandbox Metadata

result.ExampleValidation = validateAzElExampleResult( ...
    result, "interactive sandbox", struct("RequireDirectBlocked", true));
result.ExampleName = "exampleAzElInteractiveSandbox";
result.ExampleMetrics = computeAzElExampleMetrics(result);
result.ExampleInputs = struct( ...
    "initialState", initialState, ...
    "goalState", goalState, ...
    "limits", limits, ...
    "plannerOptions", options, ...
    "obstacles", obstacles, ...
    "drawnLineCollection_deg", {drawnLineCollection_deg}, ...
    "polygonObstaclePositions_deg", {polygonObstaclePositions_deg}, ...
    "sandboxControls", sandboxControls, "sandboxFigure", sandboxFigure);
result.SandboxFigure = sandboxFigure;
result.SandboxAxes = axesHandle;

%% Section 5: Local Functions

function [plannerOverrides, sandboxOverrides] = separateSandboxOverrides( overrides)
% Split custom sandbox controls from shared planner and plotting fields.
sandboxControlNames = [ ...
    "MissionTime_s", ...
    "PathObstacleRadius_deg", ...
    "PathSafetyMargin_deg", ...
    "ObstacleSafetyMargin_deg", ...
    "WorkspaceAzimuthInterval_deg", "WorkspaceElevationInterval_deg", "MaxVelocity_deg_s", "MaxAcceleration_deg_s2"];
plannerOverrides = overrides;
sandboxOverrides = struct();

for sandboxName = sandboxControlNames
    if isfield(plannerOverrides, sandboxName)
        sandboxOverrides.(sandboxName) = plannerOverrides.(sandboxName);
        plannerOverrides = rmfield(plannerOverrides, sandboxName);
    end
end
end

function [limits, controls] = resolveSandboxControls( sandboxOverrides, configuredMaxJerk_deg_s3)
% Apply defaults and validate sandbox-specific parameters.
if isempty(sandboxOverrides)
    sandboxOverrides = struct();
end
if isfield(sandboxOverrides, "MissionTime_s") && ~isempty(sandboxOverrides.MissionTime_s)
    missionTime_s = sandboxOverrides.MissionTime_s;
else
    missionTime_s = 180;
end
if isfield(sandboxOverrides, "MaxVelocity_deg_s") && ~isempty(sandboxOverrides.MaxVelocity_deg_s)
    maxVelocity_deg_s = sandboxOverrides.MaxVelocity_deg_s;
else
    maxVelocity_deg_s = [2 2];
end
if isfield(sandboxOverrides, "MaxAcceleration_deg_s2") && ~isempty(sandboxOverrides.MaxAcceleration_deg_s2)
    maxAcceleration_deg_s2 = sandboxOverrides.MaxAcceleration_deg_s2;
else
    maxAcceleration_deg_s2 = [0.75 0.75];
end
if isfield(sandboxOverrides, "PathObstacleRadius_deg") && ~isempty(sandboxOverrides.PathObstacleRadius_deg)
    pathObstacleRadius_deg = sandboxOverrides.PathObstacleRadius_deg;
else
    pathObstacleRadius_deg = 0.5;
end
if isfield(sandboxOverrides, "PathSafetyMargin_deg") && ~isempty(sandboxOverrides.PathSafetyMargin_deg)
    pathSafetyMargin_deg = sandboxOverrides.PathSafetyMargin_deg;
else
    pathSafetyMargin_deg = 0;
end
if isfield(sandboxOverrides, "ObstacleSafetyMargin_deg") && ~isempty(sandboxOverrides.ObstacleSafetyMargin_deg)
    obstacleSafetyMargin_deg = sandboxOverrides.ObstacleSafetyMargin_deg;
else
    obstacleSafetyMargin_deg = 0.2;
end
if isfield(sandboxOverrides, "WorkspaceAzimuthInterval_deg") && ~isempty(sandboxOverrides.WorkspaceAzimuthInterval_deg)
    workspaceAzimuthInterval_deg = sandboxOverrides.WorkspaceAzimuthInterval_deg;
else
    workspaceAzimuthInterval_deg = [-180 180];
end
if isfield(sandboxOverrides, "WorkspaceElevationInterval_deg") && ...
        ~isempty(sandboxOverrides.WorkspaceElevationInterval_deg)
    workspaceElevationInterval_deg = sandboxOverrides.WorkspaceElevationInterval_deg;
else
    workspaceElevationInterval_deg = [-90 90];
end
validateattributes(missionTime_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(maxVelocity_deg_s, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2, 'positive'});
validateattributes(maxAcceleration_deg_s2, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2, 'positive'});
validateattributes(pathObstacleRadius_deg, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(pathSafetyMargin_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(obstacleSafetyMargin_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(workspaceAzimuthInterval_deg, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2, 'increasing'});
validateattributes(workspaceElevationInterval_deg, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2, 'increasing'});

limits = struct( ...
    "MissionTime_s", missionTime_s, ...
    "maxVelocity_deg_s", reshape(double(maxVelocity_deg_s), 1, 2), ...
    "maxAcceleration_deg_s2", reshape(double(maxAcceleration_deg_s2), 1, 2), ...
    "maxJerk_deg_s3", reshape(double(configuredMaxJerk_deg_s3), 1, 2));
limits.azimuthInterval_deg = reshape(double(workspaceAzimuthInterval_deg), 1, 2);
limits.elevationInterval_deg = reshape(double(workspaceElevationInterval_deg), 1, 2);
controls = struct( ...
    "MissionTime_s", missionTime_s, ...
    "MaxVelocity_deg_s", maxVelocity_deg_s, ...
    "MaxAcceleration_deg_s2", maxAcceleration_deg_s2, ...
    "MaxJerk_deg_s3", configuredMaxJerk_deg_s3, ...
    "PathObstacleRadius_deg", pathObstacleRadius_deg, ...
    "PathSafetyMargin_deg", pathSafetyMargin_deg, ...
    "ObstacleSafetyMargin_deg", obstacleSafetyMargin_deg, ...
    "WorkspaceAzimuthInterval_deg", workspaceAzimuthInterval_deg, ...
    "WorkspaceElevationInterval_deg", workspaceElevationInterval_deg);
end

function controlHandles = createSandboxControlPanel( figureHandle, controls, isVerbose)
% Show editable motion limits and planner verbosity beside the canvas.
panelHandle = uipanel(figureHandle, ...
    "Title", "Planning controls", "Units", "normalized", "Position", [0.74 0.38 0.23 0.55]);
controlHandles = struct( ...
    "MissionTimeHandle", addSandboxScalarControl(panelHandle, ...
        "Mission horizon (s)", 0.94, ...
        sprintf("%.6g", controls.MissionTime_s)), ...
    "VelocityHandles", addSandboxAxisPairControl(panelHandle, ...
        "Maximum velocity (deg/s)", 0.82, controls.MaxVelocity_deg_s), ...
    "AccelerationHandles", addSandboxAxisPairControl(panelHandle, ...
        "Maximum acceleration (deg/s^2)", 0.66, ...
        controls.MaxAcceleration_deg_s2), ...
    "JerkHandles", addSandboxAxisPairControl(panelHandle, "Maximum jerk (deg/s^3)", 0.50, controls.MaxJerk_deg_s3));
verboseHandle = uicontrol(panelHandle, ...
    "Style", "checkbox", ...
    "String", "Verbose planner output", ...
    "Units", "normalized", ...
    "Position", [0.08 0.35 0.84 0.05], "Value", logical(isVerbose), "HorizontalAlignment", "left");
controlHandles.VerboseHandle = verboseHandle;
logPanelHandle = uipanel(figureHandle, ...
    "Title", "Planner log", "Units", "normalized", "Position", [0.09 0.05 0.88 0.26]);
controlHandles.VerboseLogHandle = uicontrol(logPanelHandle, ...
    "Style", "listbox", ...
    "String", {"Verbose output will appear here."}, ...
    "Units", "normalized", "Position", [0.02 0.06 0.96 0.90], "HorizontalAlignment", "left", "Max", 2, "Min", 0);
end

function editHandle = addSandboxScalarControl( panelHandle, labelText, topPosition, defaultText)
% Place one labeled editable control without duplicating panel geometry.
uicontrol(panelHandle, ...
    "Style", "text", ...
    "String", labelText, ...
    "Units", "normalized", "Position", [0.08 topPosition 0.84 0.04], "HorizontalAlignment", "left");
editHandle = uicontrol(panelHandle, ...
    "Style", "edit", ...
    "String", defaultText, ...
    "Units", "normalized", "Position", [0.08 topPosition - 0.055 0.84 0.05], "HorizontalAlignment", "left");
end

function controlHandles = addSandboxAxisPairControl( panelHandle, labelText, topPosition, defaultValues)
% Place individual azimuth and elevation fields for one vector limit.
uicontrol(panelHandle, ...
    "Style", "text", ...
    "String", labelText, ...
    "Units", "normalized", "Position", [0.08 topPosition 0.84 0.035], "HorizontalAlignment", "left");
uicontrol(panelHandle, ...
    "Style", "text", ...
    "String", "Azimuth", ...
    "Units", "normalized", "Position", [0.08 topPosition - 0.04 0.39 0.025], "HorizontalAlignment", "left");
uicontrol(panelHandle, ...
    "Style", "text", ...
    "String", "Elevation", ...
    "Units", "normalized", "Position", [0.53 topPosition - 0.04 0.39 0.025], "HorizontalAlignment", "left");
controlHandles = struct( ...
    "AzimuthHandle", uicontrol(panelHandle, ...
        "Style", "edit", ...
        "String", sprintf("%.6g", defaultValues(1)), ...
        "Units", "normalized", ...
        "Position", [0.08 topPosition - 0.09 0.39 0.045]), ...
    "ElevationHandle", uicontrol(panelHandle, ...
        "Style", "edit", ...
        "String", sprintf("%.6g", defaultValues(2)), ...
        "Units", "normalized", "Position", [0.53 topPosition - 0.09 0.39 0.045]));
end

function [limits, controls, options] = readSandboxControlPanel( limits, controls, options, controlHandles)
% Read and validate the limits selected in the persistent sandbox panel.
missionTime_s = str2double(get(controlHandles.MissionTimeHandle, "String"));
validateattributes(missionTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'}, "exampleAzElInteractiveSandbox", "Mission horizon (s)");
maxVelocity_deg_s = readAxisPairControl( controlHandles.VelocityHandles, "Maximum velocity (deg/s)");
maxAcceleration_deg_s2 = readAxisPairControl( controlHandles.AccelerationHandles, "Maximum acceleration (deg/s^2)");
maxJerk_deg_s3 = readAxisPairControl( controlHandles.JerkHandles, "Maximum jerk (deg/s^3)");
limits.MissionTime_s = missionTime_s;
limits.maxVelocity_deg_s = maxVelocity_deg_s;
limits.maxAcceleration_deg_s2 = maxAcceleration_deg_s2;
limits.maxJerk_deg_s3 = maxJerk_deg_s3;
controls.MissionTime_s = missionTime_s;
controls.MaxVelocity_deg_s = maxVelocity_deg_s;
controls.MaxAcceleration_deg_s2 = maxAcceleration_deg_s2;
controls.MaxJerk_deg_s3 = maxJerk_deg_s3;
options.Verbose = logical(get(controlHandles.VerboseHandle, "Value"));
end

function values = readAxisPairControl(controlHandles, fieldName)
% Read one positive azimuth/elevation limit pair from separate fields.
values = [ ...
    str2double(get(controlHandles.AzimuthHandle, "String")), str2double(get(controlHandles.ElevationHandle, "String"))];
validateattributes(values, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2, 'positive'}, "exampleAzElInteractiveSandbox", fieldName);
values = reshape(values, 1, 2);
end

function result = runSandboxPlanner( obstacles, initialState, goalState, limits, options, controlHandles)
% Run the planner and present requested verbose output inside the figure.
if options.Verbose
    set(controlHandles.VerboseLogHandle, "String", {"Planning..."});
    drawnow;
    result = struct();
    plannerLog = evalc( 'result = planAzElMotion(obstacles, initialState, goalState, limits, options);');
    updateSandboxVerboseLog(controlHandles.VerboseLogHandle, plannerLog, result);
    result.SandboxVerboseLog = string(plannerLog);
else
    result = planAzElMotion( obstacles, initialState, goalState, limits, options);
    set(controlHandles.VerboseLogHandle, "String", {"Verbose output is disabled."});
    result.SandboxVerboseLog = "";
end
end

function updateSandboxVerboseLog(logHandle, plannerLog, result)
% Show the complete captured planner transcript in the scrollable log.
logLines = splitlines(string(plannerLog));
logLines = strtrim(logLines);
logLines = logLines(strlength(logLines) > 0);
if isempty(logLines)
    logLines = result.Message + " (" + result.TerminationReason + ")";
end
set(logHandle, "String", cellstr(logLines), "Value", 1);
end

function point_deg = collectPoint(axesHandle, prompt)
% Collect exactly one point via mouse click.
title(axesHandle, prompt);
drawnow("nocallbacks");
set(axesHandle, "ButtonDownFcn", " ");
point_deg = [];

while true
    [azimuth_deg, elevation_deg, button] = ginput(1);
    if isempty(azimuth_deg) || isempty(elevation_deg) || button ~= 1
        break;
    end
    point_deg = [azimuth_deg, elevation_deg];
    break;
end
if isempty(point_deg)
    return;
end
end

function [lineCollection_deg, polygonCollection_deg] = collectFreehandObstacles(axesHandle, prompt)
% Add simplified line and polygon obstacles until a right-click ends input.
title(axesHandle, prompt);
drawnow("nocallbacks");
figureHandle = ancestor(axesHandle, "figure");
lineCollection_deg = cell(1, 0);
polygonCollection_deg = cell(1, 0);
stroke_deg = zeros(0, 2);
isDrawing = false;
minimumTraceSpacing_deg = 0.25;
freehandSimplificationTolerance_deg = 1;
traceHandle = plot(axesHandle, NaN, NaN, "b-", "LineWidth", 1.2, "HandleVisibility", "off");
previousCallbacks = struct( ...
    "Down", get(figureHandle, "WindowButtonDownFcn"), ...
    "Motion", get(figureHandle, "WindowButtonMotionFcn"), "Up", get(figureHandle, "WindowButtonUpFcn"));
set(figureHandle, "WindowButtonDownFcn", @startTrace);
set(figureHandle, "WindowButtonMotionFcn", @extendTrace);
set(figureHandle, "WindowButtonUpFcn", @finishTrace);
uiwait(figureHandle);
restoreFigureCallbacks();

    function startTrace(~, ~)
        % Begin a freehand stroke only when the user clicks the drawing canvas with the primary mouse button.
        clickedHandle = hittest(figureHandle);
        clickedAxesHandle = ancestor(clickedHandle, "axes");
        isCanvasClick = isequal(clickedHandle, axesHandle) || isequal(clickedAxesHandle, axesHandle);
        if ~isCanvasClick
            return;
        end
        if string(get(figureHandle, "SelectionType")) ~= "normal"
            uiresume(figureHandle);
            return;
        end
        isDrawing = true;
        appendCursorPoint();
    end

    function extendTrace(~, ~)
        % Sample cursor motion only while a stroke is active so ordinary pointer movement does not create geometry.
        if isDrawing
            appendCursorPoint();
        end
    end

    function finishTrace(~, ~)
        % Finalize the active stroke, simplify its points, and store the resulting line or polygon.
        if ~isDrawing
            return;
        end
        appendCursorPoint();
        isDrawing = false;
        simplifiedStroke_deg = simplifyFreehandBoundary( stroke_deg, freehandSimplificationTolerance_deg);
        if size(simplifiedStroke_deg, 1) == 2
            lineCollection_deg{end + 1} = simplifiedStroke_deg;
            plot(axesHandle, simplifiedStroke_deg(:, 1), ...
                simplifiedStroke_deg(:, 2), "--", "Color", [0.85 0.1 0.1], "LineWidth", 1.2, "HandleVisibility", "off");
        elseif size(simplifiedStroke_deg, 1) >= 3
            polygonCollection_deg{end + 1} = simplifiedStroke_deg;
            fill(axesHandle, simplifiedStroke_deg(:, 1), ...
                simplifiedStroke_deg(:, 2), [0.2 0.3 0.9], ...
                "FaceAlpha", 0.18, "EdgeColor", [0.2 0.3 0.9], "HandleVisibility", "off");
        end
        stroke_deg = zeros(0, 2);
        set(traceHandle, "XData", NaN, "YData", NaN);
        title(axesHandle, prompt);
        drawnow("limitrate");
    end

    function appendCursorPoint()
        % Append a cursor sample only after it has moved far enough to improve the freehand trace.
        cursorPosition_deg = get(axesHandle, "CurrentPoint");
        cursorPosition_deg = cursorPosition_deg(1, 1:2);
        if ~isempty(stroke_deg) && norm(cursorPosition_deg - stroke_deg(end, :)) < minimumTraceSpacing_deg
            return;
        end
        stroke_deg(end + 1, :) = cursorPosition_deg;
        set(traceHandle, "XData", stroke_deg(:, 1), "YData", stroke_deg(:, 2));
        drawnow("limitrate");
    end

    function restoreFigureCallbacks()
        % Restore callbacks owned by the caller and remove the temporary trace graphic.
        if isgraphics(figureHandle)
            set(figureHandle, "WindowButtonDownFcn", previousCallbacks.Down);
            set(figureHandle, "WindowButtonMotionFcn", previousCallbacks.Motion);
            set(figureHandle, "WindowButtonUpFcn", previousCallbacks.Up);
        end
        if isgraphics(traceHandle)
            delete(traceHandle);
        end
    end
end

function simplified_deg = simplifyFreehandBoundary( points_deg, tolerance_deg)
% Keep shape-defining turns while removing mouse-event oversampling.
if size(points_deg, 1) <= 2
    simplified_deg = points_deg;
    return;
end
lineVector_deg = points_deg(end, :) - points_deg(1, :);
lineLength_deg = norm(lineVector_deg);
interiorPoints_deg = points_deg(2:end - 1, :);
if lineLength_deg <= eps
    distance_deg = vecnorm( interiorPoints_deg - points_deg(1, :), 2, 2);
else
    relativePoints_deg = interiorPoints_deg - points_deg(1, :);
    distance_deg = abs( ...
        relativePoints_deg(:, 1) * lineVector_deg(2) - relativePoints_deg(:, 2) * lineVector_deg(1)) / lineLength_deg;
end
[maximumDistance_deg, maximumIndex] = max(distance_deg);
if maximumDistance_deg <= tolerance_deg
    simplified_deg = points_deg([1 end], :);
    return;
end
splitIndex = maximumIndex + 1;
firstHalf_deg = simplifyFreehandBoundary( points_deg(1:splitIndex, :), tolerance_deg);
secondHalf_deg = simplifyFreehandBoundary( points_deg(splitIndex:end, :), tolerance_deg);
simplified_deg = [firstHalf_deg(1:end - 1, :); secondHalf_deg];
end

function pathObstacleData = pathToObstacleData( path_deg, time_s, radius_deg, safetyMargin_deg)
% Convert a drawn path into one continuous buffered no-fly obstacle.
path_deg = double(path_deg);
path_deg = path_deg(all(isfinite(path_deg), 2), :);
if size(path_deg, 1) > 1
    isDistinctPoint = [true; vecnorm(diff(path_deg, 1, 1), 2, 2) > eps];
    path_deg = path_deg(isDistinctPoint, :);
end
if size(path_deg, 1) < 2
    pathObstacleData = cell(0, 1);
    return;
end
% A dense round buffer turns one short drawn line into hundreds of boundary
% vertices. The fallback profile stops at every selected vertex, which
% creates visible acceleration and jerk oscillations. Joined low-vertex
% capsules retain the requested radius while exposing only a few detour
% vertices to the planner.
pathBuffer = buildPathCapsuleShape(path_deg, radius_deg);
bufferVertices_deg = pathBuffer.Vertices;
if size(bufferVertices_deg, 1) < 3
    pathObstacleData = cell(0, 1);
    return;
end
pathObstacleData = {makeAzElObstacleData( ...
    "drawn path obstacle", time_s, bufferVertices_deg(:, 1), bufferVertices_deg(:, 2), safetyMargin_deg)};
end

function pathShape = buildPathCapsuleShape(path_deg, radius_deg)
% Union low-vertex capsules so a drawn path stays continuously blocked.
endCapSegmentCount = 3;
arcIncrement_rad = pi / endCapSegmentCount;
constructionRadius_deg = radius_deg / cos(arcIncrement_rad / 2);
pathShape = polyshape();
hasSegment = false;

for segmentIndex = 1:size(path_deg, 1) - 1
    startPosition_deg = path_deg(segmentIndex, :);
    endPosition_deg = path_deg(segmentIndex + 1, :);
    direction_deg = endPosition_deg - startPosition_deg;
    segmentLength_deg = norm(direction_deg);
    if segmentLength_deg <= eps
        continue;
    end
    direction_1 = direction_deg / segmentLength_deg;
    directionAngle_rad = atan2(direction_1(2), direction_1(1));
    startAngles_rad = linspace( directionAngle_rad + pi / 2, directionAngle_rad + 3 * pi / 2, endCapSegmentCount + 1).';
    endAngles_rad = linspace( directionAngle_rad - pi / 2, directionAngle_rad + pi / 2, endCapSegmentCount + 1).';
    startCap_deg = startPosition_deg + constructionRadius_deg * [cos(startAngles_rad), sin(startAngles_rad)];
    endCap_deg = endPosition_deg + constructionRadius_deg * [cos(endAngles_rad), sin(endAngles_rad)];
    segmentShape = polyshape([startCap_deg; endCap_deg]);
    if hasSegment
        pathShape = union(pathShape, segmentShape);
    else
        pathShape = segmentShape;
        hasSegment = true;
    end
end
end

function obstacleData = obstaclePolygonsToData( polygonCollection_deg, time_s, safetyMargin_deg)
% Convert user polygons to canonical obstacle structs.
obstacleData = cell(numel(polygonCollection_deg), 1);

for polygonIndex = 1:numel(polygonCollection_deg)
    polygon_deg = polygonCollection_deg{polygonIndex};
    if ~isequal(polygon_deg(1, :), polygon_deg(end, :))
        % Closing one user polygon adds exactly one bounded row.
        polygon_deg = vertcat( polygon_deg, polygon_deg(1, :)); %#ok<AGROW>
    end
    obstacleData{polygonIndex} = makeAzElObstacleData( ...
        "drawn polygon obstacle " + polygonIndex, time_s, polygon_deg(:, 1), polygon_deg(:, 2), safetyMargin_deg);
end
end

end
