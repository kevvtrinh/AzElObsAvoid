function handles = plotAzElMotion(result, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = plotAzElMotion()
%   handles = plotAzElMotion(result)
%   handles = plotAzElMotion(result, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot obstacle geometry, candidate routes, the selected motion, and
%     its position, velocity, acceleration, and jerk histories.
%**************************************************************************
% INPUTS
%   - result (scalar planner-result struct)
%       Output from planAzElMotion.
%   - optionOverrides (scalar struct, optional; default struct())
%       FigureVisible is "on" or "off" and Title is scalar text.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       WorkspaceFigure, WorkspaceAxes, KinematicsFigure, and
%       KinematicsAxes. A zero-input call returns plot defaults.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************
defaults = struct("FigureVisible", "on", "Title", "Az/El motion plan");
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
title(workspaceAxes, options.Title + " — " + result.TerminationReason);
axis(workspaceAxes, "equal");
grid(workspaceAxes, "on");
box(workspaceAxes, "on");
legend(workspaceAxes, "Location", "best");

%% Section 2: Plot Kinematics
kinematicsFigure = figure("Visible", options.FigureVisible, ...
    "Name", options.Title + " kinematics");
layout = tiledlayout(kinematicsFigure, 4, 1, ...
    "TileSpacing", "compact", "Padding", "compact");
kinematicAxes = gobjects(4, 1);
quantities = {"position_deg", "velocity_deg_s", ...
    "acceleration_deg_s2", "jerk_deg_s3"};
labels = ["Position (deg)" "Velocity (deg/s)" ...
    "Acceleration (deg/s^2)" "Jerk (deg/s^3)"];
for quantityIndex = 1:4
    kinematicAxes(quantityIndex) = nexttile(layout);
    values = timedPath.(quantities{quantityIndex});
    if ~isempty(timedPath.time_s) && ~isempty(values)
        plot(kinematicAxes(quantityIndex), timedPath.time_s, values, ...
            "LineWidth", 1.2);
    end
    ylabel(kinematicAxes(quantityIndex), labels(quantityIndex));
    grid(kinematicAxes(quantityIndex), "on");
    box(kinematicAxes(quantityIndex), "on");
end
xlabel(kinematicAxes(end), "Time (s)");
legend(kinematicAxes(1), ["Azimuth" "Elevation"], ...
    "Location", "best");
title(layout, options.Title);
handles = struct("WorkspaceFigure", workspaceFigure, ...
    "WorkspaceAxes", workspaceAxes, ...
    "KinematicsFigure", kinematicsFigure, ...
    "KinematicsAxes", kinematicAxes);
end

%% Section 3: Local Functions
function options = resolvePlotOptions(defaults, overrides)
%% Section 0: Header & Readme
% Merge and validate the two presentation-only controls.
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
end

function plotObstacleHistory( ...
        axesHandle, obstacles, color, lineStyle, displayName)
%% Section 0: Header & Readme
% Plot at most first, middle, and last full-resolution obstacle slices.
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
