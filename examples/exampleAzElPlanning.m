function result = exampleAzElPlanning(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleAzElPlanning()
%   result = exampleAzElPlanning(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate the complete maintained workflow: create dense obstacle
%     data, plan one collision-free motion, validate it, and plot outputs.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       FigureVisible is "on" or "off"; PlotOutputs controls plotting;
%       ShowAnimation and ShowKinematicPlot select returned figures;
%       EnableJerkConstraint selects finite or unlimited jerk.
%**************************************************************************
% OUTPUTS
%   - result (scalar planner-result struct)
%       Includes independent planner validation, example inputs, and plot
%       handles when plotting is enabled.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

controls = struct("FigureVisible", "on", "PlotOutputs", true, ...
    "ShowAnimation", true, "ShowKinematicPlot", true, ...
    "EnableJerkConstraint", true);
if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
if ~isstruct(exampleOverrides) || ~isscalar(exampleOverrides)
    error("exampleAzElPlanning:InvalidOverrides", ...
        "exampleOverrides must be a scalar struct.");
end
names = fieldnames(exampleOverrides);
for nameIndex = 1:numel(names)
    name = names{nameIndex};
    if ~isfield(controls, name)
        warning("exampleAzElPlanning:UnknownOverride", ...
            "Ignored unknown override %s.", name);
    elseif ~isempty(exampleOverrides.(name))
        controls.(name) = exampleOverrides.(name);
    end
end
controls.FigureVisible = lower(string(controls.FigureVisible));
if ~isscalar(controls.FigureVisible) || ...
        ~any(controls.FigureVisible == ["on" "off"])
    error("exampleAzElPlanning:InvalidFigureVisible", ...
        "FigureVisible must be on or off.");
end
logicalNames = ["PlotOutputs" "ShowAnimation" ...
    "ShowKinematicPlot" "EnableJerkConstraint"];
for nameIndex = 1:numel(logicalNames)
    value = controls.(logicalNames(nameIndex));
    if ~(islogical(value) && isscalar(value)) && ...
            ~(isnumeric(value) && isscalar(value) && ...
            isfinite(value) && any(value == [0 1]))
        error("exampleAzElPlanning:InvalidLogicalOverride", ...
            "%s must be scalar logical or binary numeric.", ...
            logicalNames(nameIndex));
    end
    controls.(logicalNames(nameIndex)) = logical(value);
end

%% Section 2: Create Obstacles

% The 721-point circle demonstrates automatic planning-node reduction.
% Its full boundary remains untouched for collision checking.
angle_rad = (0:719).' * (2 * pi / 720);
obstacleAzimuth_deg = 2.25 * cos(angle_rad);
obstacleElevation_deg = 2.25 * sin(angle_rad);
obstacleTime_s = [0; 120];
obstacle = makeAzElObstacleData("obstacle", obstacleTime_s, ...
    obstacleAzimuth_deg, obstacleElevation_deg, 0.20);
obstacles = obstacle;

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-7 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct("time_s", 120, "position_deg", [7 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
maximumJerk_deg_s3 = [Inf Inf];
if controls.EnableJerkConstraint
    maximumJerk_deg_s3 = [2.5 2.5];
end
limits = struct("maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", maximumJerk_deg_s3);
options = struct("GoalTimeMode", "earliestArrival", ...
    "SampleTime_s", 0.05, "TurnRadius_deg", 1.0, ...
    "AllowAzimuthWrapping", false, ...
    "AzimuthInterval_deg", [-180 180], "Verbose", false);

%% Section 4: Run Planner

result = planAzElMotion(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

result.ExampleValidation = result.Validation;
if ~result.Success
    warning("exampleAzElPlanning:PlanningFailed", "%s", result.Message);
end

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if controls.PlotOutputs
    result.PlotHandles = plotAzElMotion(result, struct( ...
        "FigureVisible", controls.FigureVisible, ...
        "ShowAnimation", controls.ShowAnimation, ...
        "ShowKinematics", controls.ShowKinematicPlot, ...
        "Title", "Minimal azimuth/elevation planning example"));
end

%% Section 7: Return Example Metadata

result.ExampleControls = controls;
result.ExampleInputs = struct("obstacles", obstacles, ...
    "initialState", initialState, "goalState", goalState, ...
    "limits", limits, "options", options);
end
