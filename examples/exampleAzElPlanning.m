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
%       ShowAnimation, ShowKinematicPlot, and ShowVisibilityGraphs select
%       returned figures;
%       MaxJerk_deg_s3 sets the required finite jerk limit; known
%       planAzElMotion options are forwarded unchanged.
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

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, jerkConfiguration] = resolveAzElExampleOptions( ...
    exampleOverrides, struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "SampleTime_s", 0.05, ...
    "AllowAzimuthWrapping", false, ...
    "AzimuthInterval_deg", [-180 180], ...
    "UseParallel", false, ...
    "Verbose", false, ...
    "FigureVisible", "on", ...
    "Title", "Minimal azimuth/elevation planning example"), ...
    [2.5 2.5]);

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
limits = struct("maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);

%% Section 4: Run Planner

result = planAzElMotion(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

result.ExampleValidation = validateAzElExampleResult( ...
    result, "minimal planning example", ...
    struct("RequireDirectBlocked", true));
if ~result.Success
    warning("exampleAzElPlanning:PlanningFailed", "%s", result.Message);
end

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if jerkConfiguration.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, jerkConfiguration.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleConfiguration = jerkConfiguration;
result.ExampleInputs = struct("obstacles", obstacles, ...
    "initialState", initialState, "goalState", goalState, ...
    "limits", limits, "options", options);
end
