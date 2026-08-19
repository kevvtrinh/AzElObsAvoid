function result = exampleUShapedAzElTimeSpace(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleUShapedAzElTimeSpace()
%   result = exampleUShapedAzElTimeSpace(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Plan from the cavity of one protected U-shaped obstacle to an exterior
%     goal without waypoints or a directed route.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Planner result, independent validation, plots, and example metrics.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveAzElExampleOptions( ...
    exampleOverrides, struct( ...
    "GoalTimeMode", "earliestArrival", "MaximumSeedCount", 3, ...
    "CollocationSegmentCount", 7, "MaximumPlanningTime_s", 35), ...
    [2.5 2.5]);

%% Section 2: Create Obstacles

missionEndTime_s = 120;
obstacleTime_s = [0; missionEndTime_s];
obstaclePosition_deg = [ ...
    -8 7; -5 7; -5 -4; 5 -4; ...
    5 7; 8 7; 8 -7; -8 -7];
safetyMargin_deg = 0.20;
obstacles = makeAzElObstacleData( ...
    "Static U-shaped obstacle", obstacleTime_s, ...
    obstaclePosition_deg(:, 1), obstaclePosition_deg(:, 2), ...
    safetyMargin_deg);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [0 -10]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

result.ExampleValidation = validateAzElExampleResult( ...
    result, "single U", struct("RequireDirectBlocked", true));

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if displayOptions.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, displayOptions.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleName = "exampleUShapedAzElTimeSpace";
result.ExampleMetrics = computeAzElExampleMetrics(result);
result.ExampleControls = displayOptions;
result.uBoundary_deg = obstaclePosition_deg;
result.ExampleGeometry = struct( ...
    "obstacleTime_s", obstacleTime_s, ...
    "obstaclePosition_deg", obstaclePosition_deg, ...
    "safetyMargin_deg", safetyMargin_deg);
end
