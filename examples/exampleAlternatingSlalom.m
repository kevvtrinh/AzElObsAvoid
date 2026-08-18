function result = exampleAlternatingSlalom(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleAlternatingSlalom()
%   result = exampleAlternatingSlalom(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate one input-driven route through alternating static barriers.
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
    "GoalTimeMode", "fixedArrival", "MaximumSeedCount", 3, ...
    "CollocationSegmentCount", 8, "MaximumPlanningTime_s", 40, ...
    "ElevationInterval_deg", [-6 6]));

%% Section 2: Create Obstacles

obstacleTime_s = [0; 30];
centerAzimuth_deg = [-4; 0; 4];
centerElevation_deg = [2.5; -2.5; 2.5];
obstacles = combineAzElObstacles();
for obstacleIndex = 1:numel(centerAzimuth_deg)
    center_deg = [centerAzimuth_deg(obstacleIndex), ...
        centerElevation_deg(obstacleIndex)];
    rectangle_deg = center_deg + [ ...
        -0.7 -2.5; 0.7 -2.5; 0.7 2.5; -0.7 2.5];
    obstacle = makeAzElObstacleData( ...
        "barrier " + obstacleIndex, obstacleTime_s, ...
        rectangle_deg(:, 1), rectangle_deg(:, 2), 0.1);
    obstacles = combineAzElObstacles(obstacles, obstacle);
end

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-8 0]);
goalState = struct("time_s", 22, "position_deg", [8 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2]);

%% Section 4: Run Planner

result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

result.ExampleValidation = validateAzElTrajectory(result);

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if displayOptions.PlotOutputs
    plotOptions = rmfield(displayOptions, 'PlotOutputs');
    result.PlotHandles = plotAzElMotion(result, plotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleName = "exampleAlternatingSlalom";
result.ExampleMetrics = computeAzElExampleMetrics(result);
result.ExampleControls = displayOptions;
result.ExampleGeometry = struct( ...
    "obstacleTime_s", obstacleTime_s, ...
    "centerAzimuth_deg", centerAzimuth_deg, ...
    "centerElevation_deg", centerElevation_deg);
end
