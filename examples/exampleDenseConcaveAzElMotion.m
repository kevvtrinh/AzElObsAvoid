function result = exampleDenseConcaveAzElMotion(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleDenseConcaveAzElMotion()
%   result = exampleDenseConcaveAzElMotion(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate protected dense concave geometry without special waypoints.
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
    "GoalTimeMode", "earliestArrival", "MaximumSeedCount", 2, ...
    "CollocationSegmentCount", 7, "MaximumPlanningTime_s", 60));

%% Section 2: Create Obstacles

vertexCount = 80;
angle_rad = (0:vertexCount - 1).' * (2 * pi / vertexCount);
radius_deg = 1.6 + 0.45 * cos(5 * angle_rad);
obstacleAzimuth_deg = radius_deg .* cos(angle_rad);
obstacleElevation_deg = radius_deg .* sin(angle_rad);
obstacleTime_s = [0; 20];
safetyMargin_deg = 0.1;
obstacles = makeAzElObstacleData( ...
    "dense concave polygon", obstacleTime_s, ...
    obstacleAzimuth_deg, obstacleElevation_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-6 0]);
goalState = struct("time_s", 15, "position_deg", [6 0]);
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
    result.PlotHandles = plotAzElMotion( ...
        result, displayOptions.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleName = "exampleDenseConcaveAzElMotion";
result.ExampleMetrics = computeAzElExampleMetrics(result);
result.ExampleControls = displayOptions;
result.ExampleGeometry = struct( ...
    "obstacleTime_s", obstacleTime_s, ...
    "obstacleAzimuth_deg", obstacleAzimuth_deg, ...
    "obstacleElevation_deg", obstacleElevation_deg, ...
    "safetyMargin_deg", safetyMargin_deg);
end
