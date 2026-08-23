function result = exampleMovingCircleNoAzimuthWrap(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingCircleNoAzimuthWrap()
%   result = exampleMovingCircleNoAzimuthWrap(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate an immediate non-wrapping detour around a rising circle.
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
    "GoalTimeMode", "earliestArrival", "MaximumSeedCount", 3, "AllowAzimuthWrapping", false), [2 2]);

%% Section 2: Create Obstacles

obstacleTime_s = [0; 15];
circleCenterElevation_deg = [0; 3];
circleAngle_rad = (0:23).' * (2 * pi / 24);
circleRadius_deg = 1.5;
azimuthBySlice_deg = cell(2, 1);
elevationBySlice_deg = cell(2, 1);

% Create the circle at both sampled elevations while keeping its azimuth outline unchanged.
for sampleIndex = 1:2
    azimuthBySlice_deg{sampleIndex} = circleRadius_deg * cos(circleAngle_rad);
    elevationBySlice_deg{sampleIndex} = circleCenterElevation_deg(sampleIndex) + ...
        circleRadius_deg * sin(circleAngle_rad);
end
safetyMargin_deg = 0.1;
obstacles = makeAzElObstacleData( ...
    "rising circle", obstacleTime_s, azimuthBySlice_deg, elevationBySlice_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-6 0]);
goalState = struct("time_s", 15, "position_deg", [6 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

result = planAzElMotion( obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

result.ExampleValidation = validateAzElTrajectory(result);

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if displayOptions.PlotOutputs
    result.PlotHandles = plotAzElMotion( result, displayOptions.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleName = "exampleMovingCircleNoAzimuthWrap";
result.ExampleMetrics = computeAzElExampleMetrics(result);
result.ExampleControls = displayOptions;
result.ExampleGeometry = struct( ...
    "obstacleTime_s", obstacleTime_s, ...
    "circleCenterElevation_deg", circleCenterElevation_deg, ...
    "circleRadius_deg", circleRadius_deg, "safetyMargin_deg", safetyMargin_deg);
end
