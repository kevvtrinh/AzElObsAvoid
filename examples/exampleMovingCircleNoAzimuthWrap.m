function result = exampleMovingCircleNoAzimuthWrap(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingCircleNoAzimuthWrap()
%   result = exampleMovingCircleNoAzimuthWrap(options)
%**************************************************************************
% PURPOSE
%   - Construct one protected rising circle and plan left-to-right without
%     azimuth wrapping, waypoints, a preferred side, or a directed route.
%**************************************************************************
% INPUTS
%   - options (scalar struct, optional)
%       Planner option overrides plus EnableJerkConstraint and
%       MaxJerk_deg_s3 example controls.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Validated planner result and moving-obstacle geometry.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(options)
    options = struct();
end
[options, jerkConfiguration] = resolveAzElExampleOptions( ...
    options, struct( ...
    "MotionType", "velocityCarrying", ...
    "AllowAzimuthWrapping", false, ...
    "AzimuthInterval_deg", [-180 180], ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", "Rising circle with automatic non-wrapping route"), ...
    [2.5 2.5]);
% The defining behavior of this scenario is an interval-constrained route.
% Callers may customize the interval but cannot enable wrapping here.
options.AllowAzimuthWrapping = false;

%% Section 2: Create Obstacles

% The shared horizon keeps both the obstacle rate and planner deadline
% identical when the example-only jerk switch is toggled.
missionEndTime_s = 120;
safetyMargin_deg = 0.20;
obstacleTime_s = (0:0.25:missionEndTime_s).';
circleRadius_deg = 2.5;
circleCenterElevation_deg = -3.5 + ...
    7.0 * obstacleTime_s / missionEndTime_s;
circleAngle_rad = linspace(0, 2 * pi, 73).';
circleAngle_rad(end) = [];
circleAzimuth_deg = cell(numel(obstacleTime_s), 1);
circleElevation_deg = cell(numel(obstacleTime_s), 1);
for sampleIndex = 1:numel(obstacleTime_s)
    circleAzimuth_deg{sampleIndex} = ...
        circleRadius_deg * cos(circleAngle_rad);
    circleElevation_deg{sampleIndex} = ...
        circleCenterElevation_deg(sampleIndex) + ...
        circleRadius_deg * sin(circleAngle_rad);
end
obstacle = makeAzElObstacleData( ...
    "Slowly rising circle", obstacleTime_s, ...
    circleAzimuth_deg, circleElevation_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [-12 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [12 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);

%% Section 4: Run Planner

result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);

%% Section 5: Validate Result

exampleValidation = validateAzElExampleResult( ...
    result, "moving circle", struct("RequireDirectBlocked", true));

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if jerkConfiguration.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, jerkConfiguration.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleValidation = exampleValidation;
result.obstacleTime_s = obstacleTime_s;
result.circleRadius_deg = circleRadius_deg;
result.circleCenterElevation_deg = circleCenterElevation_deg;
result.azimuthWrappingAllowed = options.AllowAzimuthWrapping;
result.azimuthInterval_deg = options.AzimuthInterval_deg;
result.ExampleConfiguration = jerkConfiguration;
end
