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
%       Planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Validated planner result and moving-obstacle geometry.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
if nargin < 1 || isempty(options)
    options = struct();
end
options = exampleOptions(options, struct( ...
    "MotionType", "velocityCarrying", ...
    "AllowAzimuthWrapping", false, ...
    "AzimuthInterval_deg", [-180 180], ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", "Rising circle with automatic non-wrapping route"));
% The defining behavior of this scenario is an interval-constrained route.
% Callers may customize the interval but cannot enable wrapping here.
options.AllowAzimuthWrapping = false;

%% Section 1: Construct Canonical Obstacles
missionEndTime_s = 35;
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

%% Section 2: Define The Planning Request
initialState = struct("time_s", 0, "position_deg", [-12 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [12 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75]);

%% Section 3: Run The Maintained Planner
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);
result.obstacleTime_s = obstacleTime_s;
result.circleRadius_deg = circleRadius_deg;
result.circleCenterElevation_deg = circleCenterElevation_deg;
result.azimuthWrappingAllowed = options.AllowAzimuthWrapping;
result.azimuthInterval_deg = options.AzimuthInterval_deg;

%% Section 4: Validate The Command
validateExampleResult(result, "moving circle");
end

function validateExampleResult(result, scenarioName)
%% Section 0: Header & Readme
if ~result.Success || ~result.Validation.Passed
    error("exampleMovingCircleNoAzimuthWrap:PlanningFailed", ...
        "%s validation failed. Diagnostic plots remain open. %s", ...
        scenarioName, result.Message);
end
if ~any(result.directBlocked)
    error("exampleMovingCircleNoAzimuthWrap:ScenarioNotBlocked", ...
        "The direct path should be blocked. Diagnostic plots remain open.");
end
end

function options = exampleOptions(options, defaults)
%% Section 0: Header & Readme
names = fieldnames(defaults);
for index = 1:numel(names)
    if ~isfield(options, names{index}) || isempty(options.(names{index}))
        options.(names{index}) = defaults.(names{index});
    end
end
end
