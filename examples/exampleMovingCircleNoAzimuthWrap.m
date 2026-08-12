function result = exampleMovingCircleNoAzimuthWrap(options)
%EXAMPLEMOVINGCIRCLENOAZIMUTHWRAP Define a rising circle and call planner.
% No preferred side, detour waypoint, or directed route is supplied.
if nargin < 1 || isempty(options)
    options = struct();
end
options = exampleOptions(options, struct( ...
    "SafetyMarginDeg", 0.20, ...
    "MotionType", "velocityCarrying", ...
    "AllowAzimuthWrapping", false, ...
    "AzimuthInterval_deg", [-180 180], ...
    "FigureVisible", "on", ...
    "Title", "Rising circle with automatic non-wrapping route"));
missionEndTime_s = 35;
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
    circleAzimuth_deg, circleElevation_deg);
initialState = stateAt(0, [-12 0]);
goalState = stateAt(missionEndTime_s, [12 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75]);

result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);
result.obstacleTime_s = obstacleTime_s;
result.circleRadius_deg = circleRadius_deg;
result.circleCenterElevation_deg = circleCenterElevation_deg;
result.azimuthWrappingAllowed = false;
result.azimuthInterval_deg = options.AzimuthInterval_deg;
end

function state = stateAt(time_s, position_deg)
state = struct( ...
    "time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
end

function options = exampleOptions(options, defaults)
names = fieldnames(defaults);
for index = 1:numel(names)
    if ~isfield(options, names{index}) || isempty(options.(names{index}))
        options.(names{index}) = defaults.(names{index});
    end
end
end
