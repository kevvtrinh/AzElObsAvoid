function result = exampleUShapedAzElTimeSpace(options)
%EXAMPLEUSHAPEDAZELTIMESPACE Define one U and call planAzElMotion.
if nargin < 1 || isempty(options)
    options = struct();
end
options = exampleOptions(options, struct( ...
    "SafetyMarginDeg", 0.20, ...
    "MotionType", "velocityCarrying", ...
    "FigureVisible", "on", ...
    "Title", "U-shaped az/el obstacle"));

%% Obstacles
missionEndTime_s = 35;
uBoundary_deg = [ ...
    -8,  7; -5,  7; -5, -4; 5, -4; ...
     5,  7;  8,  7;  8, -7; -8, -7];
obstacle = makeAzElObstacleData( ...
    "Static U-shaped obstacle", [0; missionEndTime_s], ...
    uBoundary_deg(:, 1), uBoundary_deg(:, 2));

%% Planner input
initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [0 -10]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75]);

%% Plan
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);
result.uBoundary_deg = uBoundary_deg;

%% Validate
validateExampleResult(result, "single U");
end

function validateExampleResult(result, scenarioName)
if ~result.Success || ~result.Validation.Passed
    error("exampleUShapedAzElTimeSpace:PlanningFailed", ...
        "%s validation failed. Diagnostic plots remain open. %s", ...
        scenarioName, result.Message);
end
if ~any(result.directBlocked)
    error("exampleUShapedAzElTimeSpace:ScenarioNotBlocked", ...
        "The direct path should be blocked. Diagnostic plots remain open.");
end
end

function options = exampleOptions(options, defaults)
names = fieldnames(defaults);
for index = 1:numel(names)
    if ~isfield(options, names{index}) || isempty(options.(names{index}))
        options.(names{index}) = defaults.(names{index});
    end
end
end
