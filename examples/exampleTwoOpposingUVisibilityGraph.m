function result = exampleTwoOpposingUVisibilityGraph(options)
%EXAMPLETWOOPPOSINGUVISIBILITYGRAPH Define two Us and call the planner.
if nargin < 1 || isempty(options)
    options = struct();
end
options = exampleOptions(options, struct( ...
    "SafetyMarginDeg", 0.10, ...
    "MotionType", "velocityCarrying", ...
    "FigureVisible", "on", ...
    "Title", "Two opposing U-shaped az/el obstacles"));

%% Obstacles
missionEndTime_s = 35;
firstUBoundary_deg = [ ...
    -10, 8; 0, 8; 0, 5; -7, 5; ...
     -7,-5; 0,-5; 0,-8; -10,-8];
secondUBoundary_deg = [ ...
      5,28; 15,28; 15,12; 5,12; ...
      5,15; 12,15; 12,25; 5,25];
time_s = [0; missionEndTime_s];
obstacles = [ ...
    makeAzElObstacleData("Right-opening U", time_s, ...
        firstUBoundary_deg(:, 1), firstUBoundary_deg(:, 2)); ...
    makeAzElObstacleData("Left-opening U", time_s, ...
        secondUBoundary_deg(:, 1), secondUBoundary_deg(:, 2))];

%% Planner input
initialState = struct("time_s", 0, "position_deg", [-4 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [9 20]);
limits = struct( ...
    "maxVelocity_deg_s", [1 1], ...
    "maxAcceleration_deg_s2", [0.75 0.75]);

%% Plan
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
result.firstUBoundary_deg = firstUBoundary_deg;
result.secondUBoundary_deg = secondUBoundary_deg;

%% Validate
validateExampleResult(result, "two opposing Us");
end

function validateExampleResult(result, scenarioName)
if ~result.Success || ~result.Validation.Passed
    error("exampleTwoOpposingUVisibilityGraph:PlanningFailed", ...
        "%s validation failed. Diagnostic plots remain open. %s", ...
        scenarioName, result.Message);
end
if ~any(result.directBlocked)
    error("exampleTwoOpposingUVisibilityGraph:ScenarioNotBlocked", ...
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
