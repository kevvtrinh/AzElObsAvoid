function result = exampleUShapedAzElTimeSpace(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleUShapedAzElTimeSpace()
%   result = exampleUShapedAzElTimeSpace(options)
%**************************************************************************
% PURPOSE
%   - Construct one protected U-shaped obstacle and run the maintained
%     planner without waypoints or a directed route.
%**************************************************************************
% INPUTS
%   - options (scalar struct, optional)
%       Planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Validated planner result and scenario geometry.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
if nargin < 1 || isempty(options)
    options = struct();
end
options = exampleOptions(options, struct( ...
    "MotionType", "velocityCarrying", ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", "U-shaped az/el obstacle"));

%% Section 1: Construct Canonical Obstacles
missionEndTime_s = 35;
safetyMargin_deg = 0.20;
uBoundary_deg = [ ...
    -8,  7; -5,  7; -5, -4; 5, -4; ...
     5,  7;  8,  7;  8, -7; -8, -7];
obstacle = makeAzElObstacleData( ...
    "Static U-shaped obstacle", [0; missionEndTime_s], ...
    uBoundary_deg(:, 1), uBoundary_deg(:, 2), safetyMargin_deg);

%% Section 2: Define The Planning Request
initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [0 -10]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75]);

%% Section 3: Run The Maintained Planner
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);
result.uBoundary_deg = uBoundary_deg;

%% Section 4: Validate The Command
validateExampleResult(result, "single U");
end

function validateExampleResult(result, scenarioName)
%% Section 0: Header & Readme
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
%% Section 0: Header & Readme
names = fieldnames(defaults);
for index = 1:numel(names)
    if ~isfield(options, names{index}) || isempty(options.(names{index}))
        options.(names{index}) = defaults.(names{index});
    end
end
end
