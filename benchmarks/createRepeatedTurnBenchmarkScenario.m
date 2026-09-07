function [obstacles, initialState, goalState, limits, constants] = createRepeatedTurnBenchmarkScenario(turnCount, constants)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacles, initialState, goalState, limits, constants] = ...
%       createRepeatedTurnBenchmarkScenario(turnCount)
%   [obstacles, initialState, goalState, limits, constants] = ...
%       createRepeatedTurnBenchmarkScenario(turnCount, constants)
%**************************************************************************
% PURPOSE
%   - Build one scale-controlled static alternating-barrier request shared by
%     motion-construction benchmarks.
%**************************************************************************
% INPUTS
%   - turnCount (positive integer scalar)
%       Number of alternating barriers and required geometric turns.
%   - constants (scalar struct, optional; default maintained constants)
%       barrierSpacing_units, barrierHalfWidth_units,
%       barrierCenterMagnitude_units, barrierHalfHeight_units,
%       safetyMargin_units, goalTimePerStage_s, maxVelocity_units_s,
%       maxAcceleration_units_s2, maxJerk_units_s3, and
%       yInterval_units are required.
%**************************************************************************
% OUTPUTS
%   - obstacles (canonical protected obstacle struct array)
%   - initialState (scalar rest initial state)
%   - goalState (scalar rest latest-arrival goal state)
%   - limits (scalar workspace and physical-limit struct)
%   - constants (resolved scalar scenario-constant struct)
%**************************************************************************
% UNITS
%   - Geometry is coordinate units; time is seconds; derivatives use units/s powers.
%**************************************************************************

%% Section 1: Validate Benchmark Controls

% Reject invalid dimensions and timing values before obstacle construction.
% An error in this section points to benchmark setup, not planner behavior.

validateattributes(turnCount, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
if nargin < 2 || isempty(constants)
    constants = defaultConstants();
end
requiredNames = [ ...
    "barrierSpacing_units", "barrierHalfWidth_units", ...
    "barrierCenterMagnitude_units", "barrierHalfHeight_units", ...
    "safetyMargin_units", "goalTimePerStage_s", ...
    "maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3", "yInterval_units"];
if ~isstruct(constants) || ~isscalar(constants) || ~all(isfield(constants, requiredNames))
    error("createRepeatedTurnBenchmarkScenario:InvalidConstants", "constants must be scalar and contain every documented field.");
end
positiveScalarNames = [ ...
    "barrierSpacing_units", "barrierHalfWidth_units", ...
    "barrierCenterMagnitude_units", "barrierHalfHeight_units", "goalTimePerStage_s"];

% Apply the same finite and positive check to each geometry and timing value.
for fieldName = positiveScalarNames
    validateattributes(constants.(fieldName), {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
end
validateattributes(constants.safetyMargin_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
limitNames = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"];

% Check both axes of every derivative limit so one invalid axis cannot enter the benchmark.
for fieldName = limitNames
    validateattributes(constants.(fieldName), {'numeric'}, {'real', 'finite', 'vector', 'numel', 2, 'positive'});
end
validateattributes(constants.yInterval_units, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2, 'increasing'});

%% Section 2: Construct Alternating Protected Geometry

% Alternate obstacles above and below the centerline. This forces repeated
% turns as obstacle count grows. The layout measures scaling without changing
% the type of planning problem between cases.

barrierIndices      = (1:turnCount).';
centerX_units   = constants.barrierSpacing_units * (barrierIndices - (turnCount + 1) / 2);
centerY_units = constants.barrierCenterMagnitude_units * (-1).^(barrierIndices - 1);
startX_units    = centerX_units(1) - constants.barrierSpacing_units;
goalX_units     = centerX_units(end) + constants.barrierSpacing_units;
goalTime_s          = constants.goalTimePerStage_s * (turnCount + 1);
obstacleTime_s      = [0; goalTime_s];
obstacles           = obstacleAvoidance.obstacles.combineObstacles();

% Build one protected barrier at each alternating center to create the requested turn count.
for obstacleIndex = 1:turnCount
    center_units    = [centerX_units(obstacleIndex), centerY_units(obstacleIndex)];
    rectangle_units = center_units + [ -constants.barrierHalfWidth_units, -constants.barrierHalfHeight_units; constants.barrierHalfWidth_units, -constants.barrierHalfHeight_units; constants.barrierHalfWidth_units, constants.barrierHalfHeight_units; -constants.barrierHalfWidth_units, constants.barrierHalfHeight_units];
    obstacle      = obstacleAvoidance.obstacles.createObstacle("alternating barrier " + obstacleIndex, obstacleTime_s, rectangle_units(:, 1), rectangle_units(:, 2), constants.safetyMargin_units);
    obstacles     = obstacleAvoidance.obstacles.combineObstacles(obstacles, obstacle);
end

%% Section 3: Assemble Planner-Role Inputs

% Return the same input roles used by the public planner. Keep scenario values
% outside planner logic so the benchmark cannot influence route decisions.

initialState = struct("time_s", 0, "position_units", [startX_units 0], "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
goalState = struct("time_s", goalTime_s, "position_units", [goalX_units 0], "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
limits = struct("maxVelocity_units_s", constants.maxVelocity_units_s, ...
    "maxAcceleration_units_s2", constants.maxAcceleration_units_s2, ...
    "maxJerk_units_s3", constants.maxJerk_units_s3, ...
    "xInterval_units", ...
    [startX_units - 1, goalX_units + 1], "yInterval_units", constants.yInterval_units);
end

%% Section 4: Local Functions

function constants = defaultConstants()
    % Define the maintained repeated-turn geometry and physical limits once.
    constants = struct();
    constants.barrierSpacing_units         = 4;
    constants.barrierHalfWidth_units       = 0.7;
    constants.barrierCenterMagnitude_units = 2.5;
    constants.barrierHalfHeight_units      = 2.5;
    constants.safetyMargin_units           = 0.1;
    constants.goalTimePerStage_s         = 5.5;
    constants.maxVelocity_units_s          = [2 2];
    constants.maxAcceleration_units_s2     = [1 1];
    constants.maxJerk_units_s3             = [2 2];
    constants.yInterval_units      = [-5 5];
end
