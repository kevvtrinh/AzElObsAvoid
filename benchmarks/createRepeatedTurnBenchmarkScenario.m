function [obstacles, initialState, goalState, limits] = createRepeatedTurnBenchmarkScenario(turnCount, constants)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacles, initialState, goalState, limits] = createRepeatedTurnBenchmarkScenario(turnCount, constants)
%**************************************************************************
% PURPOSE
%   - Build one scale-controlled static alternating-barrier request shared by
%     motion-construction benchmarks.
%**************************************************************************
% INPUTS
%   - turnCount (positive integer scalar)
%       Number of alternating barriers and required geometric turns.
%   - constants (scalar struct)
%       barrierSpacing_deg, barrierHalfWidth_deg,
%       barrierCenterMagnitude_deg, barrierHalfHeight_deg,
%       safetyMargin_deg, goalTimePerStage_s, maxVelocity_deg_s,
%       maxAcceleration_deg_s2, maxJerk_deg_s3, and
%       elevationInterval_deg are required.
%**************************************************************************
% OUTPUTS
%   - obstacles (canonical protected obstacle struct array)
%   - initialState (scalar rest initial state)
%   - goalState (scalar rest latest-arrival goal state)
%   - limits (scalar workspace and physical-limit struct)
%**************************************************************************
% UNITS
%   - Geometry is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************

%% Section 1: Validate Benchmark Controls

validateattributes(turnCount, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
requiredNames = [ ...
    "barrierSpacing_deg", "barrierHalfWidth_deg", ...
    "barrierCenterMagnitude_deg", "barrierHalfHeight_deg", ...
    "safetyMargin_deg", "goalTimePerStage_s", ...
    "maxVelocity_deg_s", "maxAcceleration_deg_s2", "maxJerk_deg_s3", "elevationInterval_deg"];
if ~isstruct(constants) || ~isscalar(constants) || ~all(isfield(constants, requiredNames))
    error("createRepeatedTurnBenchmarkScenario:InvalidConstants", ...
        "constants must be scalar and contain every documented field.");
end
positiveScalarNames = [ ...
    "barrierSpacing_deg", "barrierHalfWidth_deg", ...
    "barrierCenterMagnitude_deg", "barrierHalfHeight_deg", "goalTimePerStage_s"];

% Apply the same finite-positive contract to every scalar geometry and timing constant.
for fieldName = positiveScalarNames
    validateattributes(constants.(fieldName), {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
end
validateattributes(constants.safetyMargin_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
limitNames = ["maxVelocity_deg_s", "maxAcceleration_deg_s2", "maxJerk_deg_s3"];

% Check both axes of every derivative limit so one invalid axis cannot enter the benchmark.
for fieldName = limitNames
    validateattributes(constants.(fieldName), {'numeric'}, {'real', 'finite', 'vector', 'numel', 2, 'positive'});
end
validateattributes(constants.elevationInterval_deg, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2, 'increasing'});

%% Section 2: Construct Alternating Protected Geometry

barrierIndices = (1:turnCount).';
centerAzimuth_deg = constants.barrierSpacing_deg * (barrierIndices - (turnCount + 1) / 2);
centerElevation_deg = constants.barrierCenterMagnitude_deg * (-1).^(barrierIndices - 1);
startAzimuth_deg = centerAzimuth_deg(1) - constants.barrierSpacing_deg;
goalAzimuth_deg = centerAzimuth_deg(end) + constants.barrierSpacing_deg;
goalTime_s = constants.goalTimePerStage_s * (turnCount + 1);
obstacleTime_s = [0; goalTime_s];
obstacles = combineAzElObstacles();

% Build one protected barrier at each alternating center to create the requested turn count.
for obstacleIndex = 1:turnCount
    center_deg = [centerAzimuth_deg(obstacleIndex), centerElevation_deg(obstacleIndex)];
    rectangle_deg = center_deg + [ ...
        -constants.barrierHalfWidth_deg, ...
        -constants.barrierHalfHeight_deg; ...
        constants.barrierHalfWidth_deg, ...
        -constants.barrierHalfHeight_deg; ...
        constants.barrierHalfWidth_deg, ...
        constants.barrierHalfHeight_deg; -constants.barrierHalfWidth_deg, constants.barrierHalfHeight_deg];
    obstacle = makeAzElObstacleData( ...
        "alternating barrier " + obstacleIndex, obstacleTime_s, ...
        rectangle_deg(:, 1), rectangle_deg(:, 2), constants.safetyMargin_deg);
    obstacles = combineAzElObstacles(obstacles, obstacle);
end

%% Section 3: Assemble Planner-Role Inputs

initialState = struct( ...
    "time_s", 0, "position_deg", [startAzimuth_deg 0], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", goalTime_s, "position_deg", [goalAzimuth_deg 0], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", constants.maxVelocity_deg_s, ...
    "maxAcceleration_deg_s2", constants.maxAcceleration_deg_s2, ...
    "maxJerk_deg_s3", constants.maxJerk_deg_s3, ...
    "azimuthInterval_deg", ...
    [startAzimuth_deg - 1, goalAzimuth_deg + 1], "elevationInterval_deg", constants.elevationInterval_deg);
end
