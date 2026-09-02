function tests = testObstacleHistoryRobustness
%% Section 0: Header & Readme
% SYNTAX
%   tests = testObstacleHistoryRobustness
%**************************************************************************
% PURPOSE
%   - Lock conservative moving-obstacle history and request-horizon behavior.
%   - Exercise representation changes, short sample spacing, status metadata,
%     and disjoint interception windows before implementation changes.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (MATLAB function-test array)
%       Deterministic obstacle-history and temporal-search regressions.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add the production planner, trajectory engine, and benchmark entry points.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "trajectory"));
end

function testEquivalentRingOrderDoesNotCreateMotion(testCase)
% Treat cyclic shifts and reversed traversal as the same occupied polygon.
base_deg = [-2 -1; 2 -1; 2 1; -2 1];
shifted_deg = circshift(base_deg, 2, 1);
reversed_deg = flipud(base_deg);
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "equivalent ring order", [0; 1; 2], ...
    {base_deg(:, 1); shifted_deg(:, 1); reversed_deg(:, 1)}, ...
    {base_deg(:, 2); shifted_deg(:, 2); reversed_deg(:, 2)}, 0);
referenceShape = polyshape(base_deg, "Simplify", false);

for queryTime_s = [0.5 1.5]
    [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
        obstacle, queryTime_s);
    verifyEqual(testCase, area(subtract(shape, referenceShape)), 0, ...
        "AbsTol", 1e-12);
    verifyEqual(testCase, area(subtract(referenceShape, shape)), 0, ...
        "AbsTol", 1e-12);
    verifyEqual(testCase, geometry.VertexSpeedBound_deg_s, 0, ...
        "AbsTol", 1e-12);
end
end

function testRotatingRectangleUsesLinearVertexMotion(testCase)
% Define sampled rotation as linear corresponding-vertex motion, not rigid arc.
lower_deg = [-2 -1; 2 -1; 2 1; -2 1];
angle_rad = deg2rad(20);
rotation = [cos(angle_rad), -sin(angle_rad); ...
    sin(angle_rad), cos(angle_rad)];
upper_deg = lower_deg * rotation.';
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "sampled rotating rectangle", [0; 2], ...
    {lower_deg(:, 1); upper_deg(:, 1)}, ...
    {lower_deg(:, 2); upper_deg(:, 2)}, 0);
[~, geometry] = obstacleAvoidance.obstacles.shapeAtTime(obstacle, 1, true);

verifyEqual(testCase, ...
    [geometry.azimuth_deg, geometry.elevation_deg], ...
    0.5 * (lower_deg + upper_deg), "AbsTol", 1e-12);
verifyTrue(testCase, geometry.TopologyIsInterpolated);
end

function testTopologyMismatchContainsBetweenSampleSweep(testCase)
% A conservative fallback must cover the gap between separated endpoints.
left_deg = [-4 -1; -2 -1; -2 1; -4 1];
rightLower_deg = [2 -1; 4 -1; 4 1; 2 1];
rightUpper_deg = [2 2; 3 2; 3 3; 2 3];
right_deg = [rightLower_deg; NaN NaN; rightUpper_deg];
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "mismatched topology translation", [0; 2], ...
    {left_deg(:, 1); right_deg(:, 1)}, ...
    {left_deg(:, 2); right_deg(:, 2)}, 0);

[occupied, ~, details] = ...
    obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacle, 0, 0, 1);
[~, geometry] = obstacleAvoidance.obstacles.shapeAtTime(obstacle, 1);
verifyTrue(testCase, occupied, ...
    "The fallback omitted the between-sample swept gap.");
verifyLessThanOrEqual(testCase, details.MinimumClearance_deg, 0);
verifyFalse(testCase, geometry.TopologyIsInterpolated);
end

function testShortSampleSpacingRetainsTimedProposalLayer(testCase)
% Preserve a 0.1-second obstacle event in the time-expanded proposal search.
lower_deg = [-0.2 1.8; 0.2 1.8; 0.2 2.2; -0.2 2.2];
middle_deg = lower_deg + [0.2 0];
upper_deg = lower_deg + [0.4 0];
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "short sampled motion", [0; 0.1; 0.2], ...
    {lower_deg(:, 1); middle_deg(:, 1); upper_deg(:, 1)}, ...
    {lower_deg(:, 2); middle_deg(:, 2); upper_deg(:, 2)}, 0);
initialState = restState(0, [-2 0]);
goalState = restState(0.2, [2 0]);
limits = physicalLimits();
options = obstacleAvoidance.input.resolvePlannerOptions(struct( ...
    "GoalTimeMode", "fixedArrival", "MaximumSeedCount", 3, ...
    "MaximumTimeLayerCount", 17));
[obstacle, initialState, goalState, limits] = ...
    obstacleAvoidance.input.normalizePlannerRequest( ...
    obstacle, initialState, goalState, limits, options);
[~, diagnostics] = obstacleAvoidance.search.createRouteCandidates( ...
    obstacleAvoidance.obstacles.prepareDynamic(obstacle), ...
    initialState, goalState, limits, options);

verifyTrue(testCase, diagnostics.Coverage.TimedSearchAttempted);
verifyTrue(testCase, any(abs( ...
    diagnostics.TemporalLayerTimes_s - 0.1) <= 16 * eps));
end

function testProjectionUsesOnlyRequestHorizon(testCase)
% Exclude remote history samples while retaining exact horizon endpoints.
source_deg = [-1 -1; 1 -1; 1 1; -1 1];
positions_deg = {source_deg + [-100 0]; source_deg; ...
    source_deg + [1 0]; source_deg + [100 0]};
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "long history", [-10; 0; 1; 10], ...
    cellfun(@(value) value(:, 1), positions_deg, ...
    "UniformOutput", false), ...
    cellfun(@(value) value(:, 2), positions_deg, ...
    "UniformOutput", false), 0);
[~, projection] = ...
    obstacleAvoidance.obstacles.createStaticPlanningProjection( ...
    obstacle, 0, 1);
boundary_deg = projection.Records.Boundary_deg;

verifyGreaterThanOrEqual(testCase, min(boundary_deg(:, 1)), -1 - 1e-12);
verifyLessThanOrEqual(testCase, max(boundary_deg(:, 1)), 2 + 1e-12);
end

function testVisibilityStatusIsConservativeMetadata(testCase)
% A non-visible status label must not silently deactivate supplied geometry.
boundary_deg = [-1 -1; 1 -1; 1 1; -1 1];
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "status metadata", [0; 1], boundary_deg(:, 1), ...
    boundary_deg(:, 2), 0);
obstacle.status = ["visible"; "hidden"];
obstacle = obstacleAvoidance.obstacles.createObstacle(obstacle);
occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacle, 0, 0, 0.75);

verifyTrue(testCase, occupied);
end

function testEarliestInterceptFindsFirstOfTwoFeasibleWindows(testCase)
% Do not assume feasibility remains monotone after the first failed trial.
blocked_deg = [3.9999 -0.0001; 4.0001 -0.0001; ...
    4.0001 0.0001; 3.9999 0.0001];
clear_deg = blocked_deg + [0 5];
obstacleTime_s = [0; 2.89; 2.90; 3.08; 3.09; 6.49; 6.50; 8; 8.01; 10];
isClear = [false; false; true; true; false; false; true; true; false; false];
positionBySlice_deg = cell(numel(obstacleTime_s), 1);
for sampleIndex = 1:numel(obstacleTime_s)
    if isClear(sampleIndex)
        positionBySlice_deg{sampleIndex} = clear_deg;
    else
        positionBySlice_deg{sampleIndex} = blocked_deg;
    end
end
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "intercept window blocker", obstacleTime_s, ...
    cellfun(@(value) value(:, 1), positionBySlice_deg, ...
    "UniformOutput", false), ...
    cellfun(@(value) value(:, 2), positionBySlice_deg, ...
    "UniformOutput", false), 0);
targetMotion = struct("time_s", [0; 10], ...
    "position_deg", [4 0; 4 0]);
options = struct("InterceptMode", "earliest", ...
    "MaximumSearchDuration_s", 10, ...
    "PlannerOptions", struct("MaximumSeedCount", 1));
result = obstacleAvoidance.planMovingTargetIntercept( ...
    obstacle, restState(0, [0 0]), targetMotion, ...
    physicalLimits(), options);

verifyTrue(testCase, result.Success, result.Message);
verifyGreaterThan(testCase, result.Intercept.Time_s, 2.89);
verifyLessThan(testCase, result.Intercept.Time_s, 3.09, ...
    "The chronological search skipped the first feasible window.");
verifyGreaterThanOrEqual(testCase, ...
    result.Intercept.Search.FeasibleWindowCount, 2);
verifyEqual(testCase, result.Intercept.Search.SelectedWindowIndex, 1);
verifyTrue(testCase, result.Intercept.Search.ObstaclePreparationReused);
verifyFalse(testCase, ...
    isfield(result.Inputs.obstacles, "InternalPreparation"));
end

function testDirectWaitFindsFirstOfTwoFeasibleWindows(testCase)
% Refine every direct-wait opening and retain the earliest validated motion.
blocked_deg = [1.9 -10; 2.1 -10; 2.1 10; 1.9 10];
clear_deg = blocked_deg + [20 0];
obstacleTime_s = [0; 2.5; 2.6; 3.4; 3.5; 5.5; 5.6; 8];
isClear = logical([0; 0; 1; 1; 0; 0; 1; 1]);
azimuthBySlice_deg = cell(numel(obstacleTime_s), 1);
elevationBySlice_deg = cell(numel(obstacleTime_s), 1);
for sampleIndex = 1:numel(obstacleTime_s)
    position_deg = blocked_deg;
    if isClear(sampleIndex)
        position_deg = clear_deg;
    end
    azimuthBySlice_deg{sampleIndex} = position_deg(:, 1);
    elevationBySlice_deg{sampleIndex} = position_deg(:, 2);
end
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "direct-wait window blocker", obstacleTime_s, ...
    azimuthBySlice_deg, elevationBySlice_deg, 0);
limits = struct( ...
    "maxVelocity_deg_s", [10 10], ...
    "maxAcceleration_deg_s2", [1000 1000], ...
    "maxJerk_deg_s3", [10000 10000], ...
    "azimuthInterval_deg", [-1 6], ...
    "elevationInterval_deg", [-10 10]);
options = struct("GoalTimeMode", "earliestArrival", ...
    "MaximumSeedCount", 3, "MaximumTimeLayerCount", 9, ...
    "SampleTime_s", 0.02);
result = obstacleAvoidance.planTrajectory( ...
    obstacle, restState(0, [0 0]), restState(8, [4 0]), ...
    limits, options);

directWaitIndex = find(string({result.Seeds.Source}) == "directWait", 1);
verifyTrue(testCase, result.Success, result.Message);
verifyNotEmpty(testCase, directWaitIndex);
verifyEqual(testCase, result.SelectedSeedIndex, directWaitIndex);
diagnostics = result.SeedSummaries(directWaitIndex).SolverDiagnostics;
verifyGreaterThanOrEqual(testCase, diagnostics.FeasibleWindowCount, 2);
verifyEqual(testCase, diagnostics.SelectedWindowIndex, 1);
verifyGreaterThan(testCase, diagnostics.FinalWaitTime_s, 2);
verifyLessThan(testCase, diagnostics.FinalWaitTime_s, 3.5, ...
    "The direct-wait refinement skipped the first feasible window.");
verifyEqual(testCase, diagnostics.HorizonProjectionKey, ...
    "candidateTimeRange_s");
verifyFalse(testCase, result.SearchDiagnostics.SeedEarlyExit.Applied);
verifyEqual(testCase, result.SearchDiagnostics.SeedEarlyExit.Reason, ...
    "lowerBoundNotReached");

balancedOptions = options;
balancedOptions.GoalTimeMode = "balancedArrival";
balancedResult = obstacleAvoidance.planTrajectory( ...
    obstacle, restState(0, [0 0]), restState(8, [4 0]), ...
    limits, balancedOptions);
balancedDirectWaitIndex = find( ...
    string({balancedResult.Seeds.Source}) == "directWait", 1);
verifyTrue(testCase, balancedResult.Success, balancedResult.Message);
verifyNotEmpty(testCase, balancedDirectWaitIndex);
balancedDiagnostics = balancedResult.SeedSummaries( ...
    balancedDirectWaitIndex).SolverDiagnostics;
verifyGreaterThan(testCase, balancedDiagnostics.RefinementCount, 0);
verifyEqual(testCase, balancedDiagnostics.SelectedWindowIndex, 1);
verifyLessThan(testCase, balancedDiagnostics.FinalWaitTime_s, 3.5, ...
    "Balanced-arrival ranking retained an avoidable later wait.");
end

function state = restState(time_s, position_deg)
% Create one explicit two-axis rest state.
state = struct("time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
end

function limits = physicalLimits()
% Use generous dynamics so geometry, not travel time, controls these tests.
limits = struct( ...
    "maxVelocity_deg_s", [4 4], ...
    "maxAcceleration_deg_s2", [4 4], ...
    "maxJerk_deg_s3", [20 20], ...
    "azimuthInterval_deg", [-120 120], ...
    "elevationInterval_deg", [-20 20]);
end
