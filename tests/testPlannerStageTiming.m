function tests = testPlannerStageTiming
%% Section 0: Header & Readme
% Verify exclusive stage timing for the maintained obstacle planner.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Create common planner inputs for timing checks. Named stages must be
% nonnegative, exclusive, and consistent with total elapsed time. An over-count
% points to nested timers. Missing time points to an unrecorded branch.
% Add production and maintained example entry points.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "trajectory"));
addpath(fullfile(repositoryRoot, "examples"));
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testValidatorSeparatesCollisionActivity(testCase)
% Keep nested collision work a subset rather than a second additive stage.
initialState = state(0, [-1 0]);
initialState.velocity_deg_s = [2 0];
goalState = state(1, [1 0]);
goalState.velocity_deg_s = [2 0];
limits = physicalLimits();
limits.maxVelocity_deg_s = [3 3];
trajectory = linearTrajectory(initialState, goalState);
obstacle = rectangleObstacle([0 1], [-0.1 0.1 -1 1], 0);
    validation = obstacleAvoidance.validateTrajectory( ...
    trajectory, obstacle, initialState, goalState, limits, fixedHs3Options());

verifyFalse(testCase, validation.Passed);
verifyGreaterThan(testCase, validation.CollisionIntervalCount, 0);
verifyEqual(testCase, validation.UnresolvedIntervalCount, 0);
verifyGreaterThanOrEqual(testCase, ...
    validation.CollisionCheckingElapsedTime_s, 0);
verifyLessThanOrEqual(testCase, ...
    validation.CollisionCheckingElapsedTime_s, ...
    validation.ElapsedTime_s + timingTolerance(validation.ElapsedTime_s));
end

function testUnresolvedCollisionReportsProofBounds(testCase)
% Preserve the limiting interval and speed evidence on a fail-closed exit.
initialState = state(0, [-1 0]);
initialState.velocity_deg_s = [2 0];
goalState = state(1, [1 0]);
goalState.velocity_deg_s = [2 0];
limits = physicalLimits();
limits.maxVelocity_deg_s = [3 3];
options = fixedHs3Options();
options.CollisionMinimumTimeStep_s = 1;
trajectory = linearTrajectory(initialState, goalState);
nearMiss = rectangleObstacle([0 1], [-0.1 0.1 0.2 0.3], 0);

validation = obstacleAvoidance.validateTrajectory( ...
    trajectory, nearMiss, initialState, goalState, limits, options);
diagnostics = validation.CollisionDiagnostics;

verifyFalse(testCase, validation.CollisionFree);
verifyFalse(testCase, validation.CollisionResolved);
verifyEqual(testCase, validation.UnresolvedIntervalCount, 1);
verifyEqual(testCase, diagnostics.TerminationReason, ...
    "minimumTimeStepUnresolved");
verifyEqual(testCase, diagnostics.LastUnresolvedInterval_s, [0 1]);
verifyEqual(testCase, diagnostics.LastUnresolvedObstacleIndex, 1);
verifyEqual(testCase, diagnostics.LastPathSpeedBound_deg_s, 2, ...
    "AbsTol", 1e-10);
verifyEqual(testCase, diagnostics.LastObstacleSpeedBound_deg_s, 0);
verifyGreaterThan(testCase, diagnostics.LastRequiredClearance_deg, ...
    diagnostics.LastObservedClearance_deg);
verifyEqual(testCase, diagnostics.MinimumTimeStep_s, 1);
end

function testSuccessAndEndpointFailureShareTiming(testCase)
% Require timing on a solved request and critical early exit.
initialState = state(0, [0 0]);
goalState = state(4, [1 0]);
limits = physicalLimits();
options = fixedHs3Options();
success = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
blockingObstacle = rectangleObstacle([0 4], [-1 1 -1 1], 0);
failure = obstacleAvoidance.planTrajectory( ...
    blockingObstacle, initialState, goalState, limits, options);

verifyTrue(testCase, success.Success, success.Message);
verifyTrue(testCase, success.Validation.Passed, success.Validation.Message);
verifyFalse(testCase, isfield(success, "SelectedMotionSource"));
verifyFalse(testCase, failure.Success);
verifyEqual(testCase, failure.TerminationReason, "endpointBlocked");
verifyEqual(testCase, ...
    fieldnames(success.SearchDiagnostics.StageTiming), ...
    fieldnames(failure.SearchDiagnostics.StageTiming));
verifyStageTiming(testCase, success);
verifyStageTiming(testCase, failure);
verifyEqual(testCase, ...
    failure.SearchDiagnostics.StageTiming.TopologyElapsedTime_s, 0);
end

function testMotionSolverWorkReconcilesTiming(testCase)
% Keep selected-engine solver and validation work visible in exclusive stages.
initialState = state(0, [0 0]);
goalState = state(3, [1 0]);
limits = physicalLimits();
options = fixedHs3Options();
farObstacle = rectangleObstacle([0 3], [-100 -90 70 80], 0);
result = obstacleAvoidance.planTrajectory( ...
    farObstacle, initialState, goalState, limits, options);

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyFalse(testCase, isfield(result, "SelectedMotionSource"));
summary = result.SeedSummaries(result.SelectedSeedIndex);
verifyTrue(testCase, isstruct(summary.SolverDiagnostics));
verifyTrue(testCase, isfield(summary.SolverDiagnostics, "ElapsedTime_s"));
verifyGreaterThan(testCase, summary.SeedPlanningElapsedTime_s, 0);
verifyGreaterThan(testCase, ...
    result.SearchDiagnostics.StageTiming.MotionSolvingElapsedTime_s, 0);
verifyStageTiming(testCase, result);
end

function testFinalizerReportsOverAttribution(testCase)
% Preserve a valid result while exposing finite stage-accounting disagreement.
timing = obstacleAvoidance.planner.stageTiming();
timing.MotionSolvingElapsedTime_s = 2;

reconciled = obstacleAvoidance.planner.stageTiming(timing, 1);
verifyFalse(testCase, reconciled.TimingAccountingValid);
verifyEqual(testCase, reconciled.TimingAccountingResidual_s, -1);
verifyEqual(testCase, reconciled.UnattributedElapsedTime_s, 0);

result = struct("Success", true, ...
    "SearchDiagnostics", struct(), "ElapsedPlanningTime_s", NaN);
planningTimer = tic;
timing.MotionSolvingElapsedTime_s = 10;
finalized = obstacleAvoidance.planner.stageTiming( ...
    result, planningTimer, timing);
verifyTrue(testCase, finalized.Success);
verifyFalse(testCase, ...
    finalized.SearchDiagnostics.StageTiming.TimingAccountingValid);
verifyLessThan(testCase, ...
    finalized.SearchDiagnostics.StageTiming.TimingAccountingResidual_s, 0);
end

function testFinalizerRejectsCorruptTimingValues(testCase)
% Continue to throw for malformed, negative, or nonfinite timing input.
timing = obstacleAvoidance.planner.stageTiming();
negativeTiming = timing;
negativeTiming.MotionSolvingElapsedTime_s = -1;
verifyError(testCase, @() ...
    obstacleAvoidance.planner.stageTiming(negativeTiming, 1), ...
    "stageTiming:InvalidTimingValue");
nonfiniteTiming = timing;
nonfiniteTiming.MotionSolvingElapsedTime_s = NaN;
verifyError(testCase, @() ...
    obstacleAvoidance.planner.stageTiming(nonfiniteTiming, 1), ...
    "stageTiming:InvalidTimingValue");
verifyError(testCase, @() ...
    obstacleAvoidance.planner.stageTiming(timing, Inf), ...
    "stageTiming:InvalidTotalElapsedTime");
end

function verifyStageTiming(testCase, result)
% Verify exact shared field ownership and top-level reconciliation.
requiredNames = [ ...
    "TopologyElapsedTime_s"; "CorridorConstructionElapsedTime_s"; ...
    "MotionSolvingElapsedTime_s"; ...
    "CollisionCheckingElapsedTime_s"; ...
    "FinalValidationElapsedTime_s"; ...
    "UnattributedElapsedTime_s"; "TotalElapsedTime_s"; ...
    "TimingAccountingValid"; "TimingAccountingResidual_s"];
timing = result.SearchDiagnostics.StageTiming;
verifyEqual(testCase, string(fieldnames(timing)), requiredNames);
verifyAdditiveTiming(testCase, timing);
verifyEqual(testCase, timing.TotalElapsedTime_s, ...
    result.ElapsedPlanningTime_s, ...
    "AbsTol", timingTolerance(timing.TotalElapsedTime_s));
end

function verifyAdditiveTiming(testCase, timing)
% Verify finite stages plus explicit signed accounting residual evidence.
durationNames = ["TopologyElapsedTime_s"; ...
    "CorridorConstructionElapsedTime_s"; ...
    "MotionSolvingElapsedTime_s"; ...
    "CollisionCheckingElapsedTime_s"; ...
    "FinalValidationElapsedTime_s"; ...
    "UnattributedElapsedTime_s"; "TotalElapsedTime_s"];
for name = durationNames.'
    value = timing.(name);
    verifyTrue(testCase, ...
        isnumeric(value) && isreal(value) && isscalar(value));
    verifyTrue(testCase, isfinite(value));
    verifyGreaterThanOrEqual(testCase, value, 0);
end
verifyTrue(testCase, islogical(timing.TimingAccountingValid) && ...
    isscalar(timing.TimingAccountingValid));
verifyTrue(testCase, isnumeric(timing.TimingAccountingResidual_s) && ...
    isreal(timing.TimingAccountingResidual_s) && ...
    isscalar(timing.TimingAccountingResidual_s) && ...
    isfinite(timing.TimingAccountingResidual_s));
exclusiveElapsedTime_s = sum([timing.TopologyElapsedTime_s, ...
    timing.CorridorConstructionElapsedTime_s, ...
    timing.MotionSolvingElapsedTime_s, ...
    timing.CollisionCheckingElapsedTime_s, ...
    timing.FinalValidationElapsedTime_s]);
unattributedElapsedTime_s = timing.UnattributedElapsedTime_s;
totalElapsedTime_s = timing.TotalElapsedTime_s;
tolerance_s = timingTolerance(totalElapsedTime_s);
verifyLessThanOrEqual(testCase, ...
    exclusiveElapsedTime_s, totalElapsedTime_s + tolerance_s);
verifyTrue(testCase, timing.TimingAccountingValid);
verifyEqual(testCase, timing.TimingAccountingResidual_s, ...
    totalElapsedTime_s - exclusiveElapsedTime_s, "AbsTol", tolerance_s);
verifyEqual(testCase, unattributedElapsedTime_s, ...
    max(0, totalElapsedTime_s - exclusiveElapsedTime_s), ...
    "AbsTol", tolerance_s);
verifyEqual(testCase, exclusiveElapsedTime_s + ...
    unattributedElapsedTime_s, totalElapsedTime_s, ...
    "AbsTol", tolerance_s);
end

function tolerance_s = timingTolerance(totalElapsedTime_s)
% Use a clock-accounting tolerance, not a performance threshold.
tolerance_s = max(1e-6, 64 * eps(max(1, totalElapsedTime_s)));
end

function options = fixedHs3Options()
% Return deterministic HS3 controls for timing tests.
options = obstacleAvoidance.planTrajectory();
options.GoalTimeMode = "fixedArrival";
options.MaximumSeedCount = 1;
options.SampleTime_s = 0.05;
end

function value = state(time_s, position_deg)
% Construct one stationary-derivative endpoint.
value = struct( ...
    "time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
end

function limits = physicalLimits()
% Construct shared two-axis workspace and derivative limits.
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2], ...
    "azimuthInterval_deg", [-180 180], ...
    "elevationInterval_deg", [-90 90]);
end

function obstacle = rectangleObstacle(time_s, bounds_deg, margin_deg)
% Construct a static rectangle from [minAz maxAz minEl maxEl].
azimuth_deg = bounds_deg([1 2 2 1]).';
elevation_deg = bounds_deg([3 3 4 4]).';
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "rectangle", time_s(:), azimuth_deg, elevation_deg, margin_deg);
end

function trajectory = linearTrajectory(initialState, goalState)
% Build one exact constant-velocity polynomial sampled at its endpoints.
duration_s = goalState.time_s - initialState.time_s;
positionPower_deg = zeros(1, 2, 6);
positionPower_deg(1, :, 1) = initialState.position_deg;
positionPower_deg(1, :, 2) = ...
    goalState.position_deg - initialState.position_deg;
velocityPower_deg_s = zeros(1, 2, 5);
velocityPower_deg_s(1, :, 1) = initialState.velocity_deg_s;
polynomial = struct( ...
    "SegmentCount", 1, ...
    "SegmentStartTime_s", initialState.time_s, ...
    "SegmentDuration_s", duration_s, ...
    "FinalTime_s", goalState.time_s, ...
    "positionPower_deg", positionPower_deg, ...
    "velocityPower_deg_s", velocityPower_deg_s, ...
    "accelerationPower_deg_s2", zeros(1, 2, 4), ...
    "jerkPower_deg_s3", zeros(1, 2, 3), ...
    "TerminalState", struct());
trajectory = struct( ...
    "time_s", [initialState.time_s; goalState.time_s], ...
    "position_deg", [initialState.position_deg; goalState.position_deg], ...
    "velocity_deg_s", ...
    [initialState.velocity_deg_s; goalState.velocity_deg_s], ...
    "acceleration_deg_s2", zeros(2, 2), ...
    "jerk_deg_s3", zeros(2, 2), "Polynomial", polynomial);
end
