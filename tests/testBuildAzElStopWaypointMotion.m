function tests = testBuildAzElStopWaypointMotion
%% Section 0: Header & Readme
% SYNTAX
%   tests = testBuildAzElStopWaypointMotion
%**************************************************************************
% PURPOSE
%   - Verify deterministic analytic motion on geometric polyline seeds.
%   - Verify certified limits, fixed time, and honest failure behavior.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% PURPOSE
%   - Add the repository root for direct focused-test execution.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testEarliestMotionStopsAtEveryWaypoint(testCase)
% PURPOSE
%   - Verify the earliest certified profile and exact corner stop.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(100, [3 4], [0 0], [0 0]);
limits = physicalLimits([2 2], [3 3], [10 10]);
options = plannerOptions("earliestArrival");
options.SampleTime_s = 0.037;
seed = geometricSeed([0 0; 3 0; 3 4]);
candidate = azElInternal.buildAzElStopWaypointMotion( ...
    [], initialState, goalState, limits, options, seed);
verifyTrue(testCase, candidate.Success, candidate.Message);
verifyTrue(testCase, candidate.ConstructionFeasible);
verifyFalse(testCase, candidate.OptimizerFeasible);
verifyTrue(testCase, candidate.Validation.Passed);
verifyEqual(testCase, candidate.TerminationReason, ...
    "seedMotionValidated");
verifyEqual(testCase, candidate.Polynomial.SegmentCount, 8);
edgeDuration_s = 4 * candidate.Polynomial.SegmentDuration_s(1:4:end);
verifyEqual(testCase, candidate.MotionDuration_s, sum(edgeDuration_s), ...
    "AbsTol", 1e-12);
waypointTime_s = initialState.time_s + edgeDuration_s(1);
[waypointError_s, waypointIndex] = min( ...
    abs(candidate.time_s - waypointTime_s));
verifyLessThanOrEqual(testCase, waypointError_s, 1e-12);
verifyEqual(testCase, candidate.position_deg(waypointIndex, :), ...
    [3 0], "AbsTol", 1e-11);
verifyEqual(testCase, candidate.velocity_deg_s(waypointIndex, :), ...
    [0 0], "AbsTol", 1e-11);
verifyEqual(testCase, ...
    candidate.acceleration_deg_s2(waypointIndex, :), ...
    [0 0], "AbsTol", 1e-10);
firstEdge = candidate.time_s <= waypointTime_s;
secondEdge = candidate.time_s >= waypointTime_s;
verifyEqual(testCase, candidate.position_deg(firstEdge, 2), ...
    zeros(nnz(firstEdge), 1), "AbsTol", 1e-11);
verifyEqual(testCase, candidate.position_deg(secondEdge, 1), ...
    3 * ones(nnz(secondEdge), 1), "AbsTol", 1e-11);
verifyLessThanOrEqual(testCase, ...
    candidate.AnalyticDiagnostics.CertifiedPeakVelocity_deg_s, ...
    limits.maxVelocity_deg_s * (1 + 1e-12));
verifyLessThanOrEqual(testCase, ...
    candidate.AnalyticDiagnostics.CertifiedPeakAcceleration_deg_s2, ...
    limits.maxAcceleration_deg_s2 * (1 + 1e-12));
verifyLessThanOrEqual(testCase, ...
    candidate.AnalyticDiagnostics.CertifiedPeakJerk_deg_s3, ...
    limits.maxJerk_deg_s3 * (1 + 1e-12));
end

function testFixedArrivalUsesTheCompleteTimeWindow(testCase)
% PURPOSE
%   - Verify deterministic time dilation to an exact fixed arrival.
initialState = state(2, [-2 1], [0 0], [0 0]);
goalState = state(22, [4 3], [0 0], [0 0]);
limits = physicalLimits([3 3], [2 2], [8 8]);
options = plannerOptions("fixedArrival");
seed = geometricSeed([-2 1; 1 1; 4 3]);
candidate = azElInternal.buildAzElStopWaypointMotion( ...
    [], initialState, goalState, limits, options, seed);
verifyTrue(testCase, candidate.Success, candidate.Message);
verifyEqual(testCase, candidate.FinalTime_s, 22, "AbsTol", 1e-12);
verifyEqual(testCase, candidate.MotionDuration_s, 20, ...
    "AbsTol", 1e-12);
verifyEqual(testCase, sum(candidate.Polynomial.SegmentDuration_s), ...
    20, "AbsTol", 1e-12);
verifyTrue(testCase, candidate.Validation.GoalTimeSatisfied);
end

function testAsymmetricLimitsCertifyADiagonalZigzag(testCase)
% PURPOSE
%   - Verify the analytic bounds on several non-axis-aligned seed edges.
waypoint_deg = [-5 -2; -1 4; 2 -3; 8 1];
initialState = state(0, waypoint_deg(1, :), [0 0], [0 0]);
goalState = state(1000, waypoint_deg(end, :), [0 0], [0 0]);
limits = physicalLimits([1.3 2.7], [0.8 1.4], [0.6 3.2]);
options = plannerOptions("earliestArrival");
seed = geometricSeed(waypoint_deg);
candidate = azElInternal.buildAzElStopWaypointMotion( ...
    [], initialState, goalState, limits, options, seed);
verifyTrue(testCase, candidate.Success, candidate.Message);
verifyTrue(testCase, candidate.Validation.VelocityWithinLimits);
verifyTrue(testCase, candidate.Validation.AccelerationWithinLimits);
verifyTrue(testCase, candidate.Validation.JerkWithinLimits);
edgeDuration_s = 4 * candidate.Polynomial.SegmentDuration_s(1:4:end);
for waypointIndex = 2:size(waypoint_deg, 1) - 1
    waypointTime_s = sum(edgeDuration_s(1:waypointIndex - 1));
    [timeError_s, sampleIndex] = min( ...
        abs(candidate.time_s - waypointTime_s));
    verifyLessThanOrEqual(testCase, timeError_s, 1e-11);
    verifyEqual(testCase, candidate.position_deg(sampleIndex, :), ...
        waypoint_deg(waypointIndex, :), "AbsTol", 1e-10);
    verifyEqual(testCase, candidate.velocity_deg_s(sampleIndex, :), ...
        [0 0], "AbsTol", 1e-10);
    verifyEqual(testCase, ...
        candidate.acceleration_deg_s2(sampleIndex, :), ...
        [0 0], "AbsTol", 1e-9);
end
end

function testShortFixedArrivalReturnsStableFailure(testCase)
% PURPOSE
%   - Verify that an insufficient fixed horizon returns no clipped motion.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(1, [4 0], [0 0], [0 0]);
limits = physicalLimits([1 1], [1 1], [1 1]);
options = plannerOptions("fixedArrival");
seed = geometricSeed([0 0; 4 0]);
candidate = azElInternal.buildAzElStopWaypointMotion( ...
    [], initialState, goalState, limits, options, seed);
verifyFalse(testCase, candidate.Success);
verifyFalse(testCase, candidate.ConstructionFeasible);
verifyEqual(testCase, candidate.TerminationReason, ...
    "fixedArrivalInfeasible");
verifyEmpty(testCase, candidate.time_s);
verifyEqual(testCase, candidate.Polynomial.positionPower_deg, ...
    zeros(0, 2, 6));
verifyGreaterThan(testCase, ...
    candidate.AnalyticDiagnostics.MinimumUniformEdgeDuration_s, 1);
verifyTrue(testCase, isfield(candidate, "Validation"));
verifyTrue(testCase, isfield(candidate, "SolverDiagnostics"));
end

function testNonzeroEndpointDerivativeIsExplicitlyUnsupported(testCase)
% PURPOSE
%   - Verify an unsupported boundary state does not enter construction.
initialState = state(0, [0 0], [0.1 0], [0 0]);
goalState = state(20, [4 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [2 2], [2 2]);
options = plannerOptions("earliestArrival");
seed = geometricSeed([0 0; 4 0]);
candidate = azElInternal.buildAzElStopWaypointMotion( ...
    [], initialState, goalState, limits, options, seed);
verifyFalse(testCase, candidate.Success);
verifyFalse(testCase, candidate.ConstructionFeasible);
verifyEqual(testCase, candidate.TerminationReason, ...
    "unsupportedEndpointDerivatives");
verifyEmpty(testCase, candidate.time_s);
end

function testEarliestMovingGoalIsExplicitlyUnsupported(testCase)
% PURPOSE
%   - Verify that an unresolved moving endpoint returns a stable failure.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(20, [5 0], [0 0], [0 0]);
goalState.targetTime_s = [0; 20];
goalState.targetPosition_deg = [4 0; 5 0];
goalState.InterpolationMethod = "linear";
limits = physicalLimits([2 2], [2 2], [2 2]);
options = plannerOptions("earliestArrival");
seed = geometricSeed([0 0; 5 0]);
candidate = azElInternal.buildAzElStopWaypointMotion( ...
    [], initialState, goalState, limits, options, seed);
verifyFalse(testCase, candidate.Success);
verifyFalse(testCase, candidate.ConstructionFeasible);
verifyEqual(testCase, candidate.TerminationReason, ...
    "unsupportedMovingGoal");
verifyEmpty(testCase, candidate.time_s);
end

function testTimedWaitUsesTheSeedDuration(testCase)
% PURPOSE
%   - Verify a stationary timed edge waits for a different moving barrier.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(20, [4 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [2 2], [4 4]);
options = plannerOptions("earliestArrival");
seed = geometricSeed([0 0; 0 0; 4 0]);
seed.Source = "directWait";
seed.tau = [0; 0.5; 1];
seed.EstimatedDuration_s = 10;
obstacle = makeAzElObstacleData( ...
    "temporary barrier", [0; 4], ...
    [1.5; 2.5; 2.5; 1.5], [-1; -1; 1; 1], 0);
candidate = azElInternal.buildAzElStopWaypointMotion( ...
    obstacle, initialState, goalState, limits, options, seed);
verifyTrue(testCase, candidate.Success, candidate.Message);
verifyEqual(testCase, candidate.MotionDuration_s, 10, "AbsTol", 1e-10);
beforeMotion = candidate.time_s <= 5;
verifyEqual(testCase, candidate.position_deg(beforeMotion, :), ...
    zeros(nnz(beforeMotion), 2), "AbsTol", 1e-11);
verifyTrue(testCase, candidate.Validation.CollisionFree);
end

function testEarlyFailureKeepsValidationFieldOrder(testCase)
% PURPOSE
%   - Verify early and validated candidates use one validation schema.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(20, [4 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [2 2], [2 2]);
options = plannerOptions("earliestArrival");
seed = geometricSeed([0 0; 4 0]);
validCandidate = azElInternal.buildAzElStopWaypointMotion( ...
    [], initialState, goalState, limits, options, seed);
initialState.velocity_deg_s = [0.1 0];
earlyFailure = azElInternal.buildAzElStopWaypointMotion( ...
    [], initialState, goalState, limits, options, seed);
verifyTrue(testCase, validCandidate.Validation.Passed);
verifyFalse(testCase, earlyFailure.Validation.Passed);
verifyEqual(testCase, fieldnames(earlyFailure.Validation), ...
    fieldnames(validCandidate.Validation));
end

function testCollisionFailureRetainsTheConstructedMotion(testCase)
% PURPOSE
%   - Verify that geometry collision cannot become analytic success.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(10, [4 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [2 2], [4 4]);
options = plannerOptions("fixedArrival");
seed = geometricSeed([0 0; 4 0]);
obstacle = makeAzElObstacleData( ...
    "blocking rectangle", [0; 10], ...
    [1.5; 2.5; 2.5; 1.5], [-1; -1; 1; 1], 0);
candidate = azElInternal.buildAzElStopWaypointMotion( ...
    obstacle, initialState, goalState, limits, options, seed);
verifyFalse(testCase, candidate.Success);
verifyTrue(testCase, candidate.ConstructionFeasible);
verifyFalse(testCase, candidate.Validation.CollisionFree);
verifyEqual(testCase, candidate.TerminationReason, ...
    "seedMotionCollision");
verifyNotEmpty(testCase, candidate.time_s);
verifyTrue(testCase, candidate.Validation.VelocityWithinLimits);
verifyTrue(testCase, candidate.Validation.AccelerationWithinLimits);
verifyTrue(testCase, candidate.Validation.JerkWithinLimits);
end

function options = plannerOptions(goalTimeMode)
% PURPOSE
%   - Return complete resolved-style options for focused helper tests.
options = planAzElMotion();
options.GoalTimeMode = goalTimeMode;
options.SampleTime_s = 0.05;
options.ConstraintTolerance = 1e-7;
end

function value = state(time_s, position_deg, velocity_deg_s, ...
        acceleration_deg_s2)
% PURPOSE
%   - Construct one normalized static endpoint state.
value = struct( ...
    "time_s", time_s, ...
    "position_deg", position_deg, ...
    "velocity_deg_s", velocity_deg_s, ...
    "acceleration_deg_s2", acceleration_deg_s2);
end

function limits = physicalLimits(velocity_deg_s, acceleration_deg_s2, ...
        jerk_deg_s3)
% PURPOSE
%   - Construct one normalized physical-limit record.
limits = struct( ...
    "maxVelocity_deg_s", velocity_deg_s, ...
    "maxAcceleration_deg_s2", acceleration_deg_s2, ...
    "maxJerk_deg_s3", jerk_deg_s3, ...
    "azimuthInterval_deg", [-180 180], ...
    "elevationInterval_deg", [-90 90]);
end

function seed = geometricSeed(position_deg)
% PURPOSE
%   - Construct one deterministic geometric seed with no corridor claim.
edgeLength_deg = vecnorm(diff(position_deg, 1, 1), 2, 2);
distance_deg = [0; cumsum(edgeLength_deg)];
seed = struct( ...
    "Index", 1, ...
    "Source", "focusedTest", ...
    "position_deg", position_deg, ...
    "tau", distance_deg / distance_deg(end), ...
    "CorridorBoundary_deg", zeros(0, 2), ...
    "EstimatedDuration_s", NaN, ...
    "Length_deg", distance_deg(end));
end
