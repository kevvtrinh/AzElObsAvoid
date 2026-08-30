function tests = testPlannerContract
%% Section 0: Header & Readme
% SYNTAX
%   tests = testPlannerContract
%**************************************************************************
% PURPOSE
%   - Verify the maintained public planner contract without legacy-engine
%     names, private entry points, or implementation-specific solver counts.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%       Deterministic public success, failure, validation, and diagnostic tests.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add the public planner and independent BMTP engine to the MATLAB path.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
end

function testDefaultsExposeStablePlannerChoices(testCase)
% Keep the zero-input interface as the single defaults source.
options = obstacleAvoidance.planTrajectory();
requiredFields = {'GoalTimeMode', 'SampleTime_s', 'MaximumSeedCount', ...
    'MaximumWaitRefinementIterations', ...
    'CollisionClearanceTolerance_deg', 'AllowAzimuthWrapping', 'Verbose'};
verifyTrue(testCase, isstruct(options) && isscalar(options));
verifyTrue(testCase, all(isfield(options, requiredFields)));
verifyEqual(testCase, options.GoalTimeMode, "earliestArrival");
verifyEqual(testCase, options.MaximumWaitRefinementIterations, 16);
end

function testObstacleFreeEarliestMotionPassesPublicValidation(testCase)
% Require a finite rest-to-rest direct motion inside the supplied horizon.
initialState = restState(0, [0 0]);
goalState = restState(20, [4 2]);
result = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, physicalLimits(), plannerOptions("earliestArrival"));

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyTrue(testCase, result.Validation.CollisionFree);
verifyTrue(testCase, result.Validation.VelocityWithinLimits);
verifyTrue(testCase, result.Validation.AccelerationWithinLimits);
verifyTrue(testCase, result.Validation.JerkWithinLimits);
verifyEqual(testCase, result.TerminationReason, "goalReached");
verifyEqual(testCase, result.SeedSummaries(1).SeedSource, ...
    "directRestToRest");
verifyLessThan(testCase, result.TrajectoryDuration_s, ...
    goalState.time_s - initialState.time_s);
end

function testSuccessAndEndpointFailureShareResultShape(testCase)
% Preserve every public field on expected failure as well as success.
initialState = restState(0, [-2 0]);
goalState = restState(10, [2 0]);
limits = physicalLimits();
options = plannerOptions("earliestArrival");
success = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, limits, options);
blocking = obstacleAvoidance.obstacles.createObstacle( ...
    "blocked start", [0; 10], [-3 -1 -1 -3], [-1 -1 1 1], 0);
failure = obstacleAvoidance.planTrajectory( ...
    blocking, initialState, goalState, limits, options);

verifyTrue(testCase, success.Success);
verifyFalse(testCase, failure.Success);
verifyEqual(testCase, failure.TerminationReason, "endpointBlocked");
verifyEqual(testCase, fieldnames(failure), fieldnames(success));
verifyEqual(testCase, fieldnames(failure.Validation), ...
    fieldnames(success.Validation));
verifyEqual(testCase, fieldnames(failure.PlaneCertificate), ...
    fieldnames(success.PlaneCertificate));
end

function testStaticDetourReturnsCertifiedCollisionFreeMotion(testCase)
% Exercise topology generation, BMTP, and independent static-plane replay.
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "center box", [0; 30], [-1 1 1 -1], [-1 -1 1 1], 0.2);
initialState = restState(0, [-4 0]);
goalState = restState(30, [4 0]);
result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, physicalLimits(), ...
    plannerOptions("earliestArrival"));

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyTrue(testCase, result.Validation.CollisionFree);
verifyTrue(testCase, result.Validation.CollisionResolved);
verifyGreaterThan(testCase, result.Validation.MinimumClearance_deg, 0);
verifyGreaterThan(testCase, ...
    sum(vecnorm(diff(result.position_deg, 1, 1), 2, 2)), 8);
end

function testReflectedProgressAxisUsesValidatedPolynomial(testCase)
% Exercise negative first-axis progress through two floating convex barriers.
obstacleTime_s = [0; 30];
firstObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "first floating barrier", obstacleTime_s, ...
    [4.4 5.6 5.6 4.4], [-0.45 -0.45 0.45 0.45], 0.1);
secondObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "second floating barrier", obstacleTime_s, ...
    [-5.6 -4.4 -4.4 -5.6], [-0.45 -0.45 0.45 0.45], 0.1);
obstacles = obstacleAvoidance.obstacles.combineObstacles( ...
    firstObstacle, secondObstacle);
initialState = restState(0, [10 0]);
goalState = restState(30, [-10 0]);
limits = physicalLimits();
limits.azimuthInterval_deg = [-12 12];
limits.elevationInterval_deg = [-4 4];
result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, ...
    plannerOptions("earliestArrival"));

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyTrue(testCase, result.Validation.CollisionFree);
verifyTrue(testCase, result.Validation.CollisionResolved);
diagnostics = result.SearchDiagnostics.FixedClockExcursion;
verifyTrue(testCase, diagnostics.Success, diagnostics.Message);
verifyEqual(testCase, diagnostics.SelectedMode, ...
    "alternatingProgressPolynomial");
verifyEqual(testCase, ...
    diagnostics.ProgressPolynomial.ProgressAxisIndex, 1);
verifyEqual(testCase, ...
    diagnostics.ProgressPolynomial.LateralAxisIndex, 2);
verifyEqual(testCase, result.TrajectoryDuration_s, 12.5, ...
    "AbsTol", 1e-9);
verifyEqual(testCase, result.SelectedSeed_deg, result.position_deg);
verifyEqual(testCase, result.Seeds(1).Length_deg, ...
    sum(vecnorm(diff(result.position_deg), 2, 2)), "AbsTol", 1e-12);
end

function testFixedArrivalBelowPhysicalMinimumReturnsFailure(testCase)
% An infeasible clock is an expected result rather than a thrown error.
initialState = restState(0, [0 0]);
goalState = restState(0.25, [1 0]);
limits = physicalLimits();
result = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, limits, plannerOptions("fixedArrival"));

verifyFalse(testCase, result.Success);
verifyNotEmpty(testCase, result.Message);
verifyTrue(testCase, any(result.TerminationReason == ...
    ["noValidatedSeed", "fixedArrivalInfeasible", "timeWindowInfeasible"]));
verifyEmpty(testCase, result.time_s);
verifyTrue(testCase, isfield(result.SearchDiagnostics, "StageTiming"));
end

function testUnsupportedEndpointDerivativesThrowNamedError(testCase)
% The compact BMTP scope rejects non-rest endpoints explicitly.
initialState = restState(0, [0 0]);
initialState.velocity_deg_s = [0.1 0];
goalState = restState(10, [2 0]);
request = @() obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, physicalLimits(), ...
    plannerOptions("earliestArrival"));
verifyError(testCase, request, "bmtpEngine:UnsupportedRequest");
end

function testNoPathReturnsRecognizedDiagnostics(testCase)
% A full-height wall must fail with retained search evidence.
wall = obstacleAvoidance.obstacles.createObstacle( ...
    "full-height wall", [0; 20], [-0.5 0.5 0.5 -0.5], ...
    [-90 -90 90 90], 0);
initialState = restState(0, [-5 0]);
goalState = restState(12, [5 0]);
limits = physicalLimits();
limits.elevationInterval_deg = [-10 10];
options = plannerOptions("earliestArrival");
options.MaximumSeedCount = 3;
result = obstacleAvoidance.planTrajectory( ...
    wall, initialState, goalState, limits, options);

verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
verifyEmpty(testCase, result.time_s);
verifyTrue(testCase, isfield(result.SearchDiagnostics.Grid, ...
    "ExpandedCount"));
verifyGreaterThanOrEqual(testCase, ...
    result.SearchDiagnostics.AttemptedSeedCount, 1);
end

function testRepeatedDirectRequestIsDeterministic(testCase)
% Identical input must reproduce every selected motion sample exactly.
initialState = restState(0, [-1 0.5]);
goalState = restState(12, [3 -1]);
options = plannerOptions("earliestArrival");
first = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, physicalLimits(), options);
second = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, physicalLimits(), options);

verifyEqual(testCase, second.TerminationReason, first.TerminationReason);
verifyEqual(testCase, second.time_s, first.time_s);
verifyEqual(testCase, second.position_deg, first.position_deg);
verifyEqual(testCase, second.velocity_deg_s, first.velocity_deg_s);
verifyEqual(testCase, second.acceleration_deg_s2, ...
    first.acceleration_deg_s2);
verifyEqual(testCase, second.jerk_deg_s3, first.jerk_deg_s3);
end

function testTimedOpeningSeedSnapshotMatchesSelection(testCase)
% Keep an appended timed-opening winner addressable through returned seeds.
[obstacle, initialState, goalState, limits, options] = timedOpeningFixture();
result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyLessThanOrEqual(testCase, ...
    result.SelectedSeedIndex, numel(result.Seeds));
verifyEqual(testCase, ...
    result.Seeds(result.SelectedSeedIndex).Source, ...
    "timedOrthogonalOpening");
verifyEqual(testCase, ...
    result.Seeds(result.SelectedSeedIndex).position_deg, ...
    result.SelectedSeed_deg);
grid = result.SearchDiagnostics.Grid;
verifyTrue(testCase, grid.TemporalLayerLimitApplied);
verifyGreaterThan(testCase, ...
    grid.TemporalCandidateLayerCount, grid.TemporalLayerCount);
verifyEqual(testCase, grid.Coverage.CompletenessLossReason, ...
    "boundedSeedNodeAndTimeSearch");
end

function testInterceptValidatesInitialStateBeforeFieldAccess(testCase)
% Use the shared identified state error for malformed public input.
targetMotion = struct("time_s", [0; 1], ...
    "position_deg", [0 0; 1 0]);
verifyError(testCase, @() obstacleAvoidance.planMovingTargetIntercept( ...
    struct(), targetMotion, physicalLimits(), struct()), ...
    "planTrajectory:InvalidState");
end

function state = restState(time_s, position_deg)
% Create one two-axis rest endpoint.
state = struct("time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
end

function [obstacle, initialState, goalState, limits, options] = ...
        timedOpeningFixture()
% Create a timed opening whose validated candidate wins after seed append.
closedBoundary_deg = [ ...
    -8 7; -5 7; -5 -4; 5 -4; 5 7; 8 7; 8 -7; -8 -7];
leftOpenBoundary_deg = [ ...
    -8 7; -5 7; -5 -4; -1.5 -4; -1.5 -7; -8 -7];
rightOpenBoundary_deg = [ ...
    5 7; 8 7; 8 -7; 1.5 -7; 1.5 -4; 5 -4];
openBoundary_deg = [leftOpenBoundary_deg; NaN NaN; rightOpenBoundary_deg];
openingTime_s = 7.5;
obstacleTime_s = [0; openingTime_s - 1e-3; ...
    openingTime_s + 1e-3; 120];
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "timed opening", obstacleTime_s, ...
    {closedBoundary_deg(:, 1); closedBoundary_deg(:, 1); ...
    openBoundary_deg(:, 1); openBoundary_deg(:, 1)}, ...
    {closedBoundary_deg(:, 2); closedBoundary_deg(:, 2); ...
    openBoundary_deg(:, 2); openBoundary_deg(:, 2)}, 0.2);
initialState = restState(0, [0 0]);
goalState = restState(120, [0 -10]);
limits = physicalLimits();
limits.maxVelocity_deg_s = [2 2];
limits.maxAcceleration_deg_s2 = [0.75 0.75];
limits.maxJerk_deg_s3 = [2.5 2.5];
options = plannerOptions("earliestArrival");
options.MaximumSeedCount = 5;
options.MaximumTimeLayerCount = 2;
end

function limits = physicalLimits()
% Create neutral two-axis bounds used by the public-contract cases.
limits = struct("maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2], ...
    "azimuthInterval_deg", [-180 180], ...
    "elevationInterval_deg", [-90 90]);
end

function options = plannerOptions(goalTimeMode)
% Resolve only behavior-level public choices for deterministic tests.
options = struct("GoalTimeMode", goalTimeMode, ...
    "Verbose", false);
end
