function tests = testPlannerStageTiming
%% Section 0: Header & Readme
% Verify exclusive stage timing for both independent planner methods.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add production and maintained example entry points.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "examples"));
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testCorridorSuccessAndEndpointFailureShareTimingSchema(testCase)
% Require one finite additive public schema on normal and early exits.
initialState = state(0, [0 0]);
goalState = state(8, [4 0]);
limits = physicalLimits();
options = fixedOptions();
success = planAzElMotion([], initialState, goalState, limits, options);
blockingObstacle = rectangleObstacle([0 8], [-1 1 -1 1], 0);
failure = planAzElMotion( ...
    blockingObstacle, initialState, goalState, limits, options);

verifyTrue(testCase, success.Success, success.Message);
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

function testCorridorNoPathRetainsAdditiveTiming(testCase)
% Exercise a failure after topology and candidate evaluation have run.
wall = rectangleObstacle([0 12], [-0.5 0.5 -90 90], 0.2);
initialState = state(0, [-5 0]);
goalState = state(12, [5 0]);
limits = physicalLimits();
limits.elevationInterval_deg = [-10 10];
options = fixedOptions();
options.MaximumSeedCount = 3;
result = planAzElMotion(wall, initialState, goalState, limits, options);

verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
verifyStageTiming(testCase, result);
verifyGreaterThan(testCase, ...
    result.SearchDiagnostics.StageTiming.TopologyElapsedTime_s, 0);
end

function testDirectStaticRouteBypassesZeroVariableAffineModel(testCase)
% Keep a clear two-point route valid when unrelated static geometry exists.
initialState = state(0, [0 0]);
goalState = state(8, [4 0]);
limits = physicalLimits();
obstacle = rectangleObstacle([0 8], [10 11 10 11], 0.2);
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, fixedOptions());

verifyTrue(testCase, result.Success, result.Message);
verifyEqual(testCase, ...
    result.SeedSummaries(1).SolverDiagnostics.DecisionCount, 0);
verifyGreaterThan(testCase, ...
    result.SeedSummaries(1).SolverDiagnostics.ActualCorridorRecordCount, 0);
verifyStageTiming(testCase, result);
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
validation = validateAzElTrajectory( ...
    trajectory, obstacle, initialState, goalState, limits, fixedOptions());

verifyFalse(testCase, validation.Passed);
verifyGreaterThanOrEqual(testCase, ...
    validation.CollisionCheckingElapsedTime_s, 0);
verifyLessThanOrEqual(testCase, ...
    validation.CollisionCheckingElapsedTime_s, ...
    validation.ElapsedTime_s + timingTolerance(validation.ElapsedTime_s));
end

function testCompactC3ReportsAllAttemptedWork(testCase)
% Verify the bounded compact controller exposes reusable-basis evidence.
overrides = struct( ...
    "PlannerMethod", "corridorQuintic", ...
    "PlotOutputs", false, "FigureVisible", "off", ...
    "ShowWorkspace", false, "ShowKinematics", false, ...
    "ShowAnimation", false, "ShowSearchEdges", false, ...
    "ShowVisibilityGraphs", false, "ShowSweptSurfaces", false, ...
    "Verbose", false);
result = exampleUShapedAzElTimeSpace(overrides);
diagnostics = result.SearchDiagnostics.CompactC3;

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, diagnostics.Attempted);
verifyEqual(testCase, diagnostics.AffineBasisBuildCount, 1);
verifyEqual(testCase, ...
    diagnostics.AffineBasisReuseCount, diagnostics.TrialCount);
verifyStageTiming(testCase, result);
end

function testStandaloneHs3SuccessAndEndpointFailureShareTiming(testCase)
% Require standalone HS3 timing on a solved request and critical early exit.
initialState = state(0, [0 0]);
goalState = state(4, [1 0]);
limits = physicalLimits();
options = fixedHs3Options();
success = planAzElMotion([], initialState, goalState, limits, options);
blockingObstacle = rectangleObstacle([0 4], [-1 1 -1 1], 0);
failure = planAzElMotion( ...
    blockingObstacle, initialState, goalState, limits, options);

verifyTrue(testCase, success.Success, success.Message);
verifyTrue(testCase, success.Validation.Passed, success.Validation.Message);
verifyEqual(testCase, success.SelectedMotionSource, "hs3");
verifyFalse(testCase, failure.Success);
verifyEqual(testCase, failure.TerminationReason, "endpointBlocked");
verifyFalse(testCase, isfield(success, "CompositionDiagnostics"));
verifyFalse(testCase, isfield(failure, "CompositionDiagnostics"));
verifyStageTiming(testCase, success);
verifyStageTiming(testCase, failure);
verifyEqual(testCase, ...
    failure.SearchDiagnostics.StageTiming.TopologyElapsedTime_s, 0);
end

function testHs3SolverWorkReconcilesStandaloneTiming(testCase)
% Keep actual HS3 solver and validation work visible in exclusive stages.
initialState = state(0, [0 0]);
goalState = state(3, [1 0]);
limits = physicalLimits();
options = fixedHs3Options();
options.CollocationSegmentCount = 2;
options.MaximumCollocationSegmentCount = 2;
result = planAzElMotion([], initialState, goalState, limits, options);

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.SelectedMotionSource, "hs3");
verifyTrue(testCase, any([result.SeedSummaries.Hs3Attempted]));
verifyGreaterThan(testCase, result.SearchDiagnostics.Hs3ElapsedTime_s, 0);
verifyGreaterThan(testCase, ...
    result.SearchDiagnostics.StageTiming.MotionSolvingElapsedTime_s, 0);
verifyFalse(testCase, isfield(result, "CompositionDiagnostics"));
verifyStageTiming(testCase, result);
end

function testFinalizerRejectsOverAttribution(testCase)
% Reject overlapping stage ownership instead of hiding it in a zero residual.
timing = azElPlannerMethods.internal.stageTiming();
timing.MotionSolvingElapsedTime_s = 2;

verifyError(testCase, @() ...
    azElPlannerMethods.internal.stageTiming(timing, 1), ...
    "stageTiming:OverAttributed");
end

function verifyStageTiming(testCase, result)
% Verify exact shared field ownership and top-level reconciliation.
requiredNames = [ ...
    "TopologyElapsedTime_s"; "CorridorConstructionElapsedTime_s"; ...
    "MotionSolvingElapsedTime_s"; ...
    "CollisionCheckingElapsedTime_s"; ...
    "FinalValidationElapsedTime_s"; ...
    "UnattributedElapsedTime_s"; "TotalElapsedTime_s"];
timing = result.SearchDiagnostics.StageTiming;
verifyEqual(testCase, string(fieldnames(timing)), requiredNames);
verifyAdditiveTiming(testCase, timing);
verifyEqual(testCase, timing.TotalElapsedTime_s, ...
    result.ElapsedPlanningTime_s, ...
    "AbsTol", timingTolerance(timing.TotalElapsedTime_s));
end

function verifyAdditiveTiming(testCase, timing)
% Verify finite nonnegative exclusive fields, residual, and total.
values = struct2array(timing);

for value = values
    verifyTrue(testCase, ...
        isnumeric(value) && isreal(value) && isscalar(value));
    verifyTrue(testCase, isfinite(value));
    verifyGreaterThanOrEqual(testCase, value, 0);
end
exclusiveElapsedTime_s = sum(values(1:end - 2));
unattributedElapsedTime_s = values(end - 1);
totalElapsedTime_s = values(end);
tolerance_s = timingTolerance(totalElapsedTime_s);
verifyLessThanOrEqual(testCase, ...
    exclusiveElapsedTime_s, totalElapsedTime_s + tolerance_s);
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

function options = fixedOptions()
% Return deterministic corridor controls for focused timing tests.
options = planAzElMotion("corridorQuintic");
options.GoalTimeMode = "fixedArrival";
options.DirectSeedOnly = true;
options.MaximumSeedCount = 1;
options.SampleTime_s = 0.05;
options.Verbose = false;
end

function options = fixedHs3Options()
% Return deterministic standalone HS3 controls for timing tests.
options = planAzElMotion("hs3");
options.GoalTimeMode = "fixedArrival";
options.DirectSeedOnly = true;
options.MaximumSeedCount = 1;
options.CollocationSegmentCount = 3;
options.MaximumCollocationSegmentCount = 3;
options.MaximumMeshRefinementPasses = 0;
options.MaximumPlanningTime_s = 6;
options.SampleTime_s = 0.05;
options.Verbose = false;
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
obstacle = makeAzElObstacleData( ...
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
