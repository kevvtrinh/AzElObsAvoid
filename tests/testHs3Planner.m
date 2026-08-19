function tests = testHs3Planner
%% Section 0: Header & Readme
% SYNTAX
%   tests = testHs3Planner
%**************************************************************************
% PURPOSE
%   - Verify the compact HS3 planner, geometry interpolation, validation,
%     deterministic seed behavior, moving targets, and stable failures.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Test position is degrees, time is seconds, and derivatives use
%     deg/s, deg/s^2, and deg/s^3.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add the repository root for path-based test execution.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testDefaultsExposeOneSmallPlanner(testCase)
% Verify that zero-input defaults expose only the maintained HS3 controls.
options = planAzElMotion();
verifyEqual(testCase, options.MaximumSeedCount, 5);
verifyEqual(testCase, options.GoalTimeMode, "earliestArrival");
verifyFalse(testCase, isfield(options, "UseParallel"));
verifyFalse(testCase, isfield(options, "MaximumVisibilitySnapshotsPerObstacle"));
end

function testConstantJerkPolynomialPassesIndependentDynamics(testCase)
% Verify the third-order chain against analytic constant-jerk motion.
duration_s = 2;
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(duration_s, [4 / 3 0], [2 0], [2 0]);
limits = physicalLimits([3 3], [3 3], [2 2]);
trajectory = constantJerkTrajectory(duration_s);
options = fixedOptions();
validation = validateAzElTrajectory( ...
    trajectory, [], initialState, goalState, limits, options);
verifyTrue(testCase, validation.Passed, validation.Message);
verifyLessThanOrEqual(testCase, validation.MaximumDynamicsResidual, 1e-12);
end

function testFixedArrivalEnforcesTerminalState(testCase)
% Verify fixed time and nonzero initial and terminal state equalities.
initialState = state(0, [0 0], [0.1 0], [0.05 0]);
goalState = state(8, [4 2], [0.2 -0.1], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
result = planAzElMotion([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.velocity_deg_s(1, :), ...
    initialState.velocity_deg_s, "AbsTol", 1e-10);
verifyEqual(testCase, result.acceleration_deg_s2(1, :), ...
    initialState.acceleration_deg_s2, "AbsTol", 1e-10);
verifyEqual(testCase, result.time_s(end), 8, "AbsTol", 1e-7);
verifyEqual(testCase, result.velocity_deg_s(end, :), ...
    goalState.velocity_deg_s, "AbsTol", 1e-6);
end

function testEarliestArrivalIsInsideHorizon(testCase)
% Verify the two-stage earliest-arrival solve and independent validation.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(20, [4 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.GoalTimeMode = "earliestArrival";
result = planAzElMotion([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyLessThan(testCase, result.time_s(end), goalState.time_s);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
jerkRampTime_s = limits.maxAcceleration_deg_s2(1) / ...
    limits.maxJerk_deg_s3(1);
constantAccelerationTime_s = (-1.5 + sqrt(16.25)) / 2;
independentMinimumTime_s = 2 * ...
    (constantAccelerationTime_s + 2 * jerkRampTime_s);
verifyEqual(testCase, result.time_s(end), independentMinimumTime_s, ...
    "AbsTol", 0.25);
end

function testSelectedCandidateSupportsMeshRefinement(testCase)
% Verify one selected motion can be re-solved on a denser HS3 mesh.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(7, [3 1], [0 0], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.CollocationSegmentCount = 4;
options.MaximumCollocationSegmentCount = 8;
options.MaximumMeshRefinementPasses = 1;
result = planAzElMotion([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyEqual(testCase, result.Polynomial.SegmentCount, 8);
verifyEqual(testCase, ...
    result.SearchDiagnostics.MeshRefinementPassCount, 1);
end

function testStaticObstacleProducesOppositeSideSeeds(testCase)
% Verify bounded input-driven seed diversity and validated selection.
obstacle = rectangleObstacle([0 20], [-1 1 -2 2], 0.2);
initialState = state(0, [-5 0], [0 0], [0 0]);
goalState = state(12, [5 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumSeedCount = 3;
options.DirectSeedOnly = false;
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);
verifyGreaterThanOrEqual(testCase, numel(result.Seeds), 3);
verifyTrue(testCase, any([result.Seeds.Source] == "visibilityGraph"));
verifyEqual(testCase, ...
    result.SearchDiagnostics.Grid.GraphType, ...
    "timeExpandedVisibilityGraph");
verifyGreaterThan(testCase, ...
    result.SearchDiagnostics.Grid.VisibilityEdgeCount, 0);
verifyEqual(testCase, ...
    size(result.SearchDiagnostics.Grid.AcceptedEdges_deg, 2), 4);
verifyEqual(testCase, ...
    size(result.SearchDiagnostics.Grid.RejectedEdges_deg, 2), 4);
verifyTrue(testCase, all(isfinite( ...
    result.SearchDiagnostics.Grid.AcceptedEdges_deg), "all"));
minimumElevations_deg = zeros(numel(result.Seeds), 1);
maximumElevations_deg = zeros(numel(result.Seeds), 1);
for seedIndex = 1:numel(result.Seeds)
    minimumElevations_deg(seedIndex) = min( ...
        result.Seeds(seedIndex).position_deg(:, 2));
    maximumElevations_deg(seedIndex) = max( ...
        result.Seeds(seedIndex).position_deg(:, 2));
end
verifyLessThan(testCase, min(minimumElevations_deg), -2);
verifyGreaterThan(testCase, max(maximumElevations_deg), 2);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.CollisionFree);
validated = find([result.SeedSummaries.ValidationPassed]);
if result.Success
    arrival_s = [result.SeedSummaries(validated).ArrivalTime_s];
    verifyLessThanOrEqual(testCase, ...
        result.SeedSummaries(result.SelectedSeedIndex).ArrivalTime_s, ...
        min(arrival_s) + options.ArrivalTimeTolerance_s);
end
end

function testMovingObstacleUsesTrajectoryTime(testCase)
% Verify that the same point changes occupancy as protected geometry moves.
time_s = [0; 4];
source_deg = [-1 -1; 1 -1; 1 1; -1 1];
azimuth_deg = {source_deg(:, 1); source_deg(:, 1)};
elevation_deg = {source_deg(:, 2); source_deg(:, 2) + 4};
obstacle = makeAzElObstacleData( ...
    "moving", time_s, azimuth_deg, elevation_deg, 0);
occupied = queryAzElTimeObstacle( ...
    obstacle, [0; 0], [0; 0], [0; 4]);
verifyEqual(testCase, occupied, [true; false]);
end

function testDeformingObstacleInterpolatesAtTrajectoryTime(testCase)
% Verify vertex interpolation changes occupancy for a deforming polygon.
time_s = [0; 2];
first_deg = [-1 -1; 1 -1; 1 1; -1 1];
second_deg = [-2 -0.5; 2 -0.5; 2 0.5; -2 0.5];
obstacle = makeAzElObstacleData( ...
    "deforming", time_s, ...
    {first_deg(:, 1); second_deg(:, 1)}, ...
    {first_deg(:, 2); second_deg(:, 2)}, 0);
occupied = queryAzElTimeObstacle( ...
    obstacle, [1.5; 1.5], [0; 0], [0; 2]);
verifyEqual(testCase, occupied, [false; true]);
end

function testTopologyChangeUsesStaticConservativeUnion(testCase)
% Verify a topology-change interval has one finite static union bound.
first_deg = [-2 -1; 0 -1; 0 1; -2 1];
second_deg = [0 -1; 2 -1; 2 1; 0 1; NaN NaN; ...
    -2 -1; -1 -1; -1 1; -2 1];
obstacle = makeAzElObstacleData( ...
    "topology change", [0; 1], ...
    {first_deg(:, 1); second_deg(:, 1)}, ...
    {first_deg(:, 2); second_deg(:, 2)}, 0);
[shape, geometry] = azElInternal.obstacleShapeAtTime(obstacle, 0.5);
verifyFalse(testCase, geometry.TopologyIsInterpolated);
verifyEqual(testCase, geometry.VertexSpeedBound_deg_s, 0);
verifyTrue(testCase, isinterior(shape, -1.5, 0));
verifyTrue(testCase, isinterior(shape, 1.5, 0));
end

function testDeformingObstacleUsesThePlannerPath(testCase)
% Verify a deforming protected polygon uses the maintained HS3 planner.
time_s = [0; 8];
first_deg = [-1 4; 1 4; 1 6; -1 6];
second_deg = [-2 4.5; 2 4.5; 2 5.5; -2 5.5];
obstacle = makeAzElObstacleData( ...
    "deforming", time_s, ...
    {first_deg(:, 1); second_deg(:, 1)}, ...
    {first_deg(:, 2); second_deg(:, 2)}, 0.1);
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(8, [4 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, fixedOptions());
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyTrue(testCase, result.Validation.CollisionFree);
end

function testMovingBarrierGeneratesWaitingSeed(testCase)
% Verify that source-time motion creates one input-driven waiting seed.
time_s = [0; 6; 8; 12];
source_deg = [-0.5 -2; 0.5 -2; 0.5 2; -0.5 2];
centerElevation_deg = [0; 0; 6; 6];
azimuth_deg = cell(4, 1);
elevation_deg = cell(4, 1);
for sampleIndex = 1:4
    translated_deg = source_deg + [0 centerElevation_deg(sampleIndex)];
    azimuth_deg{sampleIndex} = translated_deg(:, 1);
    elevation_deg{sampleIndex} = translated_deg(:, 2);
end
obstacle = makeAzElObstacleData( ...
    "barrier", time_s, azimuth_deg, elevation_deg, 0);
initialState = state(0, [-5 0], [0 0], [0 0]);
goalState = state(12, [5 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
options = planAzElMotion();
options.MaximumSeedCount = 5;
[seeds, diagnostics] = azElInternal.generateAzElTopologySeeds( ...
    obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, any([seeds.Source] == "directWait"));
verifyGreaterThan(testCase, diagnostics.ExpandedCount, 0);
verifyEqual(testCase, diagnostics.GraphType, ...
    "timeExpandedVisibilityGraph");
verifyGreaterThan(testCase, diagnostics.TemporalLayerCount, 1);
verifyGreaterThan(testCase, diagnostics.WaitEdgeCount, 0);
end

function testWaitingSeedDoesNotImposeCornerState(testCase)
% Verify a repeated seed vertex does not become a zero-velocity equality.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(8, [4 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumSolverTime_s = options.MaximumPlanningTime_s;
seed = struct( ...
    "Index", 1, "Source", "directWait", ...
    "position_deg", [0 0; 0 0; 4 0], ...
    "tau", [0; 0.4; 1], "EstimatedDuration_s", 8, ...
    "Length_deg", 4);
candidate = azElInternal.solveAzElHs3( ...
    [], initialState, goalState, limits, options, seed);
cornerTime_s = 0.4 * goalState.time_s;
[~, sampleIndex] = min(abs(candidate.time_s - cornerTime_s));
verifyTrue(testCase, candidate.OptimizerFeasible, candidate.Message);
verifyGreaterThan(testCase, ...
    norm(candidate.velocity_deg_s(sampleIndex, :)), 0.1);
verifyGreaterThan(testCase, ...
    norm(candidate.position_deg(sampleIndex, :) - ...
    seed.position_deg(2, :)), 0.1);
end

function testAzimuthWrappingChangesThePhysicalRequest(testCase)
% Verify wrapping selects the short move and disabled wrapping keeps the long move.
initialState = state(0, [179 0], [0 0], [0 0]);
goalState = state(8, [-179 0], [0 0], [0 0]);
limits = physicalLimits([1 1], [1 1], [2 2]);
options = fixedOptions();
options.AllowAzimuthWrapping = false;
longResult = planAzElMotion([], initialState, goalState, limits, options);
options.AllowAzimuthWrapping = true;
shortResult = planAzElMotion([], initialState, goalState, limits, options);
verifyFalse(testCase, longResult.Success);
verifyTrue(testCase, shortResult.Success, shortResult.Message);
verifyEqual(testCase, shortResult.position_deg(end, 1), 181, ...
    "AbsTol", 1e-6);
verifyTrue(testCase, shortResult.Validation.AzimuthWrapPolicySatisfied);
end

function testPlanningTimeLimitReturnsStableFailure(testCase)
% Verify a time budget is an expected result with per-seed diagnostics.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(8, [4 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumPlanningTime_s = 1e-9;
result = planAzElMotion([], initialState, goalState, limits, options);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "planningTimeLimit");
verifyEqual(testCase, numel(result.SeedSummaries), numel(result.Seeds));
verifyEqual(testCase, ...
    result.SeedSummaries(1).TerminationReason, "planningTimeLimit");
end

function testBetweenNodeCollisionFailsValidation(testCase)
% Verify a collision at the segment midpoint cannot hide between samples.
initialState = state(0, [-1 0], [2 0], [0 0]);
goalState = state(1, [1 0], [2 0], [0 0]);
limits = physicalLimits([3 3], [1 1], [1 1]);
trajectory = linearTrajectory(initialState, goalState);
obstacle = rectangleObstacle([0 1], [-0.1 0.1 -1 1], 0);
validation = validateAzElTrajectory( ...
    trajectory, obstacle, initialState, goalState, limits, fixedOptions());
verifyFalse(testCase, validation.Passed);
verifyFalse(testCase, validation.CollisionFree);
verifyLessThanOrEqual(testCase, validation.MinimumClearance_deg, 0);
end

function testBetweenNodeVelocityViolationFailsValidation(testCase)
% Verify Bernstein limits detect an interior velocity peak with clear samples.
initialState = state(0, [0 0], [0 0], [4 0]);
goalState = state(1, [2 / 3 0], [0 0], [-4 0]);
limits = physicalLimits([0.8 1], [5 5], [9 9]);
trajectory = interiorVelocityPeakTrajectory();
validation = validateAzElTrajectory( ...
    trajectory, [], initialState, goalState, limits, fixedOptions());
verifyFalse(testCase, validation.Passed);
verifyFalse(testCase, validation.VelocityWithinLimits);
verifyEqual(testCase, max(abs(trajectory.velocity_deg_s), [], "all"), 0);
end

function testSafetyMarginIsAppliedExactlyOnce(testCase)
% Verify absolute reconstruction from original geometry is idempotent.
source_deg = [-1 -1; 1 -1; 1 1; -1 1];
obstacle = makeAzElObstacleData( ...
    "margin", [0; 1], source_deg(:, 1), source_deg(:, 2), 0.2);
reinflated = inflateAzElObstacleData(obstacle, 0.2);
verifyEqual(testCase, reinflated.az_deg, obstacle.az_deg, "AbsTol", 1e-12);
verifyEqual(testCase, reinflated.el_deg, obstacle.el_deg, "AbsTol", 1e-12);
verifyEqual(testCase, reinflated.safetyMargin_deg, 0.2);
end

function testTranslatedHistoryReusesExactProtectedShape(testCase)
% Verify rigid obstacle motion preserves one translated protected boundary.
source_deg = [-1 -1; 1 -1; 1 1; -1 1];
translation_deg = [3 2];
azimuth_deg = {source_deg(:, 1); ...
    source_deg(:, 1) + translation_deg(1)};
elevation_deg = {source_deg(:, 2); ...
    source_deg(:, 2) + translation_deg(2)};
obstacle = makeAzElObstacleData( ...
    "translated", [0; 1], azimuth_deg, elevation_deg, 0.2);
verifyEqual(testCase, obstacle.az_deg{2}, ...
    obstacle.az_deg{1} + translation_deg(1), "AbsTol", 1e-12);
verifyEqual(testCase, obstacle.el_deg{2}, ...
    obstacle.el_deg{1} + translation_deg(2), "AbsTol", 1e-12);
end

function testDeterministicRepeatedRun(testCase)
% Verify identical fixed inputs return identical seed order and trajectory.
initialState = state(0, [0 0], [0 0], [0 0]);
goalState = state(6, [3 1], [0 0], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
first = planAzElMotion([], initialState, goalState, limits, options);
second = planAzElMotion([], initialState, goalState, limits, options);
verifyEqual(testCase, first.Success, second.Success);
verifyEqual(testCase, [first.Seeds.Source], [second.Seeds.Source]);
verifyEqual(testCase, first.time_s, second.time_s, "AbsTol", 1e-12);
verifyEqual(testCase, first.position_deg, second.position_deg, ...
    "AbsTol", 1e-9);
end

function testNoPathReturnsStableDiagnostics(testCase)
% Verify expected no-path failure returns diagnostics instead of an error.
wall = rectangleObstacle([0 12], [-0.5 0.5 -90 90], 0);
initialState = state(0, [-5 0], [0 0], [0 0]);
goalState = state(12, [5 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumSeedCount = 3;
options.MaximumPlanningTime_s = 8;
options.MaximumNlpIterations = 40;
options.ElevationInterval_deg = [-10 10];
result = planAzElMotion(wall, initialState, goalState, limits, options);
verifyFalse(testCase, result.Success);
verifyTrue(testCase, any(result.TerminationReason == ...
    ["noValidatedSeed", "planningTimeLimit"]));
verifyEqual(testCase, numel(result.SeedSummaries), numel(result.Seeds));
verifyGreaterThanOrEqual(testCase, ...
    result.SearchDiagnostics.Grid.ExpandedCount, 0);
plotOptions = struct( ...
    "FigureVisible", "off", ...
    "ShowWorkspace", true, ...
    "ShowVisibilityGraphs", true, ...
    "ShowKinematics", false, ...
    "ShowAnimation", false);
handles = plotAzElMotion(result, plotOptions);
figureCleanup = onCleanup(@() closeTestFigures(handles));
verifyTrue(testCase, isgraphics(handles.WorkspaceFigure, "figure"));
verifyTrue(testCase, isgraphics(handles.VisibilityFigure, "figure"));
end

function testMovingTargetUsesSamePlanner(testCase)
% Verify the intercept wrapper only adapts target inputs to planAzElMotion.
targetTime_s = (0:5:20).';
targetPosition_deg = [6 + 0.1 * targetTime_s, ...
    ones(size(targetTime_s))];
targetMotion = struct( ...
    "time_s", targetTime_s, ...
    "position_deg", targetPosition_deg, ...
    "InterpolationMethod", "linear");
initialState = state(0, [0 0], [0 0], [0 0]);
limits = physicalLimits([2 2], [1 1], [2 2]);
plannerOptions = fixedOptions();
plannerOptions.GoalTimeMode = "earliestArrival";
interceptOptions = struct( ...
    "InterceptMode", "earliest", ...
    "MaximumSearchDuration_s", 20, ...
    "PlannerOptions", plannerOptions);
result = planAzElMovingTargetIntercept( ...
    initialState, targetMotion, limits, interceptOptions);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.Intercept.Mode, "earliest");
verifyEqual(testCase, result.position_deg(end, :), ...
    result.Intercept.TargetPosition_deg, "AbsTol", 1e-6);
end

function options = fixedOptions()
% Return fast deterministic settings for focused tests.
options = planAzElMotion();
options.GoalTimeMode = "fixedArrival";
options.DirectSeedOnly = true;
options.MaximumSeedCount = 1;
options.CollocationSegmentCount = 5;
options.MaximumNlpIterations = 150;
options.MaximumNlpFunctionEvaluations = 15000;
options.MaximumPlanningTime_s = 25;
options.SampleTime_s = 0.05;
options.Verbose = false;
end

function value = state(time_s, position_deg, velocity_deg_s, ...
        acceleration_deg_s2)
% Construct one complete endpoint state.
value = struct( ...
    "time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", velocity_deg_s, ...
    "acceleration_deg_s2", acceleration_deg_s2);
end

function limits = physicalLimits(velocity_deg_s, acceleration_deg_s2, ...
        jerk_deg_s3)
% Construct one complete physical-limit record.
limits = struct( ...
    "maxVelocity_deg_s", velocity_deg_s, ...
    "maxAcceleration_deg_s2", acceleration_deg_s2, ...
    "maxJerk_deg_s3", jerk_deg_s3);
end

function closeTestFigures(handles)
% Close figures created by a plot test even when its verification fails.
figureHandles = [handles.WorkspaceFigure; handles.VisibilityFigure];
close(figureHandles(isgraphics(figureHandles, "figure")));
end

function obstacle = rectangleObstacle(time_s, bounds_deg, margin_deg)
% Construct a static rectangle from [minAz maxAz minEl maxEl].
azimuth_deg = bounds_deg([1 2 2 1]).';
elevation_deg = bounds_deg([3 3 4 4]).';
obstacle = makeAzElObstacleData( ...
    "rectangle", time_s(:), azimuth_deg, elevation_deg, margin_deg);
end

function trajectory = constantJerkTrajectory(duration_s)
% Build analytic q=t^3/6, v=t^2/2, a=t, and j=1 on the first axis.
positionPower_deg = zeros(1, 2, 6);
velocityPower_deg_s = zeros(1, 2, 5);
accelerationPower_deg_s2 = zeros(1, 2, 4);
jerkPower_deg_s3 = zeros(1, 2, 3);
positionPower_deg(1, 1, 4) = duration_s^3 / 6;
velocityPower_deg_s(1, 1, 3) = duration_s^2 / 2;
accelerationPower_deg_s2(1, 1, 2) = duration_s;
jerkPower_deg_s3(1, 1, 1) = 1;
time_s = [0; duration_s];
position_deg = [0 0; duration_s^3 / 6 0];
velocity_deg_s = [0 0; duration_s^2 / 2 0];
acceleration_deg_s2 = [0 0; duration_s 0];
jerk_deg_s3 = [1 0; 1 0];
trajectory = trajectoryRecord( ...
    time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
    jerk_deg_s3, positionPower_deg, velocityPower_deg_s, ...
    accelerationPower_deg_s2, jerkPower_deg_s3, duration_s);
end

function trajectory = linearTrajectory(initialState, goalState)
% Build one exact constant-velocity segment sampled only at its endpoints.
duration_s = goalState.time_s - initialState.time_s;
positionPower_deg = zeros(1, 2, 6);
positionPower_deg(1, :, 1) = initialState.position_deg;
positionPower_deg(1, :, 2) = ...
    goalState.position_deg - initialState.position_deg;
velocityPower_deg_s = zeros(1, 2, 5);
velocityPower_deg_s(1, :, 1) = initialState.velocity_deg_s;
accelerationPower_deg_s2 = zeros(1, 2, 4);
jerkPower_deg_s3 = zeros(1, 2, 3);
trajectory = trajectoryRecord( ...
    [initialState.time_s; goalState.time_s], ...
    [initialState.position_deg; goalState.position_deg], ...
    [initialState.velocity_deg_s; goalState.velocity_deg_s], ...
    zeros(2, 2), zeros(2, 2), positionPower_deg, ...
    velocityPower_deg_s, accelerationPower_deg_s2, ...
    jerkPower_deg_s3, duration_s);
end

function trajectory = interiorVelocityPeakTrajectory()
% Build v=4s(1-s), which peaks between clear endpoint samples.
duration_s = 1;
positionPower_deg = zeros(1, 2, 6);
positionPower_deg(1, 1, 3) = 2;
positionPower_deg(1, 1, 4) = -4 / 3;
velocityPower_deg_s = zeros(1, 2, 5);
velocityPower_deg_s(1, 1, 2) = 4;
velocityPower_deg_s(1, 1, 3) = -4;
accelerationPower_deg_s2 = zeros(1, 2, 4);
accelerationPower_deg_s2(1, 1, 1) = 4;
accelerationPower_deg_s2(1, 1, 2) = -8;
jerkPower_deg_s3 = zeros(1, 2, 3);
jerkPower_deg_s3(1, 1, 1) = -8;
trajectory = trajectoryRecord( ...
    [0; 1], [0 0; 2 / 3 0], zeros(2, 2), ...
    [4 0; -4 0], [-8 0; -8 0], positionPower_deg, ...
    velocityPower_deg_s, accelerationPower_deg_s2, ...
    jerkPower_deg_s3, duration_s);
end

function trajectory = trajectoryRecord(time_s, position_deg, ...
        velocity_deg_s, acceleration_deg_s2, jerk_deg_s3, ...
        positionPower_deg, velocityPower_deg_s, ...
        accelerationPower_deg_s2, jerkPower_deg_s3, duration_s)
% Assemble one manual trajectory with the planner polynomial schema.
polynomial = struct( ...
    "SegmentCount", 1, "SegmentStartTime_s", time_s(1), ...
    "SegmentDuration_s", duration_s, "FinalTime_s", time_s(end), ...
    "positionPower_deg", positionPower_deg, ...
    "velocityPower_deg_s", velocityPower_deg_s, ...
    "accelerationPower_deg_s2", accelerationPower_deg_s2, ...
    "jerkPower_deg_s3", jerkPower_deg_s3, ...
    "TerminalState", struct());
trajectory = struct( ...
    "time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", velocity_deg_s, ...
    "acceleration_deg_s2", acceleration_deg_s2, ...
    "jerk_deg_s3", jerk_deg_s3, "Polynomial", polynomial);
end
