function tests = testCorridorPlannerDynamicTiming
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
projectRoot = fileparts(fileparts(mfilename("fullpath")));
testCase.TestData.OriginalPath = path;
addpath(projectRoot);
addpath(fullfile(projectRoot, "examples"));
end

function teardownOnce(testCase)
path(testCase.TestData.OriginalPath);
end

function testLateOpeningUsesInputDrivenTimingBracket(testCase)
[obstacles, initialState, goalState, limits] = lateOpeningRequest();
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, plannerOptions(5));

verifyExperimentalSuccess(testCase, result);
selected = result.SeedSummaries(result.SelectedSeedIndex).SolverDiagnostics;
verifyTrue(testCase, selected.HoldRecoveryUsed);
verifyGreaterThan(testCase, selected.HoldMultiplier, 2);
end

function testTranslatingCurvedObstacleRetainsValidatedTiming(testCase)
[obstacles, initialState, goalState, limits] = translatingCircleRequest();
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, plannerOptions(3));

verifyExperimentalSuccess(testCase, result);
end

function testStaticWorkspaceWallRemainsExpectedFailure(testCase)
time_s = [0; 20];
boundary_deg = [-0.4 -4; 0.4 -4; 0.4 4; -0.4 4];
obstacles = makeAzElObstacleData( ...
    "workspace-spanning wall", time_s, ...
    {boundary_deg(:, 1); boundary_deg(:, 1)}, ...
    {boundary_deg(:, 2); boundary_deg(:, 2)}, 0.1);
initialState = restState(0, [-3 0]);
goalState = restState(20, [3 0]);
limits = motionLimits([-3.5 3.5], [-3.5 3.5]);

result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, plannerOptions(3));

verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
verifyEqual(testCase, result.SearchDiagnostics.PlannerMethod, ...
    "corridorQuintic");
end

function testSmallStaticCorridorUsesValidatedTimingController(testCase)
% Verify bounded exact exchange or its shorter validated C3 replacement.
result = exampleUShapedAzElTimeSpace(struct("PlotOutputs", false));

verifyExperimentalSuccess(testCase, result);
diagnostics = result.SeedSummaries( ...
    result.SelectedSeedIndex).SolverDiagnostics;
if isfield(diagnostics, "CompactC3")
    verifyTrue(testCase, diagnostics.CompactC3.Accepted);
    verifyGreaterThan(testCase, diagnostics.CompactC3.QpCount, 0);
else
    verifyTrue(testCase, diagnostics.ExactTraversalAttempted);
    verifyTrue(testCase, diagnostics.ExactTraversalAccepted);
    verifyGreaterThan(testCase, diagnostics.ExactTraversalLinearSolveCount, 0);
end
verifyLessThanOrEqual(testCase, result.TrajectoryDuration_s, ...
    22.818548735851);
end

function [obstacles, initialState, goalState, limits] = lateOpeningRequest()
openingTime_s = 12;
transitionHalfWidth_s = 1e-3;
closedBoundary_deg = [ ...
    -9 8; -6 8; -6 -5; 6 -5; 6 8; 9 8; 9 -8; -9 -8];
leftOpen_deg = [-9 8; -6 8; -6 -5; -2 -5; -2 -8; -9 -8];
rightOpen_deg = [6 8; 9 8; 9 -8; 2 -8; 2 -5; 6 -5];
openBoundary_deg = [leftOpen_deg; NaN NaN; rightOpen_deg];
time_s = [0; openingTime_s - transitionHalfWidth_s; ...
    openingTime_s + transitionHalfWidth_s; 100];
obstacles = makeAzElObstacleData( ...
    "late-opening concavity", time_s, ...
    {closedBoundary_deg(:, 1); closedBoundary_deg(:, 1); ...
    openBoundary_deg(:, 1); openBoundary_deg(:, 1)}, ...
    {closedBoundary_deg(:, 2); closedBoundary_deg(:, 2); ...
    openBoundary_deg(:, 2); openBoundary_deg(:, 2)}, 0.2);
initialState = restState(0, [0 1]);
goalState = restState(100, [0 -11]);
limits = motionLimits([-12 12], [-12 12]);
end

function [obstacles, initialState, goalState, limits] = translatingCircleRequest()
time_s = [0; 16];
angle_rad = (0:31).' * (2 * pi / 32);
radius_deg = 1.35;
centerElevation_deg = [-0.5; 3.2];
azimuth_deg = cell(2, 1);
elevation_deg = cell(2, 1);
for index = 1:2
    azimuth_deg{index} = radius_deg * cos(angle_rad);
    elevation_deg{index} = centerElevation_deg(index) + ...
        radius_deg * sin(angle_rad);
end
obstacles = makeAzElObstacleData( ...
    "translating curved body", time_s, azimuth_deg, elevation_deg, 0.1);
initialState = restState(0, [-6.5 -0.5]);
goalState = restState(16, [6.5 -0.5]);
limits = motionLimits([-8 8], [-5 6]);
end

function options = plannerOptions(maximumSeedCount)
options = struct( ...
    "MotionMethod", "corridorQuintic", ...
    "GoalTimeMode", "earliestArrival", ...
    "MaximumSeedCount", maximumSeedCount, ...
    "SampleTime_s", 0.05, ...
    "RandomSeed", 325, ...
    "Verbose", false);
end

function state = restState(time_s, position_deg)
state = struct( ...
    "time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
end

function limits = motionLimits(azimuthInterval_deg, elevationInterval_deg)
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.8 0.8], ...
    "maxJerk_deg_s3", [2.5 2.5], ...
    "azimuthInterval_deg", azimuthInterval_deg, ...
    "elevationInterval_deg", elevationInterval_deg);
end

function verifyExperimentalSuccess(testCase, result)
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.SelectedMotionSource, "corridorQuintic");
verifyEqual(testCase, result.SearchDiagnostics.PlannerMethod, ...
    "corridorQuintic");
end
