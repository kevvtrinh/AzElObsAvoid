function tests = testPlannerScenarios
%% Section 0: Header & Readme
% SYNTAX
%   tests = testPlannerScenarios
%**************************************************************************
% PURPOSE
%   - Define deterministic physical acceptance cases for the greenfield
%     planner and its independent validator.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function-based test suite)
%**************************************************************************
% UNITS
%   - Not applicable.

tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   setupOnce(testCase)
%**************************************************************************
% PURPOSE
%   - Add repository and example folders for scenario tests.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Not applicable.

repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "examples"));
testCase.TestData.repositoryRoot = repositoryRoot;
end

function testUnobstructedCommandIsValidAndDeterministic(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testUnobstructedCommandIsValidAndDeterministic(testCase)
%**************************************************************************
% PURPOSE
%   - Verify exact endpoint states, continuous limits, and deterministic
%     command output for an unobstructed mission.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

request = baseRequest([0, 0], [20, 10]);
firstResult = planAzElAvoidance(request);
secondResult = planAzElAvoidance(request);

verifyTrue(testCase, firstResult.success);
verifyTrue(testCase, firstResult.validation.isValid);
verifyEqual(testCase, firstResult.command.time_s, ...
    secondResult.command.time_s, "AbsTol", 1e-12);
verifyEqual(testCase, firstResult.command.unwrappedPosition_deg, ...
    secondResult.command.unwrappedPosition_deg, "AbsTol", 1e-12);
verifyEqual(testCase, firstResult.command.velocity_deg_s, ...
    secondResult.command.velocity_deg_s, "AbsTol", 1e-12);
verifyEqual(testCase, firstResult.command.acceleration_deg_s2, ...
    secondResult.command.acceleration_deg_s2, "AbsTol", 1e-12);
verifyEqual(testCase, firstResult.guarantee.optimality, ...
    "Minimum arrival under a stated model");
end

function testNonzeroBoundaryVelocityAndAccelerationArePreserved(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testNonzeroBoundaryVelocityAndAccelerationArePreserved(testCase)
%**************************************************************************
% PURPOSE
%   - Verify complete nonzero boundary states are honored exactly.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Velocity is deg/s and acceleration is deg/s^2.

request = baseRequest([-10, -5], [18, 12]);
request.initialState.velocity_deg_s = [1.0, 0.4];
request.initialState.acceleration_deg_s2 = [0.2, -0.1];
request.goal.velocity_deg_s = [0.5, -0.25];
request.goal.acceleration_deg_s2 = [-0.15, 0.1];
result = planAzElAvoidance(request);

verifyTrue(testCase, result.success, result.message);
verifyTrue(testCase, result.validation.initialStateIsValid);
verifyTrue(testCase, result.validation.terminalStateIsValid);
verifyEqual(testCase, result.command.velocity_deg_s(1, :), ...
    request.initialState.velocity_deg_s, "AbsTol", 1e-12);
verifyEqual(testCase, result.command.acceleration_deg_s2(end, :), ...
    request.goal.acceleration_deg_s2, "AbsTol", 1e-12);
end

function testStaticDetourCarriesVelocityAndClearsMargin(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testStaticDetourCarriesVelocityAndClearsMargin(testCase)
%**************************************************************************
% PURPOSE
%   - Verify a blocking static polygon produces a smooth safe detour with
%     positive velocity at an interior turn.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Clearance is degrees and speed is deg/s.

request = baseRequest([-20, 0], [20, 0]);
request.obstacles = makeAzElObstacleData( ...
    "central exclusion", [0; 30], [-5; 5; 5; -5], ...
    [-5; -5; 5; 5]);
request.limits = struct( ...
    "azimuth_deg", [-60, 60], ...
    "elevation_deg", [-30, 30], ...
    "maxVelocity_deg_s", [12, 12], ...
    "maxAcceleration_deg_s2", [8, 8]);
request.options = struct("safetyMargin_deg", 1, "deadline_s", 30);
result = planAzElAvoidance(request);

verifyTrue(testCase, result.success, result.message);
verifyTrue(testCase, result.validation.isValid);
verifyGreaterThan(testCase, ...
    result.validation.collision.minimumClearance_deg, 1);
verifyGreaterThan(testCase, ...
    result.validation.motion.minimumInteriorSpeed_deg_s, 0);
verifyGreaterThan(testCase, size(result.command.time_s, 1), 2);
end

function testMovingWallProducesExplicitFeasibleWait(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testMovingWallProducesExplicitFeasibleWait(testCase)
%**************************************************************************
% PURPOSE
%   - Verify a delayed opening is handled by a real stationary interval.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Wait duration is seconds and clearance is degrees.

wallAzimuth_deg = [-2; 2; 2; -2];
blockingElevation_deg = [-30; -30; 30; 30];
clearElevation_deg = [35; 35; 55; 55];
request = baseRequest([-20, 0], [20, 0]);
request.obstacles = makeAzElObstacleData( ...
    "delayed opening", [0; 8; 8.2], ...
    {wallAzimuth_deg; wallAzimuth_deg; wallAzimuth_deg}, ...
    {blockingElevation_deg; blockingElevation_deg; clearElevation_deg});
request.limits = struct( ...
    "azimuth_deg", [-60, 60], ...
    "elevation_deg", [-20, 20], ...
    "maxVelocity_deg_s", [12, 12], ...
    "maxAcceleration_deg_s2", [8, 8]);
request.options = struct("safetyMargin_deg", 1, "deadline_s", 20);
result = planAzElAvoidance(request);

verifyTrue(testCase, result.success, result.message);
verifyTrue(testCase, result.validation.isValid);
verifyGreaterThan(testCase, result.validation.totalWaitDuration_s, 0);
verifyGreaterThan(testCase, ...
    result.validation.collision.minimumClearance_deg, 1);
verifyEqual(testCase, result.command.unwrappedPosition_deg(1, :), ...
    result.command.unwrappedPosition_deg(2, :), "AbsTol", 1e-12);
end

function testWrappedAzimuthRemainsContinuousAcrossSeam(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testWrappedAzimuthRemainsContinuousAcrossSeam(testCase)
%**************************************************************************
% PURPOSE
%   - Protect continuous unwrapped derivatives across a display seam.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Azimuth is degrees.

request = baseRequest([170, 5], [-170, 5]);
request.limits = struct( ...
    "azimuth_deg", [-180, 180], ...
    "elevation_deg", [-20, 20], ...
    "maxVelocity_deg_s", [10, 10], ...
    "maxAcceleration_deg_s2", [5, 5]);
request.options = struct("azimuthWrap", true, "deadline_s", 15);
result = planAzElAvoidance(request);

verifyTrue(testCase, result.success, result.message);
verifyTrue(testCase, result.validation.seamIsValid);
verifyEqual(testCase, result.command.unwrappedPosition_deg(end, 1), ...
    190, "AbsTol", 1e-12);
verifyEqual(testCase, result.command.position_deg(end, 1), ...
    -170, "AbsTol", 1e-12);
end

function testMovingGoalCaptureAndTrailingAreCoherent(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testMovingGoalCaptureAndTrailingAreCoherent(testCase)
%**************************************************************************
% PURPOSE
%   - Verify moving complete-state capture and post-arrival tracking.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Time is seconds and state fields carry angular units.

request = baseRequest([0, 0], [20, 0]);
request.goal = struct( ...
    "type", "moving", ...
    "time_s", [0; 15], ...
    "position_deg", [20, 0; 30, 0], ...
    "velocity_deg_s", [2 / 3, 0; 2 / 3, 0], ...
    "acceleration_deg_s2", [0, 0; 0, 0]);
request.limits = struct( ...
    "azimuth_deg", [-60, 60], ...
    "elevation_deg", [-20, 20], ...
    "maxVelocity_deg_s", [12, 12], ...
    "maxAcceleration_deg_s2", [8, 8]);
request.options = struct("deadline_s", 14, "trailingDuration_s", 1);
result = planAzElAvoidance(request);

verifyTrue(testCase, result.success, result.message);
verifyTrue(testCase, result.validation.isValid);
verifyEqual(testCase, result.command.time_s(end), ...
    result.arrivalTime_s + 1, "AbsTol", 1e-10);
verifyTrue(testCase, result.validation.terminalStateIsValid);
end

function testCanonicalTimeInterpolationDetectsInteriorCollision(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testCanonicalTimeInterpolationDetectsInteriorCollision(testCase)
%**************************************************************************
% PURPOSE
%   - Verify linearly moving canonical polygons are checked between samples,
%     including when a second obstacle has an independent time grid.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

movingObstacle = makeAzElObstacleData( ...
    "moving square", [0; 10], ...
    {[-1; 1; 1; -1]; [9; 11; 11; 9]}, ...
    {[-1; -1; 1; 1]; [-1; -1; 1; 1]});
disconnectedAzimuth_deg = [-40; -35; -35; -40; NaN; 35; 40; 40; 35];
disconnectedElevation_deg = ...
    [-8; -8; -4; -4; NaN; 4; 4; 8; 8];
independentObstacle = makeAzElObstacleData( ...
    "independent regions", [0; 4; 10], ...
    disconnectedAzimuth_deg, disconnectedElevation_deg);
request = baseRequest([-5, 0], [15, 0]);
request.obstacles = {movingObstacle, independentObstacle};
request.initialState.velocity_deg_s = [2, 0];
request.goal.velocity_deg_s = [2, 0];
request.limits.maxVelocity_deg_s = [5, 5];
request.limits.maxAcceleration_deg_s2 = [5, 5];
request.options = struct("deadline_s", 10, "safetyMargin_deg", 0.1);
command = struct( ...
    "interpolation", "quinticHermite", ...
    "time_s", [0; 10], ...
    "position_deg", [-5, 0; 15, 0], ...
    "unwrappedPosition_deg", [-5, 0; 15, 0], ...
    "velocity_deg_s", [2, 0; 2, 0], ...
    "acceleration_deg_s2", [0, 0; 0, 0], ...
    "azimuthWrap", false, ...
    "azimuthDisplayRange_deg", [-180, 180]);
validation = validateAzElCommand(command, request, 10);

verifyFalse(testCase, validation.collision.collisionFree);
verifyTrue(testCase, validation.collision.resolved);
verifyLessThan(testCase, validation.collision.minimumClearance_deg, 0.1);
end

function testHeadlessVisualizationUsesValidatedResult(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testHeadlessVisualizationUsesValidatedResult(testCase)
%**************************************************************************
% PURPOSE
%   - Exercise visualization headlessly after independent validation.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Not applicable.

request = baseRequest([0, 0], [8, 4]);
result = planAzElAvoidance(request);
originalVisibility = get(groot, "defaultFigureVisible");
cleanup = onCleanup(@() set(groot, ...
    "defaultFigureVisible", originalVisibility));
set(groot, "defaultFigureVisible", "off");
figureHandle = visualizeAzElPlan(request, result);
figureCleanup = onCleanup(@() close(figureHandle));

verifyTrue(testCase, isgraphics(figureHandle, "figure"));
verifyNotEmpty(testCase, findall(figureHandle, "Type", "axes"));
clear cleanup;
end

function testTemporalPaddingExpandsMovingOccupancy(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testTemporalPaddingExpandsMovingOccupancy(testCase)
%**************************************************************************
% PURPOSE
%   - Verify temporal uncertainty unions nearby moving-obstacle geometry.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Time is seconds and clearance is degrees.

movingObstacle = makeAzElObstacleData( ...
    "moving uncertainty", [0; 10], ...
    {[-1; 1; 1; -1]; [9; 11; 11; 9]}, ...
    {[-1; -1; 1; 1]; [-1; -1; 1; 1]});
request = baseRequest([7, 0], [7, 0]);
request.obstacles = movingObstacle;
request.goal.time_s = 4;
request.options = struct("deadline_s", 4, "safetyMargin_deg", 0.1);
command = struct( ...
    "interpolation", "quinticHermite", ...
    "time_s", [0; 4], ...
    "position_deg", [7, 0; 7, 0], ...
    "unwrappedPosition_deg", [7, 0; 7, 0], ...
    "velocity_deg_s", [0, 0; 0, 0], ...
    "acceleration_deg_s2", [0, 0; 0, 0], ...
    "azimuthWrap", false, ...
    "azimuthDisplayRange_deg", [-180, 180]);
unpadded = validateAzElCommand(command, request, 4);
request.options.temporalPadding_s = 3;
padded = validateAzElCommand(command, request, 4);

verifyTrue(testCase, unpadded.collision.collisionFree);
verifyFalse(testCase, padded.collision.collisionFree);
verifyTrue(testCase, padded.collision.resolved);
end

function testSeededObstacleFreeHoldoutsRemainValidated(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testSeededObstacleFreeHoldoutsRemainValidated(testCase)
%**************************************************************************
% PURPOSE
%   - Preserve a seeded draw order and validate every generated boundary
%     state instead of discarding difficult cases.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Positions are degrees.

rng(724, "twister");
caseCount = 12;
startPosition_deg = -30 + 60 .* rand(caseCount, 2);
goalPosition_deg = -30 + 60 .* rand(caseCount, 2);
for caseIndex = 1:caseCount
    request = baseRequest(startPosition_deg(caseIndex, :), ...
        goalPosition_deg(caseIndex, :));
    result = planAzElAvoidance(request);
    verifyTrue(testCase, result.success, sprintf( ...
        "Seed 724 case %d failed: %s", caseIndex, result.message));
    verifyTrue(testCase, result.validation.isValid);
end
end

function request = baseRequest(startPosition_deg, goalPosition_deg)
%% Section 0: Header & Readme
% SYNTAX
%   request = baseRequest(startPosition_deg, goalPosition_deg)
%**************************************************************************
% PURPOSE
%   - Build a deterministic complete-state mission for scenario tests.
%**************************************************************************
% INPUTS
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric)
%**************************************************************************
% OUTPUTS
%   - request (scalar planning request)
%**************************************************************************
% UNITS
%   - Positions are degrees and time is seconds.

request = struct();
request.obstacles = [];
request.initialState = struct( ...
    "time_s", 0, ...
    "position_deg", startPosition_deg, ...
    "velocity_deg_s", [0, 0], ...
    "acceleration_deg_s2", [0, 0]);
request.goal = struct( ...
    "type", "fixed", ...
    "time_s", NaN, ...
    "position_deg", goalPosition_deg, ...
    "velocity_deg_s", [0, 0], ...
    "acceleration_deg_s2", [0, 0]);
request.limits = struct( ...
    "azimuth_deg", [-180, 180], ...
    "elevation_deg", [-90, 90], ...
    "maxVelocity_deg_s", [10, 10], ...
    "maxAcceleration_deg_s2", [5, 5]);
request.options = struct();
end
