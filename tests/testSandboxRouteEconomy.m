function tests = testSandboxRouteEconomy
%% Section 0: Header & Readme
% SYNTAX
%   tests = testSandboxRouteEconomy
%**************************************************************************
% PURPOSE
%   - Guard sandbox-style static and moving obstacle requests against
%     excessive joint travel and repeated lateral reversals.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function-test array)
%       Deterministic route-economy regression cases.
%**************************************************************************
% UNITS
%   - Position and accumulated two-axis travel are in degrees. Time is in
%     seconds, and derivative limits use degrees per second and its powers.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add the repository and trajectory engine used by the public planner.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
testCase.TestData.Request = createRequest();
end

function testStaticCircleHasOneEconomicalDetour(testCase)
% Compare joint travel with the exact tangent-and-arc geometric lower bound.
request = testCase.TestData.Request;
circleRadius_deg = 2;
safetyMargin_deg = 0.25;
angle_rad = linspace(0, 2 * pi, 49).';
angle_rad(end) = [];
circle_deg = circleRadius_deg * [cos(angle_rad), sin(angle_rad)];
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "static circle", [0; request.goalState.time_s], ...
    circle_deg(:, 1), circle_deg(:, 2), safetyMargin_deg);

result = runAndValidate(testCase, obstacles, request);
protectedRadius_deg = circleRadius_deg + safetyMargin_deg;
halfChord_deg = 0.5 * norm( ...
    request.goalState.position_deg - request.initialState.position_deg);
tangentLength_deg = sqrt(halfChord_deg^2 - protectedRadius_deg^2);
arcAngle_rad = 2 * asin(protectedRadius_deg / halfChord_deg);
geometricLowerBound_deg = ...
    2 * tangentLength_deg + protectedRadius_deg * arcAngle_rad;

verifyLessThanOrEqual(testCase, motionLength(result), ...
    1.01 * geometricLowerBound_deg, ...
    "The circle detour exceeds the tangent-and-arc lower bound by over 1%%.");
verifyLessThanOrEqual(testCase, lateralReversalCount(result), 1, ...
    "The circle detour repeatedly reverses its lateral joint motion.");
axisReports = result.SearchDiagnostics.FixedClockExcursion.AxisReports;
verifyGreaterThan(testCase, sum([axisReports.BoundaryRefinementCount]), 0, ...
    "The fixed-clock clearance boundary was not refined.");
end

function testIrregularStaticObstacleAvoidsRepeatedJointMotion(testCase)
% Exercise a concave asymmetric outline that is unlike the circular case.
request = testCase.TestData.Request;
star_deg = createIrregularStar();
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "irregular static obstacle", [0; request.goalState.time_s], ...
    star_deg(:, 1), star_deg(:, 2), 0.25);

result = runAndValidate(testCase, obstacles, request);
directDistance_deg = norm( ...
    request.goalState.position_deg - request.initialState.position_deg);
verifyLessThanOrEqual(testCase, motionLength(result), ...
    1.04 * directDistance_deg, ...
    "The irregular static detour adds over 4%% joint travel.");
verifyLessThanOrEqual(testCase, lateralReversalCount(result), 1, ...
    "The irregular static detour repeatedly reverses lateral motion.");
end

function testMovingIrregularObstacleAvoidsRepeatedJointMotion(testCase)
% Move the same concave outline across the route during the earliest motion.
request = testCase.TestData.Request;
start_deg = createIrregularStar();
finish_deg = start_deg + [0 3];
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "irregular moving obstacle", [0; 10; request.goalState.time_s], ...
    {start_deg(:, 1); finish_deg(:, 1); finish_deg(:, 1)}, ...
    {start_deg(:, 2); finish_deg(:, 2); finish_deg(:, 2)}, 0.25);

result = runAndValidate(testCase, obstacles, request);
directDistance_deg = norm( ...
    request.goalState.position_deg - request.initialState.position_deg);
verifyLessThanOrEqual(testCase, motionLength(result), ...
    1.02 * directDistance_deg, ...
    "The moving-obstacle detour adds over 2%% joint travel.");
verifyLessThanOrEqual(testCase, lateralReversalCount(result), 1, ...
    "The moving-obstacle detour repeatedly reverses lateral motion.");
end

function request = createRequest()
% Keep one deterministic sandbox-scale request shared by all obstacle shapes.
request = struct();
request.initialState = struct("time_s", 0, "position_deg", [-8 0]);
request.goalState = struct("time_s", 20, "position_deg", [8 0]);
request.limits = struct( ...
    "azimuthInterval_deg", [-10 10], ...
    "elevationInterval_deg", [-6 6], ...
    "maxVelocity_deg_s", [3 3], ...
    "maxAcceleration_deg_s2", [2 2], ...
    "maxJerk_deg_s3", [4 4]);
request.options = struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "MaximumSeedCount", 6, ...
    "MaximumTimeLayerCount", 17, ...
    "SampleTime_s", 0.05, ...
    "MaximumNlpIterations", 80);
end

function star_deg = createIrregularStar()
% Alternate radii and scale one axis to create a concave asymmetric outline.
angle_rad = (0:9).' * (2 * pi / 10) + pi / 2;
radius_deg = repmat([2.4; 1.1], 5, 1);
star_deg = [radius_deg .* cos(angle_rad), ...
    0.7 * radius_deg .* sin(angle_rad)];
end

function result = runAndValidate(testCase, obstacles, request)
% Require the public planner and independent validator to agree on success.
result = obstacleAvoidance.planTrajectory( ...
    obstacles, request.initialState, request.goalState, ...
    request.limits, request.options);
validation = obstacleAvoidance.validateTrajectory(result);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, validation.Passed, validation.Message);
verifyTrue(testCase, validation.CollisionFree);
verifyTrue(testCase, validation.VelocityWithinLimits);
verifyTrue(testCase, validation.AccelerationWithinLimits);
verifyTrue(testCase, validation.JerkWithinLimits);
end

function count = lateralReversalCount(result)
% Ignore sampled derivative roundoff before counting meaningful reversals.
lateralVelocity_deg_s = result.velocity_deg_s(:, 2);
velocityTolerance_deg_s = 1e-8 * max(1, ...
    max(abs(lateralVelocity_deg_s)));
significantSign = sign(lateralVelocity_deg_s( ...
    abs(lateralVelocity_deg_s) > velocityTolerance_deg_s));
count = sum(diff(significantSign) ~= 0);
end

function length_deg = motionLength(result)
% Measure the returned position history without requiring a duplicate field.
length_deg = obstacleAvoidance.geometry.routeLength(result.position_deg);
end
