function tests = testCircleDetour
%% Section 0: Header & Readme
% SYNTAX
%   tests = testCircleDetour
% PURPOSE
%   Prevent excessive fixed-clock detours after an early obstacle crossing.
% INPUTS
%   None. Replay the supplied circle bundle without changing its request.
% OUTPUTS
%   Deterministic MATLAB function tests and independent validation assertions.
% UNITS
%   Degrees, seconds, and their motion derivatives.

%% Section 1: Register Tests
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    repositoryRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(repositoryRoot, fullfile(repositoryRoot, 'trajectory'));
    testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testExactSavedCircleAvoidsLateOvershoot(testCase)
    % The stored request is feasible; require an actual validated improvement.
    loaded = load(fullfile(testCase.TestData.RepositoryRoot, 'Rogue Examples', 'inefficientroutecircle.mat'));
    bundle = loaded.diagnosisBundle;
    inputs = bundle.PlannerInputs;
    result = obstacleAvoidance.planTrajectory(inputs.obstacles, inputs.initialState, inputs.goalState, inputs.limits, bundle.PlannerOptions);
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    baselineLength_deg = obstacleAvoidance.geometry.routeLength(bundle.Result.position_deg);
    % Require the path-first improvement, including removal of the jerk-cost veto.
    verifyLessThan(testCase, obstacleAvoidance.geometry.routeLength(result.position_deg), 0.92 * baselineLength_deg);
    verifyEqual(testCase, result.TrajectoryDuration_s, bundle.Result.TrajectoryDuration_s, 'AbsTol', 1e-9);
end

function testEarlyRectangleWithElevationGoverningClock(testCase)
    % Swap the governing axis, change the outline, and shift the absolute clock.
    initial = restState(7, [3 -30]);
    goal    = restState(107, [-2 70]);
    limits  = struct();
    limits.maxVelocity_deg_s      = [2 2];
    limits.maxAcceleration_deg_s2 = [0.75 0.75];
    limits.maxJerk_deg_s3         = [2.5 2.5];
    limits.azimuthInterval_deg    = [-40 40];
    limits.elevationInterval_deg  = [-40 80];
    vertices = [-12 -13; 10 -13; 10 -4; -12 -4];
    obstacle = obstacleAvoidance.obstacles.createObstacle('early rectangle', [7; 107], vertices(:,1), vertices(:,2), 0.2);
    options  = obstacleAvoidance.input.resolvePlannerOptions(struct('GoalTimeMode', 'earliestArrival'));
    result   = obstacleAvoidance.planTrajectory(obstacle, initial, goal, limits, options);
    validation = obstacleAvoidance.validateTrajectory(result);
    direct = bmtpEngine.createDirectMotion(initial, goal, limits, options);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyEqual(testCase, result.TrajectoryDuration_s, direct.TrajectoryDuration_s, 'AbsTol', 1e-9);
    verifyLessThan(testCase, obstacleAvoidance.geometry.routeLength(result.position_deg), 109);
end

function testPrescribedTurnPreservesMotionAndDefaultCompatibility(testCase)
    % Check the complete turn, including between-piece continuity and limits.
    initial = restState(7, [0 0]);
    goal = restState(47, [20 2]);
    limits = struct();
    limits.maxVelocity_deg_s      = [2 2];
    limits.maxAcceleration_deg_s2 = [0.75 0.75];
    limits.maxJerk_deg_s3         = [2.5 2.5];
    limits.azimuthInterval_deg    = [-5 25];
    limits.elevationInterval_deg  = [-10 10];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct('GoalTimeMode', 'fixedArrival'));
    direct = bmtpEngine.createDirectMotion(initial, goal, limits, options);
    knots_s = [7; 20; 47];
    offsets_deg = [0; 3; 0];
    [~, ~, baseVelocity_deg_s] = bmtpEngine.evaluatePolynomial(direct.Polynomial, knots_s(2));
    candidate = bmtpEngine.createOffsetSplineMotion(direct, knots_s, offsets_deg, 2, initial, options.SampleTime_s, 'turnRegression', [NaN; -baseVelocity_deg_s(2); NaN]);
    [~, ~, velocity_deg_s] = bmtpEngine.evaluatePolynomial(candidate.Polynomial, knots_s(2));
    validation = obstacleAvoidance.validateTrajectory(candidate, [], initial, goal, limits, options);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyEqual(testCase, velocity_deg_s(2), 0, 'AbsTol', 1e-12);
    ordinary = bmtpEngine.createOffsetSplineMotion(direct, knots_s, offsets_deg, 2, initial, options.SampleTime_s, 'turnRegression');
    emptyOverride = bmtpEngine.createOffsetSplineMotion(direct, knots_s, offsets_deg, 2, initial, options.SampleTime_s, 'turnRegression', []);
    verifyEqual(testCase, ordinary.Polynomial, emptyOverride.Polynomial);
end

function value = restState(time_s, position_deg)
    value = struct();
    value.time_s = time_s;
    value.position_deg = position_deg;
    value.velocity_deg_s = [0 0];
    value.acceleration_deg_s2 = [0 0];
end
