function tests = testTimedBmtpPlanning
%% Section 0: Header & Readme
% SYNTAX
%   tests = testTimedBmtpPlanning
%**************************************************************************
% PURPOSE
%   - Verify smooth timed multi-waypoint planning around a moving circle and
%     a static concave obstacle without forced interior rest states.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function-test array)
%       Deterministic public-planner dynamic-topology regression cases.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds. Derivatives use units/s,
%     units/s^2, and units/s^3.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Run the motivating general input family once for all assertions.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
    [obstacles, initialState, goalState, limits, options] = createScenario();
    [result, resultDiagnosis]                             = planner(obstacles, initialState, goalState, limits, options);
    testCase.TestData.Result          = result;
    testCase.TestData.ResultDiagnosis = resultDiagnosis;
    testCase.TestData.Validation      = obstacleAvoidance.validateTrajectory(result);
    % The public portfolio can accept a static projection before timed BMTP.
    % Exercise the timed owner explicitly so its coverage contract remains tested.
    timedSeedIndex = find(string({resultDiagnosis.Routes.Source}) == "timeExpandedVisibilityGraph", 1, "first");
    assertNotEmpty(testCase, timedSeedIndex);
    inputs = result.Inputs;
    [timedCandidate, timedValidation, timedDiagnostics] = obstacleAvoidance.planner.solveTimedBmtpTrajectory(resultDiagnosis.Routes(timedSeedIndex), obstacleAvoidance.obstacles.prepareObstacles(inputs.obstacles), inputs.initialState, inputs.goalState, inputs.limits, result.Options, obstacleAvoidance.planner.createStageTiming());
    testCase.TestData.TimedCandidate   = timedCandidate;
    testCase.TestData.TimedValidation  = timedValidation;
    testCase.TestData.TimedDiagnostics = timedDiagnostics;
end

function testMovingCircleAndStaticUSucceeds(testCase)
    % Require full dynamic collision and kinematic validation.
    result          = testCase.TestData.Result;
    resultDiagnosis = testCase.TestData.ResultDiagnosis;
    validation      = testCase.TestData.Validation;
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyTrue(testCase, validation.CollisionFree);
    verifyTrue(testCase, validation.VelocityWithinLimits);
    verifyTrue(testCase, validation.AccelerationWithinLimits);
    verifyTrue(testCase, validation.JerkWithinLimits);
    verifyEqual(testCase, resultDiagnosis.Routes(resultDiagnosis.SelectedAttemptIndex).Source, "timeExpandedVisibilityGraph");
end

function testInteriorWaypointsAreNotForcedToRest(testCase)
    % Check the smooth polynomial state at every interior timed-seed knot.
    result          = testCase.TestData.Result;
    resultDiagnosis = testCase.TestData.ResultDiagnosis;
    seed            = resultDiagnosis.Routes(resultDiagnosis.SelectedAttemptIndex);
    interiorTime_s  = result.time_s(1) + seed.tau(2:end - 1) * (result.time_s(end) - result.time_s(1));
    [~, ~, velocity_units_s] = bmtpEngine.evaluatePolynomial(result.Polynomial, interiorTime_s);
    verifyGreaterThan(testCase, min(vecnorm(velocity_units_s, 2, 2)), 1e-3);
    verifyTrue(testCase, testCase.TestData.TimedCandidate.Success, testCase.TestData.TimedCandidate.Message);
    verifyTrue(testCase, testCase.TestData.TimedValidation.Passed, testCase.TestData.TimedValidation.Message);
    diagnostics = testCase.TestData.TimedDiagnostics;
    verifyEqual(testCase, diagnostics.Identifier, "bmtpTimedCell");
    completedTrials    = diagnostics.TimeCellTrials([diagnostics.TimeCellTrials.ElapsedTime_s] > 0);
    acceptedTrialIndex = find([completedTrials.ValidationPassed], 1, "first");
    verifyNotEmpty(testCase, acceptedTrialIndex);
    verifyTrue(testCase, completedTrials(acceptedTrialIndex).Success);
    verifyNotEmpty(testCase, completedTrials(acceptedTrialIndex).ValidationMessage);
end

function testTimedCellsUseFullSearchLayerBudget(testCase)
    % Use one full-resolution clock instead of a coarse/fine solve portfolio.
    result                   = testCase.TestData.Result;
    resultDiagnosis          = testCase.TestData.ResultDiagnosis;
    diagnostics              = testCase.TestData.TimedDiagnostics;
    maximumTimedSegmentCount = result.Options.MaximumTimeLayerCount - 1;
    verifyFalse(testCase, isfield(diagnostics, "SegmentCountFallbackAttempted"));
    verifyEqual(testCase, diagnostics.TimedSegmentCounts, maximumTimedSegmentCount);
    verifyEqual(testCase, diagnostics.Coverage.TimedSegmentCount, maximumTimedSegmentCount);
    verifyLessThanOrEqual(testCase, diagnostics.Coverage.TimedSegmentCount, maximumTimedSegmentCount);
end

function testRejectedCertificateExplainsAdaptiveValidation(testCase)
    % A corrupt timed certificate must not silently pass or lose its rejection reason.
    candidate = testCase.TestData.TimedCandidate;
    candidate.PlaneCertificate.Coverage.BaseTimeCellCount = 0;
    result     = testCase.TestData.Result;
    inputs     = result.Inputs;
    validation = obstacleAvoidance.validateTrajectory(candidate, inputs.obstacles, inputs.initialState, inputs.goalState, inputs.limits, result.Options);
    verifyFalse(testCase, validation.PlaneCertificateCertified);
    verifyEqual(testCase, validation.CertificateRejectionReason, "baseTimeCellCount");
    verifyGreaterThan(testCase, validation.CollisionCheckCount, 0);
end

function [obstacles, initialState, goalState, limits, options] = createScenario()
    % Create input-driven static-concave and translating-convex geometry.
    missionEndTime_s = 40;
    obstacleTime_s   = [0; missionEndTime_s];
    uPosition_units    = [ ...
        -8 7; -5 7; -5 -4; 5 -4; 5 7; 8 7; 8 -7; -8 -7];
    staticObstacle = obstacleAvoidance.obstacles.createObstacle("static concave polygon", obstacleTime_s, uPosition_units(:, 1), uPosition_units(:, 2), 0.20);
    angle_rad      = linspace(0, 2 * pi, 33).';
    angle_rad(end) = [];
    startCircle_units  = [-10 + 2 * cos(angle_rad), -7 + 2 * sin(angle_rad)];
    finishCircle_units = [10 + 2 * cos(angle_rad), -7 + 2 * sin(angle_rad)];
    movingObstacle   = obstacleAvoidance.obstacles.createObstacle("translating convex polygon", [0; 10; missionEndTime_s], {startCircle_units(:, 1); startCircle_units(:, 1); finishCircle_units(:, 1)}, {startCircle_units(:, 2); startCircle_units(:, 2); finishCircle_units(:, 2)}, 0.10);
    obstacles        = obstacleAvoidance.obstacles.combineObstacles(staticObstacle, movingObstacle);
    initialState     = struct();
    initialState.time_s       = 0;
    initialState.position_units = [0 0];
    goalState = struct("time_s", missionEndTime_s, "position_units", [0 -10]);
    limits    = struct();
    limits.xInterval_units    = [-14 14];
    limits.yInterval_units  = [-12 10];
    limits.maxVelocity_units_s      = [3 3];
    limits.maxAcceleration_units_s2 = [1.5 1.5];
    limits.maxJerk_units_s3         = [3 3];
    options = struct();
    options.GoalTimeMode                   = "earliestArrival";
    options.MaximumSeedCount               = 2;
    options.MaximumTimeLayerCount          = 17;
    options.SampleTime_s                   = 0.05;
    options.UnsupportedTimedTopologyPolicy = "fail";
end
