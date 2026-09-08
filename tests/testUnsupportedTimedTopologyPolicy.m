function tests = testUnsupportedTimedTopologyPolicy
%% Section 0: Header & Readme
% SYNTAX
%   tests = testUnsupportedTimedTopologyPolicy
%**************************************************************************
% PURPOSE
%   - Verify that unsupported smooth timed routes fail by default and attempt
%     stop-at-waypoint Ruckig only under an explicit public policy.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (MATLAB function-test array)
%       Focused policy and diagnostic regression cases.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Run the identical deterministic request under both public policy values.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot);
    addpath(fullfile(repositoryRoot, "trajectory"));
    [obstacles, initialState, goalState, limits] = createScenario();
    baseOptions = struct();
    baseOptions.GoalTimeMode     = "fixedArrival";
    baseOptions.MaximumSeedCount = 3;
    baseOptions.SampleTime_s     = 0.05;
    failOptions = baseOptions;
    failOptions.UnsupportedTimedTopologyPolicy = "fail";
    fallbackOptions = baseOptions;
    fallbackOptions.UnsupportedTimedTopologyPolicy = "ruckigStopAtWaypoints";
    [testCase.TestData.FailResult, testCase.TestData.FailResultDiagnosis]         = planner(obstacles, initialState, goalState, limits, failOptions);
    [testCase.TestData.FallbackResult, testCase.TestData.FallbackResultDiagnosis] = planner(obstacles, initialState, goalState, limits, fallbackOptions);
    testCase.TestData.FallbackValidation = obstacleAvoidance.validateTrajectory(testCase.TestData.FallbackResult);
end

function testDefaultPolicyPreservesEarliestTimedFailure(testCase)
    % Retain the rejected motion and each eligibility failure without invoking Ruckig.
    result          = testCase.TestData.FailResult;
    resultDiagnosis = testCase.TestData.FailResultDiagnosis;
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
    verifyEqual(testCase, result.Options.UnsupportedTimedTopologyPolicy, "fail");
    verifyNotEmpty(testCase, resultDiagnosis.Attempts);
    % Exercise each seed covered by this regression.
    for seedIndex = 1:numel(resultDiagnosis.Attempts)
        diagnostics = testSupport.solverDetails(resultDiagnosis, seedIndex);
        verifyFalse(testCase, testSupport.diagnosisValue(diagnostics, "FallbackAttempted"));
        verifyEqual(testCase, testSupport.diagnosisValue(diagnostics, "FallbackOutcome"), "fallbackDisabledByPolicy");
        expectedReason = "unsupportedTimedMultiWaypointRoute";
        if seedIndex == 1
            expectedReason = "unsupportedDynamicDirectGuess";
        end
        verifyEqual(testCase, testSupport.diagnosisValue(diagnostics, "OriginalTerminationReason"), expectedReason);
    end
end

function testExplicitPolicyAttemptsFallbackOnlyWhenEnabled(testCase)
    % Require an explicit fallback attempt without manufacturing solver failure.
    result          = testCase.TestData.FallbackResult;
    resultDiagnosis = testCase.TestData.FallbackResultDiagnosis;
    verifyFalse(testCase, result.Success);
    verifyNotEmpty(testCase, result.time_s);
    verifyFalse(testCase, result.Validation.Passed);
    verifyFalse(testCase, testCase.TestData.FallbackValidation.Passed);
    verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
    verifyEqual(testCase, result.Options.UnsupportedTimedTopologyPolicy, "ruckigStopAtWaypoints");
    fallbackAttemptCount = 0;
    % Exercise each seed covered by this regression.
    for seedIndex = 1:numel(resultDiagnosis.Attempts)
        diagnostics = testSupport.solverDetails(resultDiagnosis, seedIndex);
        if ~any(diagnostics.Field == "FallbackAttempted") || ~testSupport.diagnosisValue(diagnostics, "FallbackAttempted")
            continue;
        end
        fallbackAttemptCount = fallbackAttemptCount + 1;
        verifyEqual(testCase, testSupport.diagnosisValue(diagnostics, "FallbackMethod"), "ruckigStopAtWaypoints");
        expectedReason = "unsupportedTimedMultiWaypointRoute";
        if seedIndex == 1
            expectedReason = "unsupportedDynamicDirectGuess";
        end
        verifyEqual(testCase, testSupport.diagnosisValue(diagnostics, "OriginalTerminationReason"), expectedReason);
        fallback = diagnostics;
        verifyLessThanOrEqual(testCase, testSupport.diagnosisValue(fallback, "FallbackDiagnostics.CompletedPartCount"), testSupport.diagnosisValue(fallback, "FallbackDiagnostics.MaximumSupportedPartCount"));
        verifyNotEmpty(testCase, testSupport.diagnosisValue(diagnostics, "FallbackOutcome"));
    end
    verifyGreaterThan(testCase, fallbackAttemptCount, 0);
end

function [obstacles, initialState, goalState, limits] = createScenario()
    % Use a tight physical deadline so timed recovery fails without solver tuning.
    missionEndTime_s = 8;
    obstacleTime_s   = [0; missionEndTime_s];
    uPosition_units    = [ ...
        -8 7; -5 7; -5 -4; 5 -4; 5 7; 8 7; 8 -7; -8 -7];
    staticObstacle  = obstacleAvoidance.obstacles.createObstacle("static U-shaped obstacle", obstacleTime_s, uPosition_units(:, 1), uPosition_units(:, 2), 0.20);
    movingStart_units = [30 30; 32 30; 32 32; 30 32];
    movingEnd_units   = movingStart_units + [4 0];
    movingObstacle  = obstacleAvoidance.obstacles.createObstacle("distant moving obstacle", obstacleTime_s, {movingStart_units(:, 1); movingEnd_units(:, 1)}, {movingStart_units(:, 2); movingEnd_units(:, 2)}, 0.10);
    obstacles       = obstacleAvoidance.obstacles.combineObstacles(staticObstacle, movingObstacle);
    initialState    = struct();
    initialState.time_s       = 0;
    initialState.position_units = [0 0];
    goalState = struct("time_s", missionEndTime_s, "position_units", [0 -10]);
    limits = struct();
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [0.75 0.75];
    limits.maxJerk_units_s3         = [2.5 2.5];
end
