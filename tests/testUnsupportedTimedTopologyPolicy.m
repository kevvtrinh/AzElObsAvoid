function tests = testUnsupportedTimedTopologyPolicy
%% Section 0: Header & Readme
% SYNTAX
%   tests = testUnsupportedTimedTopologyPolicy
%**************************************************************************
% PURPOSE
%   - Verify that unsupported smooth timed routes fail by default and use
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
%   - Position is degrees and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Run the identical deterministic request under both public policy values.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "trajectory"));
[obstacles, initialState, goalState, limits] = createScenario();
baseOptions = struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "MaximumSeedCount", 3, ...
    "MaximumNlpIterations", 1, ...
    "SampleTime_s", 0.05);
failOptions = baseOptions;
failOptions.UnsupportedTimedTopologyPolicy = "fail";
fallbackOptions = baseOptions;
fallbackOptions.UnsupportedTimedTopologyPolicy = ...
    "ruckigStopAtWaypoints";
testCase.TestData.FailResult = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, failOptions);
testCase.TestData.FallbackResult = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, fallbackOptions);
testCase.TestData.FallbackValidation = obstacleAvoidance.validateTrajectory( ...
    testCase.TestData.FallbackResult);
end

function testDefaultPolicyPreservesEarliestTimedFailure(testCase)
% Return the unsupported smooth topology without invoking Ruckig.
result = testCase.TestData.FailResult;
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, ...
    "unsupportedTimedMultiWaypointRoute");
verifyEqual(testCase, result.Options.UnsupportedTimedTopologyPolicy, "fail");
verifyNotEmpty(testCase, result.SeedSummaries);
for seedIndex = 1:numel(result.SeedSummaries)
    diagnostics = result.SeedSummaries(seedIndex).SolverDiagnostics;
    verifyFalse(testCase, diagnostics.FallbackAttempted);
    verifyEqual(testCase, diagnostics.FallbackOutcome, ...
        "fallbackDisabledByPolicy");
    verifyEqual(testCase, diagnostics.OriginalTerminationReason, ...
        "unsupportedTimedMultiWaypointRoute");
end
end

function testExplicitPolicyStillRejectsOverLimitFallbackRoutes(testCase)
% Keep explicit fallback bounded to two segments and report every attempt.
result = testCase.TestData.FallbackResult;
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
verifyEqual(testCase, result.Options.UnsupportedTimedTopologyPolicy, ...
    "ruckigStopAtWaypoints");
fallbackAttemptCount = 0;
overLimitAttemptCount = 0;
for seedIndex = 1:numel(result.SeedSummaries)
    diagnostics = result.SeedSummaries(seedIndex).SolverDiagnostics;
    if ~isfield(diagnostics, "FallbackAttempted") || ...
            ~diagnostics.FallbackAttempted
        continue;
    end
    fallbackAttemptCount = fallbackAttemptCount + 1;
    verifyEqual(testCase, diagnostics.FallbackMethod, ...
        "ruckigStopAtWaypoints");
    verifyEqual(testCase, diagnostics.OriginalTerminationReason, ...
        "unsupportedTimedMultiWaypointRoute");
    fallback = diagnostics.FallbackDiagnostics;
    verifyLessThanOrEqual(testCase, fallback.CompletedPartCount, ...
        fallback.MaximumSupportedPartCount);
    if fallback.RequestedPartCount > fallback.MaximumSupportedPartCount
        overLimitAttemptCount = overLimitAttemptCount + 1;
        verifyFalse(testCase, fallback.Accepted);
        verifyEqual(testCase, fallback.CompletedPartCount, 0);
        verifyEqual(testCase, fallback.EngineTerminationReason, ...
            "ruckigWaypointSegmentLimitExceeded");
    end
end
verifyGreaterThan(testCase, fallbackAttemptCount, 0);
verifyGreaterThan(testCase, overLimitAttemptCount, 0, ...
    "The integration case did not exercise an over-limit fallback route.");
end

function [obstacles, initialState, goalState, limits] = createScenario()
% Create a static detour plus a distant mover that triggers timed routing.
missionEndTime_s = 120;
obstacleTime_s = [0; missionEndTime_s];
uPosition_deg = [ ...
    -8 7; -5 7; -5 -4; 5 -4; 5 7; 8 7; 8 -7; -8 -7];
staticObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "static U-shaped obstacle", obstacleTime_s, ...
    uPosition_deg(:, 1), uPosition_deg(:, 2), 0.20);
movingStart_deg = [30 30; 32 30; 32 32; 30 32];
movingEnd_deg = movingStart_deg + [4 0];
movingObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "distant moving obstacle", obstacleTime_s, ...
    {movingStart_deg(:, 1); movingEnd_deg(:, 1)}, ...
    {movingStart_deg(:, 2); movingEnd_deg(:, 2)}, 0.10);
obstacles = obstacleAvoidance.obstacles.combineObstacles( ...
    staticObstacle, movingObstacle);
initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [0 -10]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", [2.5 2.5]);
end
