function tests = testTimedSearchWorkLimits
%% Section 0: Header & Readme
% SYNTAX
%   tests = testTimedSearchWorkLimits
%**************************************************************************
% PURPOSE
%   - Verify explicit state and transition bounds for timed visibility
%     search without relying on index-width limits.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function-test array)
%       Deterministic timed-search work-bound regression cases.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add the repository package root for direct search tests.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
end

function testLargeLayerRequestIsBoundedByEstimatedWork(testCase)
% Bound the retained layers by both declared work budgets.
[nodePosition_deg, edgeCost_deg, initialState, goalState, limits] = ...
    createSearchInputs();
options = obstacleAvoidance.input.resolvePlannerOptions(struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "MaximumTimeLayerCount", 1000000, ...
    "MaximumTimedSearchStateCount", 9, ...
    "MaximumTimedSearchTransitionCount", 18));
[route_deg, routeTime_s, record] = ...
    obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
    nodePosition_deg, edgeCost_deg, [], initialState, goalState, limits, ...
    linspace(0, 10, 21), options);

verifyNotEmpty(testCase, route_deg);
verifyNotEmpty(testCase, routeTime_s);
verifyEqual(testCase, record.EffectiveMaximumLayerCount, 3);
verifyLessThanOrEqual(testCase, record.EstimatedStateCount, ...
    record.MaximumStateCount);
verifyLessThanOrEqual(testCase, record.EstimatedTransitionCount, ...
    record.MaximumTransitionCount);
verifyTrue(testCase, record.StateLimitApplied);
verifyTrue(testCase, record.TransitionLimitApplied);
verifyTrue(testCase, record.WorkLimitExceeded);
verifyEqual(testCase, record.TerminationReason, "goalReached");
end

function testInsufficientWorkBudgetReturnsStableDiagnostics(testCase)
% Reject an unaffordable two-layer search without allocating its arrays.
[nodePosition_deg, edgeCost_deg, initialState, goalState, limits] = ...
    createSearchInputs();
options = obstacleAvoidance.input.resolvePlannerOptions(struct( ...
    "MaximumTimedSearchStateCount", 5, ...
    "MaximumTimedSearchTransitionCount", 8));
[route_deg, routeTime_s, record] = ...
    obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
    nodePosition_deg, edgeCost_deg, [], initialState, goalState, limits, ...
    linspace(0, 10, 21), options);

verifySize(testCase, route_deg, [0 2]);
verifySize(testCase, routeTime_s, [0 1]);
verifyEqual(testCase, record.EffectiveMaximumLayerCount, 1);
verifyEqual(testCase, record.EstimatedStateCount, 0);
verifyEqual(testCase, record.EstimatedTransitionCount, 0);
verifyTrue(testCase, record.WorkLimitExceeded);
verifyEqual(testCase, record.TerminationReason, "timedSearchWorkLimit");
end

function [nodePosition_deg, edgeCost_deg, initialState, goalState, limits] = ...
        createSearchInputs()
% Create a three-node fully connected graph with inexpensive dynamics.
nodePosition_deg = [0 0; 2 0; 1 1];
azimuthDifference_deg = ...
    nodePosition_deg(:, 1) - nodePosition_deg(:, 1).';
elevationDifference_deg = ...
    nodePosition_deg(:, 2) - nodePosition_deg(:, 2).';
edgeCost_deg = hypot(azimuthDifference_deg, elevationDifference_deg);
initialState = struct("time_s", 0, "position_deg", nodePosition_deg(1, :));
goalState = struct("time_s", 10, "position_deg", nodePosition_deg(2, :));
limits = struct( ...
    "maxVelocity_deg_s", [100 100], ...
    "maxAcceleration_deg_s2", [100 100]);
end
