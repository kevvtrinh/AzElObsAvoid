function tests = testTimedRouteArrivalHandoff
%% Section 0: Header & Readme
% SYNTAX
%   tests = testTimedRouteArrivalHandoff
% PURPOSE
%   Keep the saved early-turn request from reverting to a full-history detour.
% INPUTS
%   None; the supplied diagnosis bundle is replayed without changing inputs.
% OUTPUTS
%   MATLAB function tests for motion quality and independent validation.
% UNITS
%   Coordinate units and seconds.

%% Section 1: Register Tests
tests = functiontests(localfunctions);
end

function testSavedGrowingObstacleUsesEarlierTurn(testCase)
    % Replay the exact supplied request; do not manufacture a favorable scene.
    repositoryRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(repositoryRoot, fullfile(repositoryRoot, 'trajectory'));
    loaded = load(fullfile(repositoryRoot, 'Rogue Examples', 'inefficientroute.mat'));
    bundle = loaded.diagnosisBundle;
    inputs = bundle.PlannerInputs;
    [result, diagnosis] = planner(inputs.obstacles, inputs.initialState, inputs.goalState, inputs.limits, bundle.PlannerOptions);
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyLessThanOrEqual(testCase, result.TrajectoryDuration_s, 0.95 * bundle.Result.TrajectoryDuration_s);
    verifyLessThan(testCase, sum(vecnorm(diff(result.position_units), 2, 2)), sum(vecnorm(diff(bundle.Result.position_units), 2, 2)));
    verifyEqual(testCase, diagnosis.Routes(diagnosis.SelectedAttemptIndex).Source, "timeExpandedVisibilityGraph");
    details = testSupport.solverDetails(diagnosis, diagnosis.SelectedAttemptIndex);
    times = details(endsWith(details.Field, 'WarmStartWaypointTime_s'), :);
    verifyNotEmpty(testCase, times);
    timedSeed = bundle.Diagnosis.Routes(2);
    originalInteriorTime_s = inputs.initialState.time_s + timedSeed.tau(2:end - 1) * timedSeed.EstimatedDuration_s;
    % Every attempted arrival must retain the search's early crossing time.
    for trialIndex = 1:height(times)
        verifyEqual(testCase, times.Value{trialIndex}(2:end - 1), originalInteriorTime_s, 'AbsTol', 1e-12);
    end
end

function testSharedClockKeepsShortLivedEvents(testCase)
    % An arrival grid must retain brief changes instead of thinning them out.
    obstacle = struct('time_s', [10; 11; 11.001; 25]);
    times = obstacleAvoidance.search.createTimeLayers(obstacle, 10, 25);
    % Compute the midpoint from the stored endpoints; its decimal spelling
    % can round to a neighboring double even though the event was retained.
    midpoint_s = (obstacle.time_s(2) + obstacle.time_s(3)) / 2;
    verifyTrue(testCase, all(ismember([10; 11; midpoint_s; 11.001; 25], times)));
    verifyGreaterThan(testCase, min(diff(times)), 0);
    verifyEqual(testCase, times([1 end]), [10; 25]);
end
