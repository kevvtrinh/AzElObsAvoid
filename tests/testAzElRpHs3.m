function tests = testAzElRpHs3
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests("tests/testAzElRpHs3.m")
%**************************************************************************
% PURPOSE
%   - Verify required RL seed generation and the single HS-3 motion method.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest.FunctionTestCase array)
%       Deterministic RL seed and production-planner regression cases.
%**************************************************************************
% UNITS
%   - Position is degrees. Time is seconds. Derivative suffixes state units.
%**************************************************************************

%% Section 1: Create The Function-Test Suite

tests = functiontests(localfunctions);
end

%% Section 2: Local Test Cases

function setupOnce(testCase)
% PURPOSE
%   - Put the repository API on the test path.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
testCase.TestData.OriginalPath = path;
addpath(repositoryRoot);
end

function teardownOnce(testCase)
% PURPOSE
%   - Restore the caller path.
path(testCase.TestData.OriginalPath);
end

function testDefaultsRequireTheDeployedAgent(testCase)
% PURPOSE
%   - Keep the RL seed controls in the single planner defaults record.
options = planAzElMotion();

testCase.verifyTrue(isfile(options.RpAgentFile));
testCase.verifyGreaterThan(options.RpTurnRadius_deg, 0);
testCase.verifyFalse(isfield(options, "UseRpAgent"));
end

function testAgentRoundsOneCornerDeterministically(testCase)
% PURPOSE
%   - Evaluate the saved policy and create the same G3 seed on repeat calls.
seed = seedRecord([0 0; 10 0; 10 10], zeros(0, 1));
limits = motionLimits();
options = planAzElMotion();

[firstSeed, firstDiagnostics] = ...
    azElInternal.buildAzElRpHs3Seeds(seed, limits, options);
[secondSeed, secondDiagnostics] = ...
    azElInternal.buildAzElRpHs3Seeds(seed, limits, options);

testCase.verifyEqual(firstDiagnostics(2).PolicyStatus, ...
    "proposalEvaluated");
testCase.verifyTrue(all([firstDiagnostics.AgentLoaded]));
testCase.verifyEqual(firstDiagnostics(2).CornerCount, 1);
testCase.verifyEqual(firstDiagnostics(2).EvaluatedCornerCount, 1);
testCase.verifyEqual(firstDiagnostics(2).SeedVariant, "agentRounded");
testCase.verifyTrue(firstDiagnostics(2).AgentRouteApplied);
testCase.verifyGreaterThan(size(firstSeed(2).Route_deg, 1), 3);
testCase.verifyEqual(firstSeed(2).Route_deg([1 end], :), ...
    seed.Route_deg([1 end], :), "AbsTol", 0);
testCase.verifyEqual(firstSeed(2).Route_deg, secondSeed(2).Route_deg, ...
    "AbsTol", 0);
testCase.verifyEqual(firstDiagnostics(2).RadiusScale, ...
    secondDiagnostics(2).RadiusScale, "AbsTol", 0);
end

function testTimedSippLawSurvivesRlRounding(testCase)
% PURPOSE
%   - Preserve waiting and progress evidence independently of new seed rows.
seed = seedRecord([0 0; 5 0; 5 0; 10 0], [0; 3; 5; 12]);
limits = motionLimits();
options = planAzElMotion();

[roundedSeed, diagnostics] = ...
    azElInternal.buildAzElRpHs3Seeds(seed, limits, options);

testCase.verifyTrue(diagnostics(2).TimedSeedLawPreserved);
testCase.verifyEqual(roundedSeed(2).RpSeedRouteTimeFraction, ...
    [0; 0.25; 5 / 12; 1], "AbsTol", 1e-12);
testCase.verifyEqual(roundedSeed(2).RpSeedRouteProgress, ...
    [0; 0.5; 0.5; 1], "AbsTol", 1e-12);
testCase.verifyEqual(roundedSeed(2).RpSeedRouteDuration_s, 12);
testCase.verifyEmpty(roundedSeed(2).RouteTime_s);
end

function testMissingAgentFailsWithoutFallback(testCase)
% PURPOSE
%   - Prevent silent substitution of a deterministic or older seed method.
seed = seedRecord([0 0; 1 0], zeros(0, 1));
options = planAzElMotion();
options.RpAgentFile = fullfile(tempdir, ...
    "missing-az-el-rp-agent.mat");

testCase.verifyError(@() azElInternal.buildAzElRpHs3Seeds( ...
    seed, motionLimits(), options), ...
    "buildAzElRpHs3Seeds:AgentNotFound");
end

function testDirectPlannerReportsRlSeededHs3(testCase)
% PURPOSE
%   - Exercise the maintained planner method without an obstacle detour.
initialState = stateRecord(0, [0 0]);
goalState = stateRecord(20, [2 1]);
options = planAzElMotion();
options.MaximumDirectCollocationSeeds = 1;
options.InitialCollocationSegmentCount = 4;
options.MaximumCollocationSegmentCount = 4;
options.MaximumMeshRefinementPasses = 0;
options.MaximumPlanningTime_s = 30;
options.UseParallel = "off";

result = planAzElMotion( ...
    [], initialState, goalState, motionLimits(), options);

testCase.verifyTrue(result.Success, result.Message);
testCase.verifyEqual(result.timedSlopePath.RetimerType, ...
    "rlSeededHs3");
testCase.verifyEqual(result.SearchDiagnostics.RpSeedMethod, ...
    "requiredRlCornerPolicy");
testCase.verifyTrue(all([result.SearchDiagnostics. ...
    RpSeedDiagnostics.AgentLoaded]));
end

function seed = seedRecord(route_deg, routeTime_s)
% PURPOSE
%   - Build one stable topology seed for focused RL tests.
seed = struct( ...
    "Source", "test", ...
    "SnapshotTime_s", 0, ...
    "GraphIndex", 0, ...
    "RouteCost_deg", sum(vecnorm(diff(route_deg), 2, 2)), ...
    "Route_deg", route_deg, ...
    "RouteTime_s", routeTime_s, ...
    "RouteSignature", "test");
end

function state = stateRecord(time_s, position_deg)
% PURPOSE
%   - Build one stationary endpoint state.
state = struct( ...
    "time_s", time_s, ...
    "position_deg", position_deg, ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
end

function limits = motionLimits()
% PURPOSE
%   - Return finite physical limits used by all focused tests.
limits = struct( ...
    "maxVelocity_deg_s", [3 3], ...
    "maxAcceleration_deg_s2", [2 2], ...
    "maxJerk_deg_s3", [2.5 2.5]);
end
