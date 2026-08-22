function tests = testQuinticBsplineOptimizerPrototype
%% Section 0: Header & Readme
% SYNTAX
%   tests = testQuinticBsplineOptimizerPrototype
%**************************************************************************
% PURPOSE
%   - Verify deterministic bounded-search success and honest expected failure
%     before any production or learning-policy integration is considered.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest.Test array)
%       Deterministic function-based research optimizer tests.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************
tests = functiontests(localfunctions);
end

function testSingleTurnPassesMaintainedValidation(testCase)
% A sampled-clear search result is insufficient; exact validation must pass.
[obstacles, initialState, goalState, limits, route_deg] = ...
    repeatedTurnRequest(1);
motion = optimizeQuinticBsplinePrototype( ...
    obstacles, initialState, goalState, limits, route_deg);

verifyTrue(testCase, motion.Success);
verifyTrue(testCase, motion.Validation.Passed);
verifyTrue(testCase, motion.Validation.CollisionFree);
verifyTrue(testCase, motion.Validation.CollisionResolved);
verifyGreaterThan(testCase, motion.Validation.MinimumClearance_deg, 0);
verifyLessThanOrEqual(testCase, motion.FinalTime_s, goalState.time_s);
verifyLessThanOrEqual(testCase, ...
    motion.OptimizerDiagnostics.EvaluationCount, ...
    motion.OptimizerOptions.MaximumFunctionEvaluations);
verifyTrue(testCase, motion.OptimizerDiagnostics.HasValidatedMotion);
end

function testSingleTurnDecisionIsDeterministic(testCase)
[obstacles, initialState, goalState, limits, route_deg] = ...
    repeatedTurnRequest(1);
firstMotion = optimizeQuinticBsplinePrototype( ...
    obstacles, initialState, goalState, limits, route_deg);
secondMotion = optimizeQuinticBsplinePrototype( ...
    obstacles, initialState, goalState, limits, route_deg);

verifyEqual(testCase, ...
    firstMotion.OptimizerDiagnostics.SelectedDecision_deg, ...
    secondMotion.OptimizerDiagnostics.SelectedDecision_deg, ...
    "AbsTol", 0);
verifyEqual(testCase, firstMotion.Polynomial.positionPower_deg, ...
    secondMotion.Polynomial.positionPower_deg, "AbsTol", 0);
verifyEqual(testCase, ...
    firstMotion.OptimizerDiagnostics.EvaluationCount, ...
    secondMotion.OptimizerDiagnostics.EvaluationCount);
end

function testImpossibleHorizonReturnsTraceableFailure(testCase)
[obstacles, initialState, goalState, limits, route_deg] = ...
    repeatedTurnRequest(1);
goalState.time_s = 2;
motion = optimizeQuinticBsplinePrototype( ...
    obstacles, initialState, goalState, limits, route_deg, ...
    struct("MaximumSweeps", 1, "MaximumFunctionEvaluations", 10));

verifyFalse(testCase, motion.Success);
verifyFalse(testCase, motion.Validation.Passed);
verifyFalse(testCase, motion.OptimizerDiagnostics.HasValidatedMotion);
verifyNotEqual(testCase, motion.TerminationReason, "prototypeValidated");
verifyGreaterThan(testCase, ...
    motion.OptimizerDiagnostics.EvaluationCount, 0);
verifyNotEmpty(testCase, motion.OptimizerDiagnostics.EvaluationTrace);
end

function [obstacles, initialState, goalState, limits, route_deg] = ...
        repeatedTurnRequest(turnCount)
constants = scenarioConstants();
[obstacles, initialState, goalState, limits] = ...
    createRepeatedTurnBenchmarkScenario(turnCount, constants);
plannerOptions = planAzElMotion();
plannerOptions.GoalTimeMode = "earliestArrival";
plannerOptions.MaximumSeedCount = 3;
plannerOptions.RandomSeed = 325;
[seeds, ~] = azElInternal.generateAzElTopologySeeds( ...
    obstacles, initialState, goalState, limits, plannerOptions);
visibilitySeedIndex = find([seeds.Source] == "visibilityGraph", 1, "first");
if isempty(visibilitySeedIndex)
    error("testQuinticBsplineOptimizerPrototype:MissingVisibilitySeed", ...
        "The repeated-turn fixture did not generate a visibility seed.");
end
route_deg = seeds(visibilitySeedIndex).position_deg;
end

function constants = scenarioConstants()
constants = struct( ...
    "barrierSpacing_deg", 4, ...
    "barrierHalfWidth_deg", 0.7, ...
    "barrierCenterMagnitude_deg", 2.5, ...
    "barrierHalfHeight_deg", 2.5, ...
    "safetyMargin_deg", 0.1, ...
    "goalTimePerStage_s", 5.5, ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2], ...
    "elevationInterval_deg", [-5 5]);
end
