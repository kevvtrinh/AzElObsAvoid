function tests = testPlannerRefactorBenchmarks
%% Section 0: Header & Readme
% SYNTAX
%   tests = testPlannerRefactorBenchmarks
% PURPOSE
%   Keep unchanged physics distinct from expected outcomes and fresh validation.
% INPUTS
%   None.
% OUTPUTS
%   MATLAB function tests.
% UNITS
%   Times in the synthetic captures are seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'benchmarks'));
end

function testIdenticalPhysicsCannotHideFailedIndependentValidation(testCase)
before = capture("exampleObstacleFree", true, true);
after = before;
after.IndependentChecks{1}.Passed = false;
comparison = compareCaptures(testCase, before, after);
verifyTrue(testCase, comparison.ExamplePassed);
verifyTrue(testCase, comparison.BaselineChecksPassed);
verifyFalse(testCase, comparison.CandidateChecksPassed);
verifyFalse(testCase, comparison.Passed);
end

function testUnexpectedFailureCannotBeItsOwnExpectedOutcome(testCase)
% Old captures lacked ExpectedSuccess and could mark unexpected failures checked.
failed = capture("exampleObstacleFree", false, true);
comparison = compareCaptures(testCase, failed, failed);
verifyTrue(testCase, comparison.ExamplePassed);
verifyFalse(testCase, comparison.BaselineChecksPassed);
verifyFalse(testCase, comparison.Passed);
end

function testDeclaredNoPathRemainsAValidComparison(testCase)
failed = capture("exampleNoPath", false, true);
failed.ExpectedSuccess = false;
comparison = compareCaptures(testCase, failed, failed);
verifyTrue(testCase, comparison.Passed);
end

function testSuccessfulValidatedCapturePasses(testCase)
valid = capture("exampleObstacleFree", true, true);
valid.ExpectedSuccess = true;
comparison = compareCaptures(testCase, valid, valid);
verifyTrue(testCase, comparison.Passed);
end

function value = capture(name, success, checked)
% Minimal records isolate the comparison contract from planner runtime.
value = struct('StartingCommit', "test", 'ExampleNames', name, ...
    'PhysicalRecords', {{struct('Success', success, 'position_deg', [0 0;1 0])}}, ...
    'FocusedRecords', struct(), 'ElapsedTime_s', 1, ...
    'IndependentChecks', {{struct('Passed', checked)}});
end

function comparison = compareCaptures(testCase, before, after)
firstPath = string(tempname) + ".mat";
secondPath = string(tempname) + ".mat";
testCase.addTeardown(@() delete(firstPath));
testCase.addTeardown(@() delete(secondPath));
baseline = before;
save(firstPath, 'baseline');
baseline = after;
save(secondPath, 'baseline');
comparison = comparePlannerRefactorBaseline(firstPath, secondPath);
end

function testRuntimeRemovalPreservesPhysicalClocksAndSolverEvidence(testCase)
% Nested runtime changes are noise; duration and solver outcome changes are not.
record = struct('ElapsedPlanningTime_s', 3, 'ArrivalTime_s', 8, ...
    'TrajectoryDuration_s', 6, 'TotalTime_s', 6, ...
    'PlaneCertificate', struct('ConicSolver', ...
    struct('TotalTime_s', 0.3, 'CallCount', 4, 'Solver', "coneprog")));
repeated = record;
repeated.ElapsedPlanningTime_s = 7;
repeated.PlaneCertificate.ConicSolver.TotalTime_s = 0.8;
expected = stripPlannerRefactorRuntime({record});
verifyEqual(testCase, stripPlannerRefactorRuntime({repeated}), expected);
repeated.ArrivalTime_s = 9;
verifyNotEqual(testCase, stripPlannerRefactorRuntime({repeated}), expected);
repeated = record;
repeated.TotalTime_s = 7;
verifyNotEqual(testCase, stripPlannerRefactorRuntime({repeated}), expected);
repeated = record;
repeated.PlaneCertificate.ConicSolver.CallCount = 5;
verifyNotEqual(testCase, stripPlannerRefactorRuntime({repeated}), expected);
end
