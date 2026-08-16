function tests = testBenchmarkComparison
%% Section 0: Header & Readme
% SYNTAX
%   tests = testBenchmarkComparison()
%**************************************************************************
% PURPOSE
%   - Verify deterministic parity, dominance, key, and runtime benchmark
%     comparisons using synthetic report tables.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest.Test array)
%       Function-based tests for compareAzElBenchmarkReports.
%**************************************************************************
% UNITS
%   - Synthetic paths use degrees and synthetic durations use seconds.
%**************************************************************************

%% Section 1: Register Local Tests

tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   setupOnce(testCase)
%**************************************************************************
% PURPOSE
%   - Make the repository and benchmark entry point available to tests.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%       Test fixture receiving cleanup registration.
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Paths are text.
%**************************************************************************
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
originalPath = path;
addpath(repositoryRoot, fullfile(repositoryRoot, "benchmarks"));
testCase.addTeardown(@() path(originalPath));
end

function testParityAcceptsReorderedEquivalentRows(testCase)
% Verify order-independent equality when every synthetic row agrees.
baseline = syntheticReport();
current = baseline;
current.Runs = current.Runs([2 1], :);

comparison = compareAzElBenchmarkReports(baseline, current, "parity");

testCase.verifyTrue(comparison.Passed, comparison.Message);
testCase.verifyEqual(comparison.Mode, "parity");
testCase.verifyTrue(comparison.KeySetMatched);
testCase.verifyTrue(comparison.OutcomesPassed);
testCase.verifyTrue(comparison.MetricsPassed);
testCase.verifyFalse(comparison.RuntimeAssessed);
testCase.verifyEqual(height(comparison.Rows), 2);
end

function testDominanceAcceptsLowerPhysicalMetrics(testCase)
% Verify every physical metric is lower-is-better only in dominance mode.
baseline = syntheticReport();
current = baseline;
current.Runs.SelectedPolylineLength_deg(1) = 9.5;
current.Runs.SmoothedPathLength_deg(1) = 9.4;
current.Runs.MinimumMotionDuration_s(1) = 4.5;

parity = compareAzElBenchmarkReports(baseline, current, "parity");
dominance = compareAzElBenchmarkReports( ...
    baseline, current, "dominance");

testCase.verifyFalse(parity.Passed);
testCase.verifyTrue(dominance.Passed, dominance.Message);
testCase.verifyTrue(dominance.Rows.MetricPassed(1));
testCase.verifyLessThan(dominance.Rows.PolylineDelta_deg(1), 0);
end

function testDominanceRejectsWorseDurationAndCertificateState(testCase)
% Verify duration and certificate-state regressions remain visible.
baseline = syntheticReport();
current = baseline;
current.Runs.MinimumMotionDuration_s(2) = 7.1;
current.Runs.FiniteJerkCertified(2) = false;

comparison = compareAzElBenchmarkReports( ...
    baseline, current, "dominance");

testCase.verifyFalse(comparison.Passed);
testCase.verifyFalse(comparison.Rows.DurationPassed(2));
testCase.verifyFalse(comparison.Rows.FiniteJerkCertifiedSame(2));
testCase.verifyFalse(comparison.Rows.OutcomePassed(2));
end

function testRuntimeAssessmentUsesExplicitRatio(testCase)
% Verify runtime stays diagnostic until callers explicitly enable its gate.
baseline = syntheticReport();
current = baseline;
current.Runs.WallTime_s = [12; 6];

failingComparison = compareAzElBenchmarkReports( ...
    baseline, current, "parity", struct( ...
    "AssessRuntime", true, "MaximumRuntimeRatio", 1.10));
passingComparison = compareAzElBenchmarkReports( ...
    baseline, current, "parity", struct( ...
    "AssessRuntime", true, "MaximumRuntimeRatio", 1.20));

testCase.verifyFalse(failingComparison.Passed);
testCase.verifyFalse(failingComparison.Rows.RuntimePassed(1));
testCase.verifyEqual(failingComparison.Rows.RuntimeRatio(1), 1.2, ...
    "AbsTol", eps);
testCase.verifyTrue(passingComparison.Passed, passingComparison.Message);
end

function testKeyMismatchProducesStableFailure(testCase)
% Verify missing and unexpected row keys yield an inspectable failure.
baseline = syntheticReport();
current = baseline;
current.Runs.Example(2) = "syntheticOther";

comparison = compareAzElBenchmarkReports(baseline, current, "parity");

testCase.verifyFalse(comparison.Passed);
testCase.verifyFalse(comparison.KeySetMatched);
testCase.verifyEqual(comparison.ComparedRowCount, 3);
testCase.verifyEqual(nnz(~comparison.Rows.HasBaseline | ...
    ~comparison.Rows.HasCurrent), 2);
end

function report = syntheticReport()
%% Section 0: Header & Readme
% SYNTAX
%   report = syntheticReport()
%**************************************************************************
% PURPOSE
%   - Build a deterministic generic benchmark report for comparator tests.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - report (scalar struct)
%       Report with a two-row Runs table and no planner-specific values.
%**************************************************************************
% UNITS
%   - Paths use degrees and durations and runtimes use seconds.
%**************************************************************************
runs = table( ...
    ["syntheticOne"; "syntheticTwo"], logical([false; true]), ...
    logical([true; true]), logical([true; true]), ...
    logical([true; true]), logical([true; true]), ...
    logical([true; true]), logical([true; true]), ...
    logical([false; true]), ["goalReached"; "goalReached"], ...
    [10; 12], [9.8; 11.7], [5; 7], [10; 5], ...
    'VariableNames', { ...
    'Example', 'JerkConstrained', 'RunCompleted', 'PlannerSuccess', ...
    'ValidationPassed', 'CollisionFree', 'KinematicsPassed', ...
    'CertificatePassed', 'FiniteJerkCertified', 'TerminationReason', ...
    'SelectedPolylineLength_deg', 'SmoothedPathLength_deg', ...
    'MinimumMotionDuration_s', 'WallTime_s'});
report = struct("Runs", runs);
end
