function tests = testConicSolver
%% Section 0: Header & Readme
% SYNTAX
%   tests = testConicSolver
%**************************************************************************
% PURPOSE
%   - Verify the isolated conic interface and comparison-table contract.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Test cone programs are dimensionless; runtime is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add the trajectory package and benchmark parents used by this test file.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot, ...
    fullfile(repositoryRoot, "trajectory"), ...
    fullfile(repositoryRoot, "benchmarks"));
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testZeroInputReturnsCompleteDefaults(testCase)
% Keep one source of truth for the retained backend configuration.
options = conicSolver.solve();
verifyEqual(testCase, string(fieldnames(options)), ...
    ["Backend"; "LinearSolver"; "ConstraintTolerance"; ...
    "OptimalityTolerance"; "MaxIterations"]);
verifyEqual(testCase, options.Backend, "coneprog");
verifyEqual(testCase, options.LinearSolver, "auto");
end

function testLinearConeProgramReturnsStableSuccess(testCase)
% Solve one bounded linear problem through the cone-compatible interface.
problem = createLinearProblem(false);
result = conicSolver.solve(problem, struct("MaxIterations", 100));
verifyTrue(testCase, result.Success, result.Message);
verifyEqual(testCase, result.TerminationReason, "solved");
verifyEqual(testCase, result.PrimalSolution, 1, "AbsTol", 1e-8);
verifyEqual(testCase, result.ObjectiveValue, 1, "AbsTol", 1e-8);
verifyGreaterThanOrEqual(testCase, result.ElapsedTime_s, 0);
verifyEqual(testCase, result.Options.MaxIterations, 100);
end

function testInfeasibleProgramReturnsStableFailure(testCase)
% Preserve an expected solver failure without throwing or inventing a value.
result = conicSolver.solve(createLinearProblem(true));
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "solverFailed");
verifyLessThanOrEqual(testCase, result.ExitFlag, 0);
end

function testUnknownOptionWarnsOnceAndIsIgnored(testCase)
% Reject silent option drift while retaining the documented defaults.
problem = createLinearProblem(false);
verifyWarning(testCase, ...
    @() conicSolver.solve(problem, struct("UnknownSetting", 4)), ...
    "conicSolver:UnknownOptions");
end

function testDimensionMismatchThrowsIdentifiedError(testCase)
% Catch malformed problems at the package boundary before coneprog runs.
problem = createLinearProblem(false);
problem.InequalityMatrix = zeros(1, 2);
verifyError(testCase, @() conicSolver.solve(problem), ...
    "conicSolver:VariableCountMismatch");
end

function testBenchmarkReturnsOneComparisonTable(testCase)
% Exercise all requested backend rows with minimal deterministic repetition.
controls = struct( ...
    "WarmupCount", 0, ...
    "RepetitionCount", 1, ...
    "DisplayTable", false);
resultTable = benchmarkConicSolvers(controls);
verifyEqual(testCase, height(resultTable), 16);
verifyEqual(testCase, unique(resultTable.Backend, "stable"), ...
    ["coneprog"; "mosek"; "clarabel"; "ecos"; "scs"]);
verifyTrue(testCase, all(resultTable.Valid( ...
    resultTable.Backend == "coneprog")));
end

function problem = createLinearProblem(isInfeasible)
% Create min x subject to x>=1 and optionally x<=0.
emptyCone = secondordercone(zeros(2, 1), zeros(2, 1), 0, 0);
cones = repmat(emptyCone, 0, 1);
inequalityMatrix = -1;
inequalityBound = -1;
if isInfeasible
    inequalityMatrix = [-1; 1];
    inequalityBound = [-1; 0];
end
problem = struct( ...
    "Kind", "unitLinear", ...
    "Objective", 1, ...
    "Cones", cones, ...
    "InequalityMatrix", inequalityMatrix, ...
    "InequalityBound", inequalityBound, ...
    "EqualityMatrix", zeros(0, 1), ...
    "EqualityBound", zeros(0, 1), ...
    "LowerBound", -Inf, ...
    "UpperBound", Inf);
end
