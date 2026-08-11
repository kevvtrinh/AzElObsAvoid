function tests = testRlBranchExamples
%% Section 0: Header & Readme
% SYNTAX
%   tests = testRlBranchExamples
%**************************************************************************
% PURPOSE
%   - Preserve acceptance of the analytic missions imported from the
%     all-dijkstra-rl-parameter-auto-tune branch.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function-based test suite)
%**************************************************************************
% UNITS
%   - Not applicable.

tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   setupOnce(testCase)
%**************************************************************************
% PURPOSE
%   - Add the repository and example folders for imported-case tests.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Not applicable.

repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "examples"));
testCase.TestData.repositoryRoot = repositoryRoot;
end

function testAllAnalyticArrivalOraclesPass(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testAllAnalyticArrivalOraclesPass(testCase)
%**************************************************************************
% PURPOSE
%   - Require exact validation and the source branch's 1.03 arrival ratio.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Arrival ratios are dimensionless.

report = example04RlBranchArrivalOracles();
verifyEqual(testCase, report.caseCount, 4);
verifyEqual(testCase, report.passedCount, report.caseCount);
verifyLessThanOrEqual(testCase, report.maximumArrivalRatio, ...
    report.acceptanceThreshold + 1e-12);
for caseIndex = 1:report.caseCount
    verifyTrue(testCase, report.cases(caseIndex).result.success);
    verifyTrue(testCase, report.cases(caseIndex).validation.isValid);
    verifyEqual(testCase, ...
        report.cases(caseIndex).result.guarantee.optimality, ...
        "Minimum arrival under a stated model");
end
end
