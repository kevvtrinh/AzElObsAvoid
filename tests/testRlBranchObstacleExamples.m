function tests = testRlBranchObstacleExamples
%% Section 0: Header & Readme
% SYNTAX
%   tests = testRlBranchObstacleExamples
%**************************************************************************
% PURPOSE
%   - Preserve certified acceptance of representative obstacle missions
%     imported from the all-dijkstra-rl-parameter-auto-tune branch.
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

function setupOnce(~)
%% Section 0: Header & Readme
% SYNTAX
%   setupOnce(testCase)
%**************************************************************************
% PURPOSE
%   - Add the repository and example folders for imported obstacle tests.
%**************************************************************************
% INPUTS
%   - testCase (unused matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Not applicable.

repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "examples"));
end

function testStaticDetour(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testStaticDetour(testCase)
%**************************************************************************
% PURPOSE
%   - Require a certified command around the imported static blocker.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Clearance is degrees.

result = example05RlStaticDetour(false);
verifyTrue(testCase, result.success);
verifyTrue(testCase, result.validation.isValid);
verifyGreaterThan(testCase, ...
    result.validation.collision.minimumClearance_deg, 0.1);
end

function testDynamicSafeIntervals(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testDynamicSafeIntervals(testCase)
%**************************************************************************
% PURPOSE
%   - Require a certified command through the imported moving-wall scene.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Clearance is degrees.

result = example06RlDynamicSafeIntervals(false);
verifyTrue(testCase, result.success);
verifyTrue(testCase, result.validation.isValid);
verifyGreaterThan(testCase, ...
    result.validation.collision.minimumClearance_deg, 0.15);
end

function testWrappedSeamDetour(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testWrappedSeamDetour(testCase)
%**************************************************************************
% PURPOSE
%   - Require a continuous certified detour across the azimuth seam.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Elevation is degrees.

result = example07RlWrappedSeamDetour(false);
verifyTrue(testCase, result.success);
verifyTrue(testCase, result.validation.isValid);
verifyTrue(testCase, result.validation.seamIsValid);
verifyGreaterThan(testCase, ...
    max(abs(result.command.unwrappedPosition_deg(:, 2))), 5);
end

function testNarrowPassage(testCase)
%% Section 0: Header & Readme
% SYNTAX
%   testNarrowPassage(testCase)
%**************************************************************************
% PURPOSE
%   - Require exact clearance through the imported narrow opening.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Clearance is degrees.

result = example08RlNarrowPassage(false);
verifyTrue(testCase, result.success);
verifyTrue(testCase, result.validation.isValid);
verifyGreaterThan(testCase, ...
    result.validation.collision.minimumClearance_deg, 0.5);
end
