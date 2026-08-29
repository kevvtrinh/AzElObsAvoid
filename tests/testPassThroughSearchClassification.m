function tests = testPassThroughSearchClassification
%% Section 0: Header & Readme
% SYNTAX
%   tests = testPassThroughSearchClassification
%**************************************************************************
% PURPOSE
%   - Verify the route-geometry classification shared by preliminary and
%     refined pass-through waypoint searches.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest.FunctionTestCase array)
%       Function-based tests for scalar, coupled-monotone, and reversing
%       waypoint chains.
%**************************************************************************
% UNITS
%   - Route positions and tolerances use degrees.
%**************************************************************************
tests = functiontests(localfunctions);
end

%% Section 1: Local Functions

function setupOnce(testCase)
% Add the repository root for isolated package-function test runs.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
assumeTrue(testCase, isfile(fullfile(repositoryRoot, ...
    "+obstacleAvoidance", "+planner", "ruckigWarmStart.m")), ...
    "The optional Ruckig warm-start module is not installed.");
end

function testScalarAxisRoute(testCase)
% One globally monotone axis selects the bounded scalar search.
classification = ...
    obstacleAvoidance.planner.ruckigWarmStart( ...
    "classifySearch", [0, 0; 1, 1; 2, 0], 1e-8);
verifyEqual(testCase, classification.Mode, ...
    "oneDimensionalScalar");
verifyEqual(testCase, classification.ActiveAxisIndex, [1, 2]);
end

function testCoupledMonotoneRoute(testCase)
% Two globally monotone axes require synchronized vector-state probes.
classification = ...
    obstacleAvoidance.planner.ruckigWarmStart( ...
    "classifySearch", [0, 0; 1, 2; 3, 3], 1e-8);
verifyEqual(testCase, classification.Mode, "coupledMonotone");
verifyEqual(testCase, classification.ActiveAxisIndex, [1, 2]);
end

function testCoupledReversingRoute(testCase)
% No globally monotone active axis selects the reversing vector search.
classification = ...
    obstacleAvoidance.planner.ruckigWarmStart( ...
    "classifySearch", [0, 0; 1, 1; 0, 0], 1e-8);
verifyEqual(testCase, classification.Mode, "coupledReversing");
verifyEqual(testCase, classification.ActiveAxisIndex, [1, 2]);
end
