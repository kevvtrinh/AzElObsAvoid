function tests = testRuckigEngine
%% Section 0: Header & Readme
% SYNTAX
%   tests = testRuckigEngine
%**************************************************************************
% PURPOSE
%   - Protect the direct, self-contained Ruckig-derived trajectory engine.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Fixture position is in abstract coordinate units and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add the trajectory product folder for direct engine tests.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(fullfile(repositoryRoot, "trajectory"));
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testExactMotionMatchesIndependentMinimum(testCase)
% Verify exact switching remains independently callable and certified.
[initialState, terminalState, limits] = restToRestFixture();
result = planTrajRuckig(initialState, terminalState, limits, ...
    struct("SampleTime", 0.01));
expectedDuration = 4 * nthroot(1 / 2, 3);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.Duration, expectedDuration, "AbsTol", 1e-10);
verifyEqual(testCase, ...
    sum(vecnorm(diff(result.position), 2, 2)), sqrt(5), ...
    "AbsTol", 1e-10);
end

function testAsymmetricBoundsAreRejected(testCase)
% Verify the exact engine identifies its unsupported derivative-bound family.
[initialState, terminalState, limits] = restToRestFixture();
limits.velocityLower = [-0.5, -20];
limits.velocityUpper = [10, 20];
result = ruckigEngine.solve(initialState, terminalState, limits, struct());
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, ...
    "unsupportedAsymmetricBounds");
end

function testPathConstraintsAreRejected(testCase)
% Verify a path row cannot be silently ignored by the switching engine.
[initialState, terminalState, limits] = restToRestFixture();
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct(), pointConstraint());
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, ...
    "unsupportedPathConstraints");
end

function testFixedTimeBelowMinimumIsIdentified(testCase)
% Verify an impossible fixed duration remains an expected engine failure.
[initialState, terminalState, limits] = restToRestFixture();
options = struct("TimeMode", "fixed", "FinalTime", 1);
result = ruckigEngine.solve(initialState, terminalState, limits, options);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "fixedTimeBelowMinimum");
end

function testColumnStatesAndScalarLimitsNormalizeIdentically(testCase)
% Verify direct normalization is independent of orientation and expansion.
[initialState, terminalState, limits] = restToRestFixture();
rowResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct());
initialState.position = initialState.position.';
initialState.velocity = initialState.velocity.';
initialState.acceleration = initialState.acceleration.';
terminalState.position = terminalState.position.';
terminalState.velocity = terminalState.velocity.';
terminalState.acceleration = terminalState.acceleration.';
limits.maximumVelocity = 20;
limits.maximumAcceleration = 20;
limits.maximumJerk = 2;
columnResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct());
verifyTrue(testCase, rowResult.Success && columnResult.Success);
verifyEqual(testCase, columnResult.Inputs.initialState.position, [0, 0]);
verifyEqual(testCase, columnResult.Inputs.limits.maximumJerk, [2, 2]);
end

function testResultContainsNoDispatcherProvenance(testCase)
% Verify a direct engine result does not pretend a routing layer exists.
[initialState, terminalState, limits] = restToRestFixture();
result = ruckigEngine.solve(initialState, terminalState, limits, struct());
for fieldName = ["SolverRequested", "SolverUsed", ...
        "FallbackOccurred", "SelectionReason"]
    verifyFalse(testCase, isfield(result, fieldName));
end
end

function testPolynomialEvaluationMatchesReturnedHistories(testCase)
% Verify engine-owned reconstruction agrees at every published sample.
[initialState, terminalState, limits] = restToRestFixture();
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct("SampleTime", 0.01));
[time, position, velocity, acceleration, jerk] = ...
    ruckigEngine.internal.evaluatePolynomial( ...
    result.Polynomial, result.time);
verifyEqual(testCase, time, result.time);
verifyEqual(testCase, position, result.position, "AbsTol", 1e-12);
verifyEqual(testCase, velocity, result.velocity, "AbsTol", 1e-12);
verifyEqual(testCase, acceleration, result.acceleration, "AbsTol", 1e-12);
verifyEqual(testCase, jerk, result.jerk, "AbsTol", 1e-12);
end

function testUnknownOptionsWarnOnceAndAreIgnored(testCase)
% Verify one warning identifies every ignored direct-engine option.
[initialState, terminalState, limits] = restToRestFixture();
options = struct("UnknownOne", 1, "UnknownTwo", 2);
verifyWarning(testCase, @() ruckigEngine.solve( ...
    initialState, terminalState, limits, options), ...
    "ruckigEngine:UnknownOptions");
end

function [initialState, terminalState, limits] = restToRestFixture()
% Create an exact two-axis request with one shared scalar progress law.
initialState = struct( ...
    "time", 0, ...
    "position", [0, 0], ...
    "velocity", [0, 0], ...
    "acceleration", [0, 0]);
terminalState = struct( ...
    "position", [1, -2], ...
    "velocity", [0, 0], ...
    "acceleration", [0, 0], ...
    "maximumTime", 10);
limits = struct( ...
    "maximumVelocity", [10, 20], ...
    "maximumAcceleration", [10, 20], ...
    "maximumJerk", [1, 2]);
end

function pathConstraints = pointConstraint()
% Require a midpoint projection to remain on one side of x=-1.
pathConstraints = struct( ...
    "Tau", 0.5, ...
    "TauEnd", 0.5, ...
    "Normal", [1, 0], ...
    "LowerBound", -1);
end
