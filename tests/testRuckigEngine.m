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

function testSatisfiedPathConstraintIsCertified(testCase)
% Verify a satisfied affine row participates in continuous certification.
[initialState, terminalState, limits] = restToRestFixture();
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct(), pointConstraint());
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.ConstraintPassed);
end

function testViolatedPathConstraintIsReported(testCase)
% Verify exact profile construction never hides an affine path violation.
[initialState, terminalState, limits] = restToRestFixture();
pathConstraints = pointConstraint();
pathConstraints.LowerBound = 2;
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct(), pathConstraints);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, ...
    "pathConstraintViolation");
verifyGreaterThan(testCase, ...
    result.Validation.MaximumInequalityViolation, 0);
end

function testIntervalPathConstraintUsesContinuousHull(testCase)
% Verify one interval row constrains the complete projected subtrajectory.
[initialState, terminalState, limits] = restToRestFixture();
pathConstraints = struct( ...
    "Tau", 0.2, ...
    "TauEnd", 0.8, ...
    "Normal", [1, 0], ...
    "LowerBound", 0.01);
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct(), pathConstraints);
verifyTrue(testCase, result.Success, result.Message);
pathConstraints.LowerBound = 0.9;
violatingResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct(), pathConstraints);
verifyFalse(testCase, violatingResult.Success);
verifyEqual(testCase, violatingResult.TerminationReason, ...
    "pathConstraintViolation");
end

function testIntervalPathConstraintSpansPolynomialSegments(testCase)
% Detect a violation after the first boundary of one requested interval.
[initialState, terminalState, limits] = restToRestFixture();
pathConstraints = struct( ...
    "Tau", 0.1, "TauEnd", 0.8, ...
    "Normal", [-1, 0], "LowerBound", -0.2);
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct(), pathConstraints);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, ...
    "pathConstraintViolation");
verifyGreaterThan(testCase, ...
    result.Validation.MaximumInequalityViolation, 0);
end

function testFixedTimeBelowMinimumIsIdentified(testCase)
% Verify an impossible fixed duration remains an expected engine failure.
[initialState, terminalState, limits] = restToRestFixture();
options = struct("TimeMode", "fixed", "FinalTime", 1);
result = ruckigEngine.solve(initialState, terminalState, limits, options);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "fixedTimeBelowMinimum");
end

function testEarliestArrivalAcceptsExactAndInsideHorizon(testCase)
% Accept profiles at the horizon and within one arrival tolerance before it.
[initialState, terminalState, limits] = restToRestFixture();
minimumDuration = 4 * nthroot(1 / 2, 3);
arrivalTolerance = 1e-6;
options = struct("ArrivalTimeTolerance", arrivalTolerance);
terminalState.maximumTime = minimumDuration;
exactResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, options);
terminalState.maximumTime = minimumDuration + arrivalTolerance / 2;
insideResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, options);
verifyTrue(testCase, exactResult.Success, exactResult.Message);
verifyTrue(testCase, insideResult.Success, insideResult.Message);
end

function testWrapperRejectsUnsupportedArities(testCase)
% Return the public InvalidCall error before referencing missing arguments.
verifyError(testCase, @() planTrajRuckig(struct()), ...
    "planTrajRuckig:InvalidCall");
verifyError(testCase, @() planTrajRuckig(struct(), struct()), ...
    "planTrajRuckig:InvalidCall");
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
