function tests = testStandaloneHs3Kernel
%% Section 0: Header & Readme
% SYNTAX
%   tests = testStandaloneHs3Kernel
%**************************************************************************
% PURPOSE
%   - Prove the neutral HS3 leaf kernel matches the embedded two-axis math.
%   - Exercise dimension-neutral reconstruction in one and three dimensions.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Test coordinates use arbitrary consistent units and seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add the repository root so package functions resolve from direct test runs.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testPowerToBernsteinMatchesEmbeddedKernel(testCase)
% Verify the copied basis conversion is exactly unchanged.
powerCoefficient = reshape(sin(1:30), 6, 5);
expected = azElInternal.powerToBernstein(powerCoefficient);
actual = hs3.powerToBernstein(powerCoefficient);
verifyEqual(testCase, actual, expected);
end

function testAffineSensitivityMatchesEmbeddedKernel(testCase)
% Verify every copied scalar-axis sensitivity map is exactly unchanged.
segmentCount = 7;
duration_s = 11.25;
evaluationTau = [0; 0.03; 0.2; 0.51; 0.9; 1];
expected = ...
    azElPlannerMethods.hs3.internal.motion.hs3AffineSensitivity( ...
    segmentCount, duration_s, evaluationTau);
actual = hs3.affineSensitivity( ...
    segmentCount, duration_s, evaluationTau);
verifyEqual(testCase, actual.ControlCount, expected.ControlCount);
verifyEqual(testCase, actual.SegmentDuration, ...
    expected.SegmentDuration_s);
for fieldName = [ ...
        "positionPowerMap", "velocityPowerMap", ...
        "accelerationPowerMap", "jerkPowerMap", ...
        "terminalStateMap", "positionAtTauMap", "velocityAtTauMap"]
    verifyEqual(testCase, actual.(fieldName), expected.(fieldName));
end
end

function testIntegratedJerkMatchesEmbeddedFixedTimeKernel(testCase)
% Verify two-axis objective, gradient, and Hessian remain exact.
segmentCount = 5;
controlCount = 2 * segmentCount + 1;
decision = cos((1:2 * controlCount).');
[expectedValue, expectedGradient, expectedHessian] = ...
    azElPlannerMethods.hs3.internal.motion.integratedSquaredHs3Jerk( ...
    decision, false, 9, segmentCount, 1);
[actualValue, actualGradient, actualHessian] = ...
    hs3.integratedSquaredJerk( ...
    decision, false, 9, segmentCount, 1, 2);
verifyEqual(testCase, actualValue, expectedValue);
verifyEqual(testCase, actualGradient, expectedGradient);
verifyEqual(testCase, actualHessian, expectedHessian);
end

function testIntegratedJerkMatchesEmbeddedFreeTimeKernel(testCase)
% Verify optional final-time decision placement and gradient remain exact.
segmentCount = 4;
controlCount = 2 * segmentCount + 1;
decision = [sin((1:2 * controlCount).'); 8.75];
[expectedValue, expectedGradient] = ...
    azElPlannerMethods.hs3.internal.motion.integratedSquaredHs3Jerk( ...
    decision, true, NaN, segmentCount, 0.5);
[actualValue, actualGradient] = hs3.integratedSquaredJerk( ...
    decision, true, NaN, segmentCount, 0.5, 2);
verifyEqual(testCase, actualValue, expectedValue);
verifyEqual(testCase, actualGradient, expectedGradient);
end

function testReconstructionAndEvaluationSupportOneDimension(testCase)
% Verify the extracted integrator has no two-axis assumption.
verifyDimensionNeutralReconstruction(testCase, 1);
end

function testReconstructionAndEvaluationSupportThreeDimensions(testCase)
% Verify the extracted integrator supports a Cartesian-sized state.
verifyDimensionNeutralReconstruction(testCase, 3);
end

function testFixedTimeOneDimensionalSlew(testCase)
% Verify the public engine solves and validates a one-coordinate request.
[initialState, terminalState, limits] = fixedRestToRestProblem(1);
options = struct( ...
    "TimeMode", "fixed", "FinalTime", 4, ...
    "SegmentCount", 6, "SampleTime", 0.1);
trajectory = hs3.solve(initialState, terminalState, limits, options);
verifyTrue(testCase, trajectory.Success, trajectory.Message);
verifyTrue(testCase, trajectory.Validation.Passed, ...
    trajectory.Validation.Message);
verifySize(testCase, trajectory.position, [numel(trajectory.time), 1]);
end

function testFixedTimeSupportsNonzeroBoundaryDerivatives(testCase)
% Verify exact nonzero velocity and acceleration boundary states in two axes.
initialState = struct( ...
    "time", 1, "position", [0 1], ...
    "velocity", [0.2 -0.1], "acceleration", [0.05 0.02]);
duration = 3;
terminalState = struct( ...
    "position", initialState.position + duration * initialState.velocity + ...
    0.5 * duration^2 * initialState.acceleration, ...
    "velocity", initialState.velocity + ...
    duration * initialState.acceleration, ...
    "acceleration", initialState.acceleration, ...
    "maximumTime", initialState.time + duration);
limits = struct( ...
    "maximumVelocity", [2 3], ...
    "maximumAcceleration", [1 1.5], ...
    "maximumJerk", [2 4]);
options = struct( ...
    "TimeMode", "fixed", "FinalTime", terminalState.maximumTime, ...
    "SegmentCount", 5);
trajectory = hs3.solve(initialState, terminalState, limits, options);
verifyTrue(testCase, trajectory.Success, trajectory.Message);
verifyEqual(testCase, trajectory.ControlJerk, ...
    zeros(size(trajectory.ControlJerk)), "AbsTol", 1e-10);
end

function testFixedTimeSupportsThreeDimensionalAsymmetricLimits(testCase)
% Verify independent per-coordinate limits in three dimensions.
[initialState, terminalState, limits] = fixedRestToRestProblem(3);
limits.maximumVelocity = [0.8 1.4 2.2];
limits.maximumAcceleration = [0.7 1.1 1.8];
limits.maximumJerk = [1.5 2.5 4];
terminalState.position = [0.5 -0.75 1];
terminalState.maximumTime = 6;
options = struct( ...
    "TimeMode", "fixed", "FinalTime", 6, "SegmentCount", 7);
trajectory = hs3.solve(initialState, terminalState, limits, options);
verifyTrue(testCase, trajectory.Success, trajectory.Message);
verifySize(testCase, trajectory.position, [numel(trajectory.time), 3]);
end

function testEarliestArrivalSupportsThreeDimensions(testCase)
% Verify free final time is solved without an az/el or obstacle dependency.
[initialState, terminalState, limits] = fixedRestToRestProblem(3);
terminalState.position = [1 -1.5 0.6];
terminalState.maximumTime = 10;
limits.maximumVelocity = [2 2.5 1.5];
limits.maximumAcceleration = [2 2 1.5];
limits.maximumJerk = [5 5 4];
options = struct( ...
    "TimeMode", "earliestArrival", "SegmentCount", 6, ...
    "SampleTime", 0.1, "MaximumIterations", 250);
trajectory = hs3.solve(initialState, terminalState, limits, options);
verifyTrue(testCase, trajectory.Success, trajectory.Message);
verifyLessThan(testCase, trajectory.FinalTime, terminalState.maximumTime);
verifyTrue(testCase, trajectory.Validation.Passed, ...
    trajectory.Validation.Message);
end

function testInfeasibleFixedDurationReturnsFailure(testCase)
% Verify a physically impossible duration is reported rather than clipped.
[initialState, terminalState, limits] = fixedRestToRestProblem(1);
terminalState.position = 10;
terminalState.maximumTime = 0.1;
limits.maximumVelocity = 1;
limits.maximumAcceleration = 1;
limits.maximumJerk = 1;
trajectory = hs3.solve(initialState, terminalState, limits, struct( ...
    "TimeMode", "fixed", "FinalTime", 0.1, "SegmentCount", 4));
verifyFalse(testCase, trajectory.Success);
verifyNotEqual(testCase, trajectory.TerminationReason, "goalReached");
verifyGreaterThan(testCase, trajectory.MaximumConstraintViolation, 1e-7);
end

function testOptionalAffinePathConstraintIsEnforced(testCase)
% Verify the engine consumes only neutral coordinate-space path half-spaces.
[initialState, terminalState, limits] = fixedRestToRestProblem(2);
terminalState.position = [1 0.5];
terminalState.maximumTime = 5;
pathConstraints = struct( ...
    "Tau", [0.35; 0.65], ...
    "Normal", [1 0; -1 0], ...
    "LowerBound", [0.08; -0.92]);
trajectory = hs3.solve(initialState, terminalState, limits, struct( ...
    "TimeMode", "fixed", "FinalTime", 5, "SegmentCount", 6), ...
    pathConstraints);
verifyTrue(testCase, trajectory.Success, trajectory.Message);
constraintTime = initialState.time + ...
    pathConstraints.Tau * trajectory.Duration;
[~, position] = hs3.evaluatePolynomial( ...
    trajectory.Polynomial, constraintTime);
verifyGreaterThanOrEqual(testCase, ...
    sum(pathConstraints.Normal .* position, 2), ...
    pathConstraints.LowerBound - 1e-7);
end

function testSubintervalHullMatchesEmbeddedGenericMath(testCase)
% Verify the neutral interval map is an exact extraction of shared math.
tauStart = [0.02; 0.2; 0.51; 0.8];
tauEnd = [0.1; 0.3; 0.6; 0.8];
[expectedSegment, expectedMap] = azElInternal.subintervalHullMap( ...
    tauStart, tauEnd, 5, 6);
[actualSegment, actualMap] = hs3.subintervalHullMap( ...
    tauStart, tauEnd, 5, 6);
verifyEqual(testCase, actualSegment, expectedSegment);
verifyEqual(testCase, actualMap, expectedMap);
end

function testAffineIntervalConstraintUsesCompleteHull(testCase)
% Verify an interval row expands to every position Bernstein coefficient.
[initialState, terminalState, limits] = fixedRestToRestProblem(2);
terminalState.position = [1 0.5];
terminalState.maximumTime = 5;
segmentCount = 6;
pathConstraints = struct( ...
    "Tau", 0.2, "TauEnd", 0.3, ...
    "Normal", [1 0], "LowerBound", -0.05);
trajectory = hs3.solve(initialState, terminalState, limits, struct( ...
    "TimeMode", "fixed", "FinalTime", 5, ...
    "SegmentCount", segmentCount), pathConstraints);
verifyTrue(testCase, trajectory.Success, trajectory.Message);
[segmentIndex, hullMap] = hs3.subintervalHullMap( ...
    pathConstraints.Tau, pathConstraints.TauEnd, segmentCount, 6);
positionPower = reshape(trajectory.Polynomial.positionPower( ...
    segmentIndex, :, :), 2, 6);
projectionHull = hullMap(:, :, 1) * ...
    (pathConstraints.Normal * positionPower).';
verifyGreaterThanOrEqual(testCase, projectionHull, ...
    pathConstraints.LowerBound - 1e-7);
end

function testDefaultsAndPartialOverridesShareOneOwner(testCase)
% Verify zero-input defaults and empty partial fields resolve consistently.
defaults = hs3.solve();
verifyEqual(testCase, defaults, hs3.defaultOptions());
[initialState, terminalState, limits] = fixedRestToRestProblem(2);
trajectory = hs3.solve(initialState, terminalState, limits, struct( ...
    "TimeMode", "fixed", "FinalTime", terminalState.maximumTime, ...
    "SegmentCount", [], "Verbose", 0));
verifyTrue(testCase, trajectory.Success, trajectory.Message);
verifyEqual(testCase, trajectory.Options.SegmentCount, defaults.SegmentCount);
verifyFalse(testCase, trajectory.Options.Verbose);
end

function testUnknownOptionsWarnOnceAndDoNotChangeBehavior(testCase)
% Verify all unknown fields produce one warning and are excluded from output.
[initialState, terminalState, limits] = fixedRestToRestProblem(1);
overrides = struct( ...
    "TimeMode", "fixed", "FinalTime", terminalState.maximumTime, ...
    "FirstUnknown", 1, "SecondUnknown", 2);
verifyWarning(testCase, @() hs3.solve( ...
    initialState, terminalState, limits, overrides), ...
    "solve:UnknownOptions");
lastwarn("");
trajectory = hs3.solve(initialState, terminalState, limits, overrides);
[~, warningIdentifier] = lastwarn;
verifyEqual(testCase, string(warningIdentifier), "solve:UnknownOptions");
verifyFalse(testCase, isfield(trajectory.Options, "FirstUnknown"));
verifyFalse(testCase, isfield(trajectory.Options, "SecondUnknown"));
end

function testSuccessAndFailureReturnTheSameSchema(testCase)
% Verify expected infeasibility does not drop public result fields.
[initialState, terminalState, limits] = fixedRestToRestProblem(1);
success = hs3.solve(initialState, terminalState, limits, struct( ...
    "TimeMode", "fixed", "FinalTime", terminalState.maximumTime));
terminalState.position = 10;
terminalState.maximumTime = 0.1;
failure = hs3.solve(initialState, terminalState, limits, struct( ...
    "TimeMode", "fixed", "FinalTime", terminalState.maximumTime));
verifyTrue(testCase, success.Success);
verifyFalse(testCase, failure.Success);
verifyEqual(testCase, fieldnames(failure), fieldnames(success));
end

function testPackageSourceHasNoPlannerDependencies(testCase)
% Verify the standalone source contains none of the forbidden planner terms.
repositoryRoot = testCase.TestData.RepositoryRoot;
packageFiles = dir(fullfile(repositoryRoot, "+hs3", "*.m"));
forbiddenTerms = [ ...
    "azelinternal", "azelplannermethods", "azimuth", "elevation", ...
    "obstacle", "visibility", "homology", "seed", "plot", "planner"];
for fileIndex = 1:numel(packageFiles)
    source = lower(string(fileread(fullfile( ...
        packageFiles(fileIndex).folder, packageFiles(fileIndex).name))));
    for term = forbiddenTerms
        verifyFalse(testCase, contains(source, term), ...
            packageFiles(fileIndex).name + ...
            " contains forbidden planner term '" + term + "'.");
    end
    verifyEmpty(testCase, regexp(source, ...
        "(^|\\n)\\s*(rng|rand|randn|randi)\\s*\\(", "once"), ...
        packageFiles(fileIndex).name + " introduces random draw ordering.");
end
end

function verifyDimensionNeutralReconstruction(testCase, dimensionCount)
% Reconstruct and independently evaluate one arbitrary D-dimensional chain.
segmentCount = 4;
controlCount = 2 * segmentCount + 1;
controlJerk = reshape( ...
    sin(1:controlCount * dimensionCount), controlCount, dimensionCount);
initialState = struct( ...
    "time", 0.75, ...
    "position", linspace(-1, 1, dimensionCount), ...
    "velocity", linspace(0.2, -0.1, dimensionCount), ...
    "acceleration", linspace(-0.3, 0.4, dimensionCount));
finalTime = 6.25;
polynomial = hs3.reconstructPolynomial( ...
    controlJerk, initialState, finalTime, segmentCount);
sampleTime = unique([linspace(initialState.time, finalTime, 31).'; ...
    polynomial.SegmentStartTime; finalTime]);
[time, position, velocity, acceleration, jerk] = ...
    hs3.evaluatePolynomial(polynomial, sampleTime);
verifyEqual(testCase, time, sampleTime);
verifySize(testCase, position, [numel(sampleTime), dimensionCount]);
verifySize(testCase, velocity, [numel(sampleTime), dimensionCount]);
verifySize(testCase, acceleration, [numel(sampleTime), dimensionCount]);
verifySize(testCase, jerk, [numel(sampleTime), dimensionCount]);
verifyEqual(testCase, position(1, :), initialState.position, ...
    "AbsTol", 1e-14);
verifyEqual(testCase, velocity(1, :), initialState.velocity, ...
    "AbsTol", 1e-14);
verifyEqual(testCase, acceleration(1, :), initialState.acceleration, ...
    "AbsTol", 1e-14);
verifyEqual(testCase, position(end, :), ...
    polynomial.TerminalState.position, "AbsTol", 1e-13);
verifyEqual(testCase, velocity(end, :), ...
    polynomial.TerminalState.velocity, "AbsTol", 1e-13);
verifyEqual(testCase, acceleration(end, :), ...
    polynomial.TerminalState.acceleration, "AbsTol", 1e-13);
end

function [initialState, terminalState, limits] = ...
        fixedRestToRestProblem(dimensionCount)
% Construct one neutral rest-to-rest problem with dimension-scaled endpoints.
initialState = struct( ...
    "time", 0, ...
    "position", zeros(1, dimensionCount), ...
    "velocity", zeros(1, dimensionCount), ...
    "acceleration", zeros(1, dimensionCount));
terminalState = struct( ...
    "position", linspace(0.5, 1, dimensionCount), ...
    "velocity", zeros(1, dimensionCount), ...
    "acceleration", zeros(1, dimensionCount), ...
    "maximumTime", 4);
limits = struct( ...
    "maximumVelocity", 2, ...
    "maximumAcceleration", 2, ...
    "maximumJerk", 4);
end
